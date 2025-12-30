//! ICM-20948 I2C Driver Implementation
//!
//! Core driver implementation for reading IMU data via I2C.
//! The ICM-20948 uses a 4-bank register architecture requiring
//! bank switching for configuration access.
//!
//! This driver is platform-agnostic and works with any `embedded_hal_async::i2c::I2c`
//! implementation. Time operations require the `embassy` feature.

use super::config::{AccelRange, GyroRange, Icm20948Config, RegisterBank};
use super::registers::{self, TEMP_OFFSET, TEMP_SENSITIVITY};
use crate::devices::traits::{ImuCalibration, ImuError, ImuReading, ImuSensor};
use embedded_hal_async::i2c::I2c;
use nalgebra::Vector3;

/// Maximum consecutive errors before marking sensor unhealthy
const MAX_CONSECUTIVE_ERRORS: u32 = 3;

// =============================================================================
// Time Abstraction Helpers
// =============================================================================

/// Async delay in milliseconds
///
/// Uses `embassy_time::Timer` when the `embassy` feature is enabled.
/// No-op for host tests without embassy.
#[cfg(feature = "embassy")]
async fn delay_ms(ms: u64) {
    embassy_time::Timer::after_millis(ms).await;
}

#[cfg(not(feature = "embassy"))]
async fn delay_ms(_ms: u64) {
    // No-op for host tests
}

/// Get current timestamp in microseconds
///
/// Returns actual timestamp when `embassy` feature is enabled.
/// Returns 0 for host tests without embassy.
#[cfg(feature = "embassy")]
fn timestamp_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}

#[cfg(not(feature = "embassy"))]
fn timestamp_us() -> u64 {
    0 // Host test stub
}

/// ICM-20948 I2C Driver
///
/// Implements `ImuSensor` trait for the ICM-20948 9-axis IMU.
/// Uses I2C bypass mode for AK09916 magnetometer access.
///
/// # Type Parameters
///
/// * `I2C` - Any type implementing `embedded_hal_async::i2c::I2c`
pub struct Icm20948Driver<I2C>
where
    I2C: I2c,
{
    /// I2C bus handle
    i2c: I2C,

    /// Driver configuration
    config: Icm20948Config,

    /// Calibration data
    calibration: ImuCalibration,

    /// Gyro scale factor (raw to rad/s)
    gyro_scale: f32,

    /// Accel scale factor (raw to m/s²)
    accel_scale: f32,

    /// Current register bank
    current_bank: RegisterBank,

    /// Health status
    healthy: bool,

    /// Consecutive error count
    error_count: u32,

    /// Initialization complete flag
    initialized: bool,
}

impl<I2C> Icm20948Driver<I2C>
where
    I2C: I2c,
{
    /// Create a new ICM-20948 driver (uninitialized)
    ///
    /// Call `init()` to initialize the sensor before use.
    ///
    /// # Arguments
    ///
    /// * `i2c` - I2C bus implementing `embedded_hal_async::i2c::I2c`
    /// * `config` - Driver configuration
    pub fn new(i2c: I2C, config: Icm20948Config) -> Self {
        Self {
            i2c,
            config,
            calibration: ImuCalibration::default(),
            gyro_scale: config.gyro_range.scale_to_rad_s(),
            accel_scale: config.accel_range.scale_to_m_s2(),
            current_bank: RegisterBank::Bank0,
            healthy: false,
            error_count: 0,
            initialized: false,
        }
    }

    /// Create and initialize a new ICM-20948 driver
    ///
    /// # Arguments
    ///
    /// * `i2c` - I2C bus implementing `embedded_hal_async::i2c::I2c`
    /// * `config` - Driver configuration
    ///
    /// # Returns
    ///
    /// Initialized driver or error if initialization failed
    pub async fn new_initialized(i2c: I2C, config: Icm20948Config) -> Result<Self, ImuError> {
        let mut driver = Self::new(i2c, config);
        driver.init().await?;
        Ok(driver)
    }

    /// Initialize the ICM-20948
    ///
    /// This performs the full initialization sequence:
    /// 1. Verify WHO_AM_I
    /// 2. Reset device
    /// 3. Configure gyro/accel
    /// 4. Enable I2C bypass for magnetometer
    ///
    /// # Returns
    ///
    /// Ok(()) if initialization succeeded, or ImuError on failure
    pub async fn init(&mut self) -> Result<(), ImuError> {
        // Step 1: Ensure we're on Bank 0
        self.select_bank(RegisterBank::Bank0).await?;

        // Step 2: Verify WHO_AM_I
        let whoami = self.read_register(registers::WHO_AM_I).await?;
        if whoami != registers::ICM20948_WHO_AM_I_VALUE {
            crate::log_error!(
                "ICM-20948 WHO_AM_I mismatch: expected {:#x}, got {:#x}",
                registers::ICM20948_WHO_AM_I_VALUE,
                whoami
            );
            return Err(ImuError::NotInitialized);
        }
        crate::log_info!("ICM-20948 detected (WHO_AM_I: {:#x})", whoami);

        // Step 3: Reset device
        self.write_register(registers::PWR_MGMT_1, registers::PWR_MGMT_1_DEVICE_RESET)
            .await?;
        delay_ms(100).await;

        // After reset, bank is reset to 0
        self.current_bank = RegisterBank::Bank0;

        // Step 4: Wake up device with auto clock select
        self.write_register(registers::PWR_MGMT_1, registers::PWR_MGMT_1_CLKSEL_AUTO)
            .await?;
        delay_ms(10).await;

        // Step 5: Enable all sensors
        self.write_register(registers::PWR_MGMT_2, registers::PWR_MGMT_2_ENABLE_ALL)
            .await?;

        // Step 6: Configure gyroscope (Bank 2)
        self.select_bank(RegisterBank::Bank2).await?;

        // Set gyro sample rate divider
        self.write_register(registers::GYRO_SMPLRT_DIV, self.config.gyro_sample_rate_div)
            .await?;

        // Set gyro range and DLPF
        let gyro_config =
            self.config.gyro_range.register_value() | self.config.gyro_dlpf.register_value();
        self.write_register(registers::GYRO_CONFIG_1, gyro_config)
            .await?;

        // Step 7: Configure accelerometer (Bank 2)
        // Set accel sample rate divider (12-bit, split across two registers)
        self.write_register(
            registers::ACCEL_SMPLRT_DIV_1,
            ((self.config.accel_sample_rate_div >> 8) & 0x0F) as u8,
        )
        .await?;
        self.write_register(
            registers::ACCEL_SMPLRT_DIV_2,
            (self.config.accel_sample_rate_div & 0xFF) as u8,
        )
        .await?;

        // Set accel range and DLPF
        let accel_config =
            self.config.accel_range.register_value() | self.config.accel_dlpf.register_value();
        self.write_register(registers::ACCEL_CONFIG, accel_config)
            .await?;

        // Step 8: Return to Bank 0 and enable I2C bypass for magnetometer
        self.select_bank(RegisterBank::Bank0).await?;

        // Disable I2C master mode (required for bypass)
        self.write_register(registers::USER_CTRL, 0x00).await?;
        delay_ms(10).await;

        // Enable I2C bypass mode
        self.write_register(registers::INT_PIN_CFG, registers::INT_PIN_CFG_BYPASS_EN)
            .await?;
        delay_ms(10).await;

        // Verify bypass mode is enabled
        let int_cfg = self.read_register(registers::INT_PIN_CFG).await?;
        if (int_cfg & registers::INT_PIN_CFG_BYPASS_EN) == 0 {
            crate::log_error!("Failed to enable I2C bypass mode");
            return Err(ImuError::NotInitialized);
        }
        crate::log_debug!("I2C bypass mode enabled (INT_PIN_CFG: {:#x})", int_cfg);

        self.initialized = true;
        self.healthy = true;
        crate::log_info!("ICM-20948 initialized successfully");

        Ok(())
    }

    /// Select a register bank
    ///
    /// The ICM-20948 has 4 register banks. Bank selection is required
    /// before accessing registers in a different bank.
    pub async fn select_bank(&mut self, bank: RegisterBank) -> Result<(), ImuError> {
        if self.current_bank != bank {
            // REG_BANK_SEL is at 0x7F in all banks
            self.write_register_direct(registers::REG_BANK_SEL, bank.register_value())
                .await?;
            self.current_bank = bank;
        }
        Ok(())
    }

    /// Read a register from ICM-20948 (direct, no bank check)
    async fn read_register_direct(&mut self, reg: u8) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.config.i2c_address, &[reg], &mut buf)
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;
        Ok(buf[0])
    }

    /// Read a register from ICM-20948 (assumes correct bank is selected)
    async fn read_register(&mut self, reg: u8) -> Result<u8, ImuError> {
        self.read_register_direct(reg).await
    }

    /// Write a register to ICM-20948 (direct, no bank check)
    async fn write_register_direct(&mut self, reg: u8, value: u8) -> Result<(), ImuError> {
        self.i2c
            .write(self.config.i2c_address, &[reg, value])
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;
        Ok(())
    }

    /// Write a register to ICM-20948 (assumes correct bank is selected)
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), ImuError> {
        self.write_register_direct(reg, value).await
    }

    /// Read multiple bytes from ICM-20948
    async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), ImuError> {
        self.i2c
            .write_read(self.config.i2c_address, &[reg], buf)
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;
        Ok(())
    }

    /// Get the current register bank
    pub fn current_bank(&self) -> RegisterBank {
        self.current_bank
    }

    /// Check if the driver is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Check if the sensor is healthy
    pub fn is_healthy(&self) -> bool {
        self.initialized && self.healthy
    }

    /// Get the I2C address
    pub fn address(&self) -> u8 {
        self.config.i2c_address
    }

    /// Get the configuration
    pub fn config(&self) -> &Icm20948Config {
        &self.config
    }

    /// Get the calibration data
    pub fn calibration(&self) -> &ImuCalibration {
        &self.calibration
    }

    /// Set calibration data
    pub fn set_calibration(&mut self, calibration: ImuCalibration) {
        self.calibration = calibration;
    }

    // =========================================================================
    // Sensor Reading Methods
    // =========================================================================

    /// Read raw gyroscope and accelerometer data (12 bytes)
    ///
    /// Returns (gyro_raw, accel_raw) as raw i16 values.
    /// Sensor data registers are in Bank 0.
    pub async fn read_gyro_accel_raw(&mut self) -> Result<([i16; 3], [i16; 3]), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Ensure we're on Bank 0 for sensor data
        self.select_bank(RegisterBank::Bank0).await?;

        // Read 12 bytes: ACCEL_XOUT_H through GYRO_ZOUT_L
        // ICM-20948 data order: Accel (6 bytes), Gyro (6 bytes)
        let mut buf = [0u8; 12];
        self.read_bytes(registers::ACCEL_XOUT_H, &mut buf).await?;

        // Parse big-endian 16-bit values
        let accel = [
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ];
        let gyro = [
            i16::from_be_bytes([buf[6], buf[7]]),
            i16::from_be_bytes([buf[8], buf[9]]),
            i16::from_be_bytes([buf[10], buf[11]]),
        ];

        Ok((gyro, accel))
    }

    /// Read raw temperature data
    ///
    /// Returns raw i16 temperature value.
    pub async fn read_temp_raw(&mut self) -> Result<i16, ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Ensure we're on Bank 0 for sensor data
        self.select_bank(RegisterBank::Bank0).await?;

        // Read 2 bytes: TEMP_OUT_H, TEMP_OUT_L
        let mut buf = [0u8; 2];
        self.read_bytes(registers::TEMP_OUT_H, &mut buf).await?;

        Ok(i16::from_be_bytes([buf[0], buf[1]]))
    }

    /// Read raw gyroscope, accelerometer, and temperature data (14 bytes)
    ///
    /// Returns (gyro_raw, accel_raw, temp_raw) as raw values.
    /// This is more efficient than separate reads as it uses a single I2C transaction.
    pub async fn read_gyro_accel_temp_raw(
        &mut self,
    ) -> Result<([i16; 3], [i16; 3], i16), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Ensure we're on Bank 0 for sensor data
        self.select_bank(RegisterBank::Bank0).await?;

        // Read 14 bytes: ACCEL (6) + GYRO (6) + TEMP (2)
        // ICM-20948 register layout: ACCEL_XOUT_H (0x2D) through TEMP_OUT_L (0x3A)
        let mut buf = [0u8; 14];
        self.read_bytes(registers::ACCEL_XOUT_H, &mut buf).await?;

        // Parse big-endian 16-bit values
        let accel = [
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ];
        let gyro = [
            i16::from_be_bytes([buf[6], buf[7]]),
            i16::from_be_bytes([buf[8], buf[9]]),
            i16::from_be_bytes([buf[10], buf[11]]),
        ];
        let temp = i16::from_be_bytes([buf[12], buf[13]]);

        Ok((gyro, accel, temp))
    }

    // =========================================================================
    // Unit Conversion Methods
    // =========================================================================

    /// Convert raw gyroscope values to rad/s with calibration applied
    pub fn convert_gyro(&self, raw: [i16; 3]) -> Vector3<f32> {
        let raw_vec = Vector3::new(raw[0] as f32, raw[1] as f32, raw[2] as f32);
        let scaled = raw_vec * self.gyro_scale;
        self.calibration.apply_gyro(scaled)
    }

    /// Convert raw accelerometer values to m/s² with calibration applied
    pub fn convert_accel(&self, raw: [i16; 3]) -> Vector3<f32> {
        let raw_vec = Vector3::new(raw[0] as f32, raw[1] as f32, raw[2] as f32);
        let scaled = raw_vec * self.accel_scale;
        self.calibration.apply_accel(scaled)
    }

    /// Convert raw temperature to °C
    ///
    /// ICM-20948 temperature formula: Temp_degC = ((raw - RoomTemp_Offset) / Sensitivity) + 21
    /// Where RoomTemp_Offset = 0 and Sensitivity = 333.87 LSB/°C
    pub fn convert_temp(raw: i16) -> f32 {
        (raw as f32 / TEMP_SENSITIVITY) + TEMP_OFFSET
    }

    // =========================================================================
    // High-Level Reading Methods
    // =========================================================================

    /// Read gyroscope in rad/s (calibrated)
    pub async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError> {
        let (gyro_raw, _) = self.read_gyro_accel_raw().await?;
        Ok(self.convert_gyro(gyro_raw))
    }

    /// Read accelerometer in m/s² (calibrated)
    pub async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError> {
        let (_, accel_raw) = self.read_gyro_accel_raw().await?;
        Ok(self.convert_accel(accel_raw))
    }

    /// Read temperature in °C
    pub async fn read_temp(&mut self) -> Result<f32, ImuError> {
        let temp_raw = self.read_temp_raw().await?;
        Ok(Self::convert_temp(temp_raw))
    }

    /// Get current timestamp in microseconds
    pub fn timestamp_us() -> u64 {
        timestamp_us()
    }

    // =========================================================================
    // Configuration Methods
    // =========================================================================

    /// Reconfigure gyroscope range
    ///
    /// Note: This only updates the local scale factor. Call `apply_config()` to
    /// write the new configuration to the sensor.
    pub fn set_gyro_range(&mut self, range: GyroRange) {
        self.config.gyro_range = range;
        self.gyro_scale = range.scale_to_rad_s();
    }

    /// Reconfigure accelerometer range
    ///
    /// Note: This only updates the local scale factor. Call `apply_config()` to
    /// write the new configuration to the sensor.
    pub fn set_accel_range(&mut self, range: AccelRange) {
        self.config.accel_range = range;
        self.accel_scale = range.scale_to_m_s2();
    }

    /// Apply current configuration to the sensor
    ///
    /// Writes gyro and accel configuration to Bank 2, then returns to Bank 0.
    pub async fn apply_config(&mut self) -> Result<(), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Switch to Bank 2 for sensor configuration
        self.select_bank(RegisterBank::Bank2).await?;

        // Set gyro sample rate divider
        self.write_register(registers::GYRO_SMPLRT_DIV, self.config.gyro_sample_rate_div)
            .await?;

        // Set gyro range and DLPF
        let gyro_config =
            self.config.gyro_range.register_value() | self.config.gyro_dlpf.register_value();
        self.write_register(registers::GYRO_CONFIG_1, gyro_config)
            .await?;

        // Set accel sample rate divider
        self.write_register(
            registers::ACCEL_SMPLRT_DIV_1,
            ((self.config.accel_sample_rate_div >> 8) & 0x0F) as u8,
        )
        .await?;
        self.write_register(
            registers::ACCEL_SMPLRT_DIV_2,
            (self.config.accel_sample_rate_div & 0xFF) as u8,
        )
        .await?;

        // Set accel range and DLPF
        let accel_config =
            self.config.accel_range.register_value() | self.config.accel_dlpf.register_value();
        self.write_register(registers::ACCEL_CONFIG, accel_config)
            .await?;

        // Return to Bank 0
        self.select_bank(RegisterBank::Bank0).await?;

        Ok(())
    }

    // =========================================================================
    // Magnetometer (AK09916) Methods
    // =========================================================================

    /// Initialize the AK09916 magnetometer
    ///
    /// This requires I2C bypass mode to be enabled (done in `init()`).
    /// The AK09916 is accessible at address 0x0C via the I2C bus.
    ///
    /// # Returns
    ///
    /// Ok(()) if initialization succeeded, or ImuError on failure
    pub async fn init_magnetometer(&mut self) -> Result<(), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Verify AK09916 WHO_AM_I (WIA2 register at 0x01)
        let whoami = self.read_mag_register(registers::AK09916_WIA2).await?;
        if whoami != registers::AK09916_WHO_AM_I_VALUE {
            crate::log_error!(
                "AK09916 WHO_AM_I mismatch: expected {:#x}, got {:#x}",
                registers::AK09916_WHO_AM_I_VALUE,
                whoami
            );
            return Err(ImuError::MagInitError);
        }
        crate::log_info!("AK09916 magnetometer detected (WHO_AM_I: {:#x})", whoami);

        // Soft reset
        self.write_mag_register(registers::AK09916_CNTL3, registers::AK09916_CNTL3_SRST)
            .await?;
        delay_ms(10).await;

        // Set continuous measurement mode at 100Hz
        self.write_mag_register(registers::AK09916_CNTL2, registers::AK09916_MODE_CONT_100HZ)
            .await?;
        delay_ms(10).await;

        crate::log_info!("AK09916 configured for 100Hz continuous mode");
        Ok(())
    }

    /// Read a register from the AK09916 magnetometer
    ///
    /// The magnetometer is accessed directly via I2C at address 0x0C
    /// when bypass mode is enabled.
    async fn read_mag_register(&mut self, reg: u8) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(registers::AK09916_ADDR, &[reg], &mut buf)
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;
        Ok(buf[0])
    }

    /// Write a register to the AK09916 magnetometer
    async fn write_mag_register(&mut self, reg: u8, value: u8) -> Result<(), ImuError> {
        self.i2c
            .write(registers::AK09916_ADDR, &[reg, value])
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;
        Ok(())
    }

    /// Read raw magnetometer data from AK09916
    ///
    /// Returns raw 16-bit magnetometer values (X, Y, Z).
    /// Returns `Err(ImuError::MagNotReady)` if data is not ready.
    /// Returns `Err(ImuError::MagOverflow)` if magnetic sensor overflowed.
    pub async fn read_mag_raw(&mut self) -> Result<[i16; 3], ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Read status 1 to check data ready
        let st1 = self.read_mag_register(registers::AK09916_ST1).await?;
        if (st1 & registers::AK09916_ST1_DRDY) == 0 {
            return Err(ImuError::MagNotReady);
        }

        // Read all magnetometer data (8 bytes: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2)
        // We need to read ST2 to complete the measurement cycle
        let mut buf = [0u8; 8];
        self.i2c
            .write_read(registers::AK09916_ADDR, &[registers::AK09916_ST1], &mut buf)
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;

        // Check overflow in ST2 (byte 7)
        if (buf[7] & registers::AK09916_ST2_HOFL) != 0 {
            return Err(ImuError::MagOverflow);
        }

        // Parse little-endian 16-bit values (AK09916 uses little-endian)
        // Data layout: ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2
        let mag = [
            i16::from_le_bytes([buf[1], buf[2]]), // X
            i16::from_le_bytes([buf[3], buf[4]]), // Y
            i16::from_le_bytes([buf[5], buf[6]]), // Z
        ];

        Ok(mag)
    }

    /// Convert raw magnetometer values to µT (microtesla)
    ///
    /// AK09916 sensitivity: 0.15 µT/LSB
    pub fn convert_mag(&self, raw: [i16; 3]) -> Vector3<f32> {
        let raw_vec = Vector3::new(raw[0] as f32, raw[1] as f32, raw[2] as f32);
        let scaled = raw_vec * registers::MAG_SENSITIVITY;
        self.calibration.apply_mag(scaled)
    }

    /// Read magnetometer in µT (microtesla, calibrated)
    ///
    /// Returns `Err(ImuError::MagNotReady)` if data is not ready.
    /// Returns `Err(ImuError::MagOverflow)` if magnetic sensor overflowed.
    pub async fn read_mag(&mut self) -> Result<Vector3<f32>, ImuError> {
        let mag_raw = self.read_mag_raw().await?;
        Ok(self.convert_mag(mag_raw))
    }

    /// Read all 9-axis data (gyro, accel, mag) with optional magnetometer
    ///
    /// This reads gyro and accel from ICM-20948, and optionally magnetometer
    /// from AK09916 if it's ready. If magnetometer is not ready, returns None
    /// for the mag component.
    pub async fn read_all_raw(
        &mut self,
    ) -> Result<([i16; 3], [i16; 3], Option<[i16; 3]>), ImuError> {
        // Read gyro and accel
        let (gyro, accel) = self.read_gyro_accel_raw().await?;

        // Try to read magnetometer (non-blocking)
        let mag = match self.read_mag_raw().await {
            Ok(m) => Some(m),
            Err(ImuError::MagNotReady) => None,
            Err(e) => return Err(e),
        };

        Ok((gyro, accel, mag))
    }

    // =========================================================================
    // Hardware Calibration Methods (User Bank Offset Registers)
    // =========================================================================

    /// Write gyroscope offset to hardware registers (Bank 2)
    ///
    /// These offsets are applied by the sensor hardware before data output.
    /// The offset values are in raw LSB units (16-bit signed).
    ///
    /// # Arguments
    ///
    /// * `offset` - [X, Y, Z] offset values in raw LSB units
    pub async fn set_gyro_offset(&mut self, offset: [i16; 3]) -> Result<(), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Switch to Bank 2 for gyro offset registers
        self.select_bank(RegisterBank::Bank2).await?;

        // Write X offset
        let x_bytes = offset[0].to_be_bytes();
        self.write_register(registers::XG_OFFS_USRH, x_bytes[0])
            .await?;
        self.write_register(registers::XG_OFFS_USRL, x_bytes[1])
            .await?;

        // Write Y offset
        let y_bytes = offset[1].to_be_bytes();
        self.write_register(registers::YG_OFFS_USRH, y_bytes[0])
            .await?;
        self.write_register(registers::YG_OFFS_USRL, y_bytes[1])
            .await?;

        // Write Z offset
        let z_bytes = offset[2].to_be_bytes();
        self.write_register(registers::ZG_OFFS_USRH, z_bytes[0])
            .await?;
        self.write_register(registers::ZG_OFFS_USRL, z_bytes[1])
            .await?;

        // Return to Bank 0
        self.select_bank(RegisterBank::Bank0).await?;

        crate::log_debug!(
            "Gyro offset set: X={}, Y={}, Z={}",
            offset[0],
            offset[1],
            offset[2]
        );
        Ok(())
    }

    /// Read gyroscope offset from hardware registers (Bank 2)
    ///
    /// Returns the current hardware offset values in raw LSB units.
    pub async fn get_gyro_offset(&mut self) -> Result<[i16; 3], ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Switch to Bank 2
        self.select_bank(RegisterBank::Bank2).await?;

        // Read X offset
        let xh = self.read_register(registers::XG_OFFS_USRH).await?;
        let xl = self.read_register(registers::XG_OFFS_USRL).await?;
        let x = i16::from_be_bytes([xh, xl]);

        // Read Y offset
        let yh = self.read_register(registers::YG_OFFS_USRH).await?;
        let yl = self.read_register(registers::YG_OFFS_USRL).await?;
        let y = i16::from_be_bytes([yh, yl]);

        // Read Z offset
        let zh = self.read_register(registers::ZG_OFFS_USRH).await?;
        let zl = self.read_register(registers::ZG_OFFS_USRL).await?;
        let z = i16::from_be_bytes([zh, zl]);

        // Return to Bank 0
        self.select_bank(RegisterBank::Bank0).await?;

        Ok([x, y, z])
    }

    /// Write accelerometer offset to hardware registers (Bank 1)
    ///
    /// These offsets are applied by the sensor hardware before data output.
    /// The offset values are 15-bit signed values in raw LSB units.
    /// Note: Bit 0 of the low byte is reserved and preserved.
    ///
    /// # Arguments
    ///
    /// * `offset` - [X, Y, Z] offset values in raw LSB units (15-bit)
    pub async fn set_accel_offset(&mut self, offset: [i16; 3]) -> Result<(), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Switch to Bank 1 for accel offset registers
        self.select_bank(RegisterBank::Bank1).await?;

        // Accelerometer offsets are 15-bit values
        // Format: [14:7] in high byte, [6:0] in low byte bits [7:1], bit 0 reserved

        // Write X offset
        // First read to preserve bit 0
        let xl_old = self.read_register(registers::XA_OFFS_L).await?;
        let x_val = (offset[0] << 1) | (xl_old as i16 & 0x01);
        let x_bytes = x_val.to_be_bytes();
        self.write_register(registers::XA_OFFS_H, x_bytes[0])
            .await?;
        self.write_register(registers::XA_OFFS_L, x_bytes[1])
            .await?;

        // Write Y offset
        let yl_old = self.read_register(registers::YA_OFFS_L).await?;
        let y_val = (offset[1] << 1) | (yl_old as i16 & 0x01);
        let y_bytes = y_val.to_be_bytes();
        self.write_register(registers::YA_OFFS_H, y_bytes[0])
            .await?;
        self.write_register(registers::YA_OFFS_L, y_bytes[1])
            .await?;

        // Write Z offset
        let zl_old = self.read_register(registers::ZA_OFFS_L).await?;
        let z_val = (offset[2] << 1) | (zl_old as i16 & 0x01);
        let z_bytes = z_val.to_be_bytes();
        self.write_register(registers::ZA_OFFS_H, z_bytes[0])
            .await?;
        self.write_register(registers::ZA_OFFS_L, z_bytes[1])
            .await?;

        // Return to Bank 0
        self.select_bank(RegisterBank::Bank0).await?;

        crate::log_debug!(
            "Accel offset set: X={}, Y={}, Z={}",
            offset[0],
            offset[1],
            offset[2]
        );
        Ok(())
    }

    /// Read accelerometer offset from hardware registers (Bank 1)
    ///
    /// Returns the current hardware offset values in raw LSB units (15-bit).
    pub async fn get_accel_offset(&mut self) -> Result<[i16; 3], ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        // Switch to Bank 1
        self.select_bank(RegisterBank::Bank1).await?;

        // Read X offset (15-bit, shift right by 1)
        let xh = self.read_register(registers::XA_OFFS_H).await?;
        let xl = self.read_register(registers::XA_OFFS_L).await?;
        let x = i16::from_be_bytes([xh, xl]) >> 1;

        // Read Y offset
        let yh = self.read_register(registers::YA_OFFS_H).await?;
        let yl = self.read_register(registers::YA_OFFS_L).await?;
        let y = i16::from_be_bytes([yh, yl]) >> 1;

        // Read Z offset
        let zh = self.read_register(registers::ZA_OFFS_H).await?;
        let zl = self.read_register(registers::ZA_OFFS_L).await?;
        let z = i16::from_be_bytes([zh, zl]) >> 1;

        // Return to Bank 0
        self.select_bank(RegisterBank::Bank0).await?;

        Ok([x, y, z])
    }
}

// =============================================================================
// ImuSensor Trait Implementation
// =============================================================================

impl<I2C> ImuSensor for Icm20948Driver<I2C>
where
    I2C: I2c,
{
    /// Read all 9 axes: gyro (rad/s), accel (m/s²), mag (µT)
    ///
    /// Returns calibrated sensor data with timestamp.
    /// If magnetometer data is not ready, uses zeros for mag values.
    async fn read_all(&mut self) -> Result<ImuReading, ImuError> {
        // Read gyro, accel, and temperature
        let (gyro_raw, accel_raw, temp_raw) = self.read_gyro_accel_temp_raw().await?;

        // Try to read magnetometer (may not be ready if not initialized or data not ready)
        let mag_raw = match self.read_mag_raw().await {
            Ok(m) => m,
            Err(ImuError::MagNotReady) | Err(ImuError::NotInitialized) => [0, 0, 0],
            Err(e) => return Err(e),
        };

        Ok(ImuReading {
            gyro: self.convert_gyro(gyro_raw),
            accel: self.convert_accel(accel_raw),
            mag: self.convert_mag(mag_raw),
            temperature: Self::convert_temp(temp_raw),
            timestamp_us: Self::timestamp_us(),
        })
    }

    /// Read gyroscope only (rad/s, body frame)
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError> {
        let (gyro_raw, _) = self.read_gyro_accel_raw().await?;
        Ok(self.convert_gyro(gyro_raw))
    }

    /// Read accelerometer only (m/s², body frame)
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError> {
        let (_, accel_raw) = self.read_gyro_accel_raw().await?;
        Ok(self.convert_accel(accel_raw))
    }

    /// Read magnetometer only (µT, body frame)
    async fn read_mag(&mut self) -> Result<Vector3<f32>, ImuError> {
        let mag_raw = self.read_mag_raw().await?;
        Ok(self.convert_mag(mag_raw))
    }

    /// Apply calibration data
    ///
    /// Calibration is applied to all subsequent readings.
    fn set_calibration(&mut self, calibration: ImuCalibration) {
        self.calibration = calibration;
    }

    /// Get current calibration data
    fn calibration(&self) -> &ImuCalibration {
        &self.calibration
    }

    /// Get sensor health status
    ///
    /// Returns false if sensor has consecutive read errors or
    /// data appears invalid (e.g., stuck values).
    fn is_healthy(&self) -> bool {
        self.initialized && self.healthy
    }
}

// =============================================================================
// Unit Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::imu::icm20948::config::{AccelRange, GyroRange};

    /// Stub I2C for testing static methods
    ///
    /// This is a minimal implementation that satisfies the trait bound
    /// but is never actually used - only needed for type instantiation.
    struct StubI2c;

    impl embedded_hal_async::i2c::ErrorType for StubI2c {
        type Error = embedded_hal_async::i2c::ErrorKind;
    }

    impl embedded_hal_async::i2c::I2c for StubI2c {
        async fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [embedded_hal_async::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_convert_temp_room_temperature() {
        // At room temperature offset (raw = 0), should return 21°C
        let temp = Icm20948Driver::<StubI2c>::convert_temp(0);
        assert!((temp - 21.0).abs() < 0.01);
    }

    #[test]
    fn test_convert_temp_above_room() {
        // Test positive offset
        // raw = 333.87 should give 22°C
        let temp = Icm20948Driver::<StubI2c>::convert_temp(334);
        assert!((temp - 22.0).abs() < 0.1);
    }

    #[test]
    fn test_convert_temp_below_room() {
        // Test negative offset
        // raw = -333.87 should give 20°C
        let temp = Icm20948Driver::<StubI2c>::convert_temp(-334);
        assert!((temp - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_gyro_scale_dps250() {
        let scale = GyroRange::Dps250.scale_to_rad_s();
        // ±250°/s at 131 LSB/°/s
        // Scale = π/180 / 131 ≈ 0.000133
        assert!(scale > 0.0001 && scale < 0.0002);
    }

    #[test]
    fn test_gyro_scale_dps500() {
        let scale = GyroRange::Dps500.scale_to_rad_s();
        // ±500°/s at 65.5 LSB/°/s
        assert!(scale > 0.0002 && scale < 0.0004);
    }

    #[test]
    fn test_gyro_scale_dps1000() {
        let scale = GyroRange::Dps1000.scale_to_rad_s();
        // ±1000°/s at 32.8 LSB/°/s
        assert!(scale > 0.0004 && scale < 0.0008);
    }

    #[test]
    fn test_accel_scale_g2() {
        let scale = AccelRange::G2.scale_to_m_s2();
        // ±2g at 16384 LSB/g
        // Scale = 9.80665 / 16384 ≈ 0.000598
        assert!(scale > 0.0005 && scale < 0.0007);
    }

    #[test]
    fn test_accel_scale_g4() {
        let scale = AccelRange::G4.scale_to_m_s2();
        // ±4g at 8192 LSB/g
        assert!(scale > 0.001 && scale < 0.0015);
    }

    #[test]
    fn test_accel_scale_g16() {
        let scale = AccelRange::G16.scale_to_m_s2();
        // ±16g at 2048 LSB/g
        assert!(scale > 0.004 && scale < 0.006);
    }

    #[test]
    fn test_max_consecutive_errors() {
        // Verify the MAX_CONSECUTIVE_ERRORS constant
        assert_eq!(MAX_CONSECUTIVE_ERRORS, 3);
    }
}
