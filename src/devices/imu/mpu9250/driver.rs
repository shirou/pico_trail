//! MPU-9250 I2C Driver Implementation
//!
//! Core driver implementation for reading IMU data via I2C.

use super::config::{AccelRange, GyroRange, Mpu9250Config};
use super::registers::{self, MAG_SENSITIVITY, TEMP_OFFSET, TEMP_SENSITIVITY};
use crate::devices::traits::{ImuCalibration, ImuError, ImuReading, ImuSensor};
use embassy_rp::i2c::{Async, I2c};
use embassy_time::Instant;
use embedded_hal_async::i2c::I2c as AsyncI2c;
use nalgebra::Vector3;

/// Maximum consecutive errors before marking sensor unhealthy
const MAX_CONSECUTIVE_ERRORS: u32 = 3;

/// MPU-9250/MPU-6500/MPU-9255 I2C Driver
///
/// Implements `ImuSensor` trait for the MPU-9250 9-axis IMU.
/// Also supports MPU-6500 (6-axis, no magnetometer) and MPU-9255.
/// Uses I2C bypass mode for AK8963 magnetometer access when available.
pub struct Mpu9250Driver<'d, T: embassy_rp::i2c::Instance> {
    /// I2C bus handle
    i2c: I2c<'d, T, Async>,

    /// Driver configuration
    config: Mpu9250Config,

    /// Calibration data
    calibration: ImuCalibration,

    /// Gyro scale factor (raw to rad/s)
    gyro_scale: f32,

    /// Accel scale factor (raw to m/s²)
    accel_scale: f32,

    /// Magnetometer sensitivity adjustment values
    mag_asa: [f32; 3],

    /// Whether the device has a magnetometer (MPU-9250/9255: true, MPU-6500: false)
    has_magnetometer: bool,

    /// Health status
    healthy: bool,

    /// Consecutive error count
    error_count: u32,

    /// Initialization complete flag
    initialized: bool,
}

impl<'d, T: embassy_rp::i2c::Instance> Mpu9250Driver<'d, T> {
    /// Create and initialize a new MPU-9250 driver
    ///
    /// # Arguments
    ///
    /// * `i2c` - Embassy I2C handle in async mode
    /// * `config` - Driver configuration
    ///
    /// # Returns
    ///
    /// Initialized driver or error if initialization failed
    pub async fn new(i2c: I2c<'d, T, Async>, config: Mpu9250Config) -> Result<Self, ImuError> {
        let mut driver = Self {
            i2c,
            config,
            calibration: ImuCalibration::default(),
            gyro_scale: config.gyro_range.scale_to_rad_s(),
            accel_scale: config.accel_range.scale_to_m_s2(),
            mag_asa: [1.0, 1.0, 1.0],
            has_magnetometer: false, // Set by init() based on detected device
            healthy: false,
            error_count: 0,
            initialized: false,
        };

        driver.init().await?;
        Ok(driver)
    }

    /// Initialize the MPU-9250/MPU-6500/MPU-9255 and optionally AK8963
    async fn init(&mut self) -> Result<(), ImuError> {
        // Step 1: Verify WHO_AM_I (accept MPU-6500, MPU-9250, MPU-9255)
        let whoami = self.read_register(registers::WHO_AM_I).await?;
        let (device_name, has_magnetometer) = match whoami {
            registers::MPU6500_WHO_AM_I_VALUE => ("MPU-6500", false),
            registers::MPU9250_WHO_AM_I_VALUE => ("MPU-9250", true),
            registers::MPU9255_WHO_AM_I_VALUE => ("MPU-9255", true),
            _ => {
                crate::log_error!(
                    "Unknown IMU WHO_AM_I: {:#x} (expected 0x70, 0x71, or 0x73)",
                    whoami
                );
                return Err(ImuError::NotInitialized);
            }
        };
        crate::log_info!("{} detected (WHO_AM_I: {:#x})", device_name, whoami);

        // Step 2: Reset device
        self.write_register(registers::PWR_MGMT_1, registers::PWR_MGMT_1_H_RESET)
            .await?;
        embassy_time::Timer::after_millis(100).await;

        // Step 3: Wake up device with auto clock select
        self.write_register(registers::PWR_MGMT_1, registers::PWR_MGMT_1_CLKSEL_AUTO)
            .await?;
        embassy_time::Timer::after_millis(10).await;

        // Step 4: Configure sample rate divider
        self.write_register(registers::SMPLRT_DIV, self.config.sample_rate_div)
            .await?;

        // Step 5: Configure gyroscope
        self.write_register(
            registers::CONFIG,
            self.config.gyro_dlpf.gyro_register_value(),
        )
        .await?;
        self.write_register(
            registers::GYRO_CONFIG,
            self.config.gyro_range.register_value(),
        )
        .await?;

        // Step 6: Configure accelerometer
        self.write_register(
            registers::ACCEL_CONFIG,
            self.config.accel_range.register_value(),
        )
        .await?;
        self.write_register(
            registers::ACCEL_CONFIG_2,
            self.config.accel_dlpf.accel_register_value(),
        )
        .await?;

        // Store magnetometer availability
        self.has_magnetometer = has_magnetometer;

        // Step 7-8: Initialize magnetometer only if present (MPU-9250/MPU-9255)
        if has_magnetometer {
            // Enable I2C bypass for AK8963 access
            self.write_register(registers::INT_PIN_CFG, registers::INT_PIN_CFG_BYPASS_EN)
                .await?;
            embassy_time::Timer::after_millis(10).await;

            // Initialize AK8963 magnetometer
            self.init_ak8963().await?;
        } else {
            crate::log_info!("{} has no magnetometer, skipping AK8963 init", device_name);
        }

        self.initialized = true;
        self.healthy = true;
        crate::log_info!("{} initialized successfully", device_name);

        Ok(())
    }

    /// Initialize the AK8963 magnetometer
    async fn init_ak8963(&mut self) -> Result<(), ImuError> {
        // Verify AK8963 WHO_AM_I
        let whoami = self.read_mag_register(registers::AK8963_WIA).await?;
        if whoami != registers::AK8963_WHO_AM_I_VALUE {
            crate::log_error!(
                "AK8963 WHO_AM_I mismatch: expected {:#x}, got {:#x}",
                registers::AK8963_WHO_AM_I_VALUE,
                whoami
            );
            return Err(ImuError::NotInitialized);
        }

        // Reset magnetometer
        self.write_mag_register(registers::AK8963_CNTL2, registers::AK8963_CNTL2_SRST)
            .await?;
        embassy_time::Timer::after_millis(10).await;

        // Enter Fuse ROM access mode to read sensitivity adjustment
        self.write_mag_register(registers::AK8963_CNTL1, registers::AK8963_MODE_FUSE_ROM)
            .await?;
        embassy_time::Timer::after_millis(10).await;

        // Read sensitivity adjustment values
        let asax = self.read_mag_register(registers::AK8963_ASAX).await?;
        let asay = self.read_mag_register(registers::AK8963_ASAY).await?;
        let asaz = self.read_mag_register(registers::AK8963_ASAZ).await?;

        // Calculate adjustment factors: Hadj = H * (((ASA - 128) * 0.5 / 128) + 1)
        self.mag_asa[0] = ((asax as f32 - 128.0) * 0.5 / 128.0) + 1.0;
        self.mag_asa[1] = ((asay as f32 - 128.0) * 0.5 / 128.0) + 1.0;
        self.mag_asa[2] = ((asaz as f32 - 128.0) * 0.5 / 128.0) + 1.0;

        // Power down
        self.write_mag_register(registers::AK8963_CNTL1, registers::AK8963_MODE_POWER_DOWN)
            .await?;
        embassy_time::Timer::after_millis(10).await;

        // Set continuous measurement mode
        self.write_mag_register(
            registers::AK8963_CNTL1,
            self.config.mag_mode.register_value(),
        )
        .await?;
        embassy_time::Timer::after_millis(10).await;

        crate::log_info!(
            "AK8963 initialized: ASA=[{}, {}, {}]",
            self.mag_asa[0],
            self.mag_asa[1],
            self.mag_asa[2]
        );

        Ok(())
    }

    /// Read a register from MPU-9250
    async fn read_register(&mut self, reg: u8) -> Result<u8, ImuError> {
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

    /// Write a register to MPU-9250
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), ImuError> {
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

    /// Read a register from AK8963 (via bypass mode)
    async fn read_mag_register(&mut self, reg: u8) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(registers::AK8963_ADDR, &[reg], &mut buf)
            .await
            .map_err(|_| ImuError::I2cError)?;
        Ok(buf[0])
    }

    /// Write a register to AK8963 (via bypass mode)
    async fn write_mag_register(&mut self, reg: u8, value: u8) -> Result<(), ImuError> {
        self.i2c
            .write(registers::AK8963_ADDR, &[reg, value])
            .await
            .map_err(|_| ImuError::I2cError)?;
        Ok(())
    }

    /// Read raw gyro and accelerometer data (14 bytes)
    async fn read_gyro_accel_raw(&mut self) -> Result<([i16; 3], [i16; 3], i16), ImuError> {
        if !self.initialized {
            return Err(ImuError::NotInitialized);
        }

        let mut buf = [0u8; 14];
        self.i2c
            .write_read(
                self.config.i2c_address,
                &[registers::ACCEL_XOUT_H],
                &mut buf,
            )
            .await
            .map_err(|_| {
                self.error_count += 1;
                if self.error_count >= MAX_CONSECUTIVE_ERRORS {
                    self.healthy = false;
                }
                ImuError::I2cError
            })?;
        self.error_count = 0;

        // Parse big-endian 16-bit values
        let accel = [
            i16::from_be_bytes([buf[0], buf[1]]),
            i16::from_be_bytes([buf[2], buf[3]]),
            i16::from_be_bytes([buf[4], buf[5]]),
        ];
        let temp = i16::from_be_bytes([buf[6], buf[7]]);
        let gyro = [
            i16::from_be_bytes([buf[8], buf[9]]),
            i16::from_be_bytes([buf[10], buf[11]]),
            i16::from_be_bytes([buf[12], buf[13]]),
        ];

        Ok((gyro, accel, temp))
    }

    /// Read raw magnetometer data (7 bytes)
    async fn read_mag_raw(&mut self) -> Result<Option<[i16; 3]>, ImuError> {
        // Check data ready
        let st1 = self.read_mag_register(registers::AK8963_ST1).await?;
        if (st1 & registers::AK8963_ST1_DRDY) == 0 {
            return Ok(None); // Data not ready
        }

        // Read 7 bytes: HXL, HXH, HYL, HYH, HZL, HZH, ST2
        let mut buf = [0u8; 7];
        self.i2c
            .write_read(registers::AK8963_ADDR, &[registers::AK8963_HXL], &mut buf)
            .await
            .map_err(|_| ImuError::I2cError)?;

        // Check overflow
        if (buf[6] & registers::AK8963_ST2_HOFL) != 0 {
            return Err(ImuError::InvalidData);
        }

        // Parse little-endian 16-bit values (AK8963 uses little-endian!)
        let mag = [
            i16::from_le_bytes([buf[0], buf[1]]),
            i16::from_le_bytes([buf[2], buf[3]]),
            i16::from_le_bytes([buf[4], buf[5]]),
        ];

        Ok(Some(mag))
    }

    /// Convert raw gyro to rad/s with calibration
    fn convert_gyro(&self, raw: [i16; 3]) -> Vector3<f32> {
        let raw_vec = Vector3::new(raw[0] as f32, raw[1] as f32, raw[2] as f32);
        let scaled = raw_vec * self.gyro_scale;
        self.calibration.apply_gyro(scaled)
    }

    /// Convert raw accel to m/s² with calibration
    fn convert_accel(&self, raw: [i16; 3]) -> Vector3<f32> {
        let raw_vec = Vector3::new(raw[0] as f32, raw[1] as f32, raw[2] as f32);
        let scaled = raw_vec * self.accel_scale;
        self.calibration.apply_accel(scaled)
    }

    /// Convert raw mag to µT with sensitivity adjustment and calibration
    fn convert_mag(&self, raw: [i16; 3]) -> Vector3<f32> {
        let raw_vec = Vector3::new(
            raw[0] as f32 * self.mag_asa[0],
            raw[1] as f32 * self.mag_asa[1],
            raw[2] as f32 * self.mag_asa[2],
        );
        let scaled = raw_vec * MAG_SENSITIVITY;
        self.calibration.apply_mag(scaled)
    }

    /// Convert raw temperature to °C
    fn convert_temp(raw: i16) -> f32 {
        (raw as f32 / TEMP_SENSITIVITY) + TEMP_OFFSET
    }

    /// Get current timestamp in microseconds
    fn timestamp_us() -> u64 {
        Instant::now().as_micros()
    }

    /// Reconfigure gyroscope range
    pub fn set_gyro_range(&mut self, range: GyroRange) {
        self.config.gyro_range = range;
        self.gyro_scale = range.scale_to_rad_s();
    }

    /// Reconfigure accelerometer range
    pub fn set_accel_range(&mut self, range: AccelRange) {
        self.config.accel_range = range;
        self.accel_scale = range.scale_to_m_s2();
    }
}

impl<'d, T: embassy_rp::i2c::Instance> ImuSensor for Mpu9250Driver<'d, T> {
    async fn read_all(&mut self) -> Result<ImuReading, ImuError> {
        let (gyro_raw, accel_raw, temp_raw) = self.read_gyro_accel_raw().await?;

        // Only read magnetometer if available (MPU-9250/9255)
        let mag_raw = if self.has_magnetometer {
            self.read_mag_raw().await?.unwrap_or([0, 0, 0])
        } else {
            [0, 0, 0]
        };

        Ok(ImuReading {
            gyro: self.convert_gyro(gyro_raw),
            accel: self.convert_accel(accel_raw),
            mag: self.convert_mag(mag_raw),
            temperature: Self::convert_temp(temp_raw),
            timestamp_us: Self::timestamp_us(),
        })
    }

    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError> {
        let (gyro_raw, _, _) = self.read_gyro_accel_raw().await?;
        Ok(self.convert_gyro(gyro_raw))
    }

    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError> {
        let (_, accel_raw, _) = self.read_gyro_accel_raw().await?;
        Ok(self.convert_accel(accel_raw))
    }

    async fn read_mag(&mut self) -> Result<Vector3<f32>, ImuError> {
        // Return zeros if no magnetometer (MPU-6500)
        if !self.has_magnetometer {
            return Ok(Vector3::new(0.0, 0.0, 0.0));
        }

        match self.read_mag_raw().await? {
            Some(mag_raw) => Ok(self.convert_mag(mag_raw)),
            None => Err(ImuError::MagNotReady),
        }
    }

    fn set_calibration(&mut self, calibration: ImuCalibration) {
        self.calibration = calibration;
    }

    fn calibration(&self) -> &ImuCalibration {
        &self.calibration
    }

    fn is_healthy(&self) -> bool {
        self.initialized && self.healthy
    }
}
