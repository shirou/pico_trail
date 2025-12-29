//! ICM-20948 Register Definitions
//!
//! This module defines all registers for the ICM-20948 9-axis IMU.
//! The ICM-20948 uses a 4-bank register architecture, with bank selection
//! via the REG_BANK_SEL register (0x7F) in all banks.
//!
//! ## Register Banks
//!
//! - Bank 0: User configuration and sensor data
//! - Bank 1: Self-test data
//! - Bank 2: Sensor configuration (gyro/accel ranges, DLPF)
//! - Bank 3: I2C master configuration

// =============================================================================
// I2C Addresses
// =============================================================================

/// ICM-20948 default I2C address (AD0 = LOW)
pub const ICM20948_ADDR: u8 = 0x68;

/// ICM-20948 alternate I2C address (AD0 = HIGH)
pub const ICM20948_ADDR_ALT: u8 = 0x69;

/// AK09916 magnetometer I2C address (accessible via bypass mode)
pub const AK09916_ADDR: u8 = 0x0C;

// =============================================================================
// Bank Selection
// =============================================================================

/// Register bank selection (available in all banks at 0x7F)
pub const REG_BANK_SEL: u8 = 0x7F;

/// Bank 0 value (User Bank)
pub const BANK_0: u8 = 0x00;

/// Bank 1 value (Self-test)
pub const BANK_1: u8 = 0x10;

/// Bank 2 value (Sensor configuration)
pub const BANK_2: u8 = 0x20;

/// Bank 3 value (I2C master)
pub const BANK_3: u8 = 0x30;

// =============================================================================
// Bank 0 Registers (User Bank)
// =============================================================================

/// Device ID (WHO_AM_I)
pub const WHO_AM_I: u8 = 0x00;

/// User control
pub const USER_CTRL: u8 = 0x03;

/// Low power control
pub const LP_CONFIG: u8 = 0x05;

/// Power management 1
pub const PWR_MGMT_1: u8 = 0x06;

/// Power management 2
pub const PWR_MGMT_2: u8 = 0x07;

/// Interrupt pin configuration
pub const INT_PIN_CFG: u8 = 0x0F;

/// Interrupt enable
pub const INT_ENABLE: u8 = 0x10;

/// Interrupt enable 1
pub const INT_ENABLE_1: u8 = 0x11;

/// Interrupt enable 2
pub const INT_ENABLE_2: u8 = 0x12;

/// Interrupt enable 3
pub const INT_ENABLE_3: u8 = 0x13;

/// I2C master status
pub const I2C_MST_STATUS: u8 = 0x17;

/// Interrupt status
pub const INT_STATUS: u8 = 0x19;

/// Interrupt status 1
pub const INT_STATUS_1: u8 = 0x1A;

/// Interrupt status 2
pub const INT_STATUS_2: u8 = 0x1B;

/// Interrupt status 3
pub const INT_STATUS_3: u8 = 0x1C;

/// Delay time high byte
pub const DELAY_TIMEH: u8 = 0x28;

/// Delay time low byte
pub const DELAY_TIMEL: u8 = 0x29;

/// Accelerometer X output high byte
pub const ACCEL_XOUT_H: u8 = 0x2D;

/// Accelerometer X output low byte
pub const ACCEL_XOUT_L: u8 = 0x2E;

/// Accelerometer Y output high byte
pub const ACCEL_YOUT_H: u8 = 0x2F;

/// Accelerometer Y output low byte
pub const ACCEL_YOUT_L: u8 = 0x30;

/// Accelerometer Z output high byte
pub const ACCEL_ZOUT_H: u8 = 0x31;

/// Accelerometer Z output low byte
pub const ACCEL_ZOUT_L: u8 = 0x32;

/// Gyroscope X output high byte
pub const GYRO_XOUT_H: u8 = 0x33;

/// Gyroscope X output low byte
pub const GYRO_XOUT_L: u8 = 0x34;

/// Gyroscope Y output high byte
pub const GYRO_YOUT_H: u8 = 0x35;

/// Gyroscope Y output low byte
pub const GYRO_YOUT_L: u8 = 0x36;

/// Gyroscope Z output high byte
pub const GYRO_ZOUT_H: u8 = 0x37;

/// Gyroscope Z output low byte
pub const GYRO_ZOUT_L: u8 = 0x38;

/// Temperature output high byte
pub const TEMP_OUT_H: u8 = 0x39;

/// Temperature output low byte
pub const TEMP_OUT_L: u8 = 0x3A;

/// External sensor data registers (for I2C master mode)
pub const EXT_SLV_SENS_DATA_00: u8 = 0x3B;

/// FIFO enable 1
pub const FIFO_EN_1: u8 = 0x66;

/// FIFO enable 2
pub const FIFO_EN_2: u8 = 0x67;

/// FIFO reset
pub const FIFO_RST: u8 = 0x68;

/// FIFO mode
pub const FIFO_MODE: u8 = 0x69;

/// FIFO count high byte
pub const FIFO_COUNTH: u8 = 0x70;

/// FIFO count low byte
pub const FIFO_COUNTL: u8 = 0x71;

/// FIFO read/write
pub const FIFO_R_W: u8 = 0x72;

/// Data ready status
pub const DATA_RDY_STATUS: u8 = 0x74;

/// FIFO configuration
pub const FIFO_CFG: u8 = 0x76;

// =============================================================================
// Bank 1 Registers (Self-test and Accelerometer Offsets)
// =============================================================================

/// X accelerometer offset high byte (Bank 1)
/// 15-bit value: [14:7] in high byte, [6:0] in low byte bit [7:1]
pub const XA_OFFS_H: u8 = 0x14;

/// X accelerometer offset low byte (Bank 1)
/// Bit 0 is reserved
pub const XA_OFFS_L: u8 = 0x15;

/// Y accelerometer offset high byte (Bank 1)
pub const YA_OFFS_H: u8 = 0x17;

/// Y accelerometer offset low byte (Bank 1)
pub const YA_OFFS_L: u8 = 0x18;

/// Z accelerometer offset high byte (Bank 1)
pub const ZA_OFFS_H: u8 = 0x1A;

/// Z accelerometer offset low byte (Bank 1)
pub const ZA_OFFS_L: u8 = 0x1B;

// =============================================================================
// Bank 2 Registers (Sensor Configuration)
// =============================================================================

/// Gyroscope sample rate divider
pub const GYRO_SMPLRT_DIV: u8 = 0x00;

/// Gyroscope configuration 1
pub const GYRO_CONFIG_1: u8 = 0x01;

/// Gyroscope configuration 2
pub const GYRO_CONFIG_2: u8 = 0x02;

/// X gyroscope offset high byte
pub const XG_OFFS_USRH: u8 = 0x03;

/// X gyroscope offset low byte
pub const XG_OFFS_USRL: u8 = 0x04;

/// Y gyroscope offset high byte
pub const YG_OFFS_USRH: u8 = 0x05;

/// Y gyroscope offset low byte
pub const YG_OFFS_USRL: u8 = 0x06;

/// Z gyroscope offset high byte
pub const ZG_OFFS_USRH: u8 = 0x07;

/// Z gyroscope offset low byte
pub const ZG_OFFS_USRL: u8 = 0x08;

/// ODR alignment enable
pub const ODR_ALIGN_EN: u8 = 0x09;

/// Accelerometer sample rate divider high byte
pub const ACCEL_SMPLRT_DIV_1: u8 = 0x10;

/// Accelerometer sample rate divider low byte
pub const ACCEL_SMPLRT_DIV_2: u8 = 0x11;

/// Accelerometer intelligence control
pub const ACCEL_INTEL_CTRL: u8 = 0x12;

/// Wake on motion threshold
pub const WOM_THR: u8 = 0x13;

/// Accelerometer configuration
pub const ACCEL_CONFIG: u8 = 0x14;

/// Accelerometer configuration 2
pub const ACCEL_CONFIG_2: u8 = 0x15;

/// FSYNC configuration
pub const FSYNC_CONFIG: u8 = 0x52;

/// Temperature configuration
pub const TEMP_CONFIG: u8 = 0x53;

/// Mod control (user)
pub const MOD_CTRL_USR: u8 = 0x54;

// =============================================================================
// Bank 3 Registers (I2C Master)
// =============================================================================

/// I2C master ODR configuration
pub const I2C_MST_ODR_CONFIG: u8 = 0x00;

/// I2C master control
pub const I2C_MST_CTRL: u8 = 0x01;

/// I2C master delay control
pub const I2C_MST_DELAY_CTRL: u8 = 0x02;

/// I2C slave 0 address
pub const I2C_SLV0_ADDR: u8 = 0x03;

/// I2C slave 0 register
pub const I2C_SLV0_REG: u8 = 0x04;

/// I2C slave 0 control
pub const I2C_SLV0_CTRL: u8 = 0x05;

/// I2C slave 0 data out
pub const I2C_SLV0_DO: u8 = 0x06;

/// I2C slave 1 address
pub const I2C_SLV1_ADDR: u8 = 0x07;

/// I2C slave 1 register
pub const I2C_SLV1_REG: u8 = 0x08;

/// I2C slave 1 control
pub const I2C_SLV1_CTRL: u8 = 0x09;

/// I2C slave 1 data out
pub const I2C_SLV1_DO: u8 = 0x0A;

/// I2C slave 2 address
pub const I2C_SLV2_ADDR: u8 = 0x0B;

/// I2C slave 2 register
pub const I2C_SLV2_REG: u8 = 0x0C;

/// I2C slave 2 control
pub const I2C_SLV2_CTRL: u8 = 0x0D;

/// I2C slave 2 data out
pub const I2C_SLV2_DO: u8 = 0x0E;

/// I2C slave 3 address
pub const I2C_SLV3_ADDR: u8 = 0x0F;

/// I2C slave 3 register
pub const I2C_SLV3_REG: u8 = 0x10;

/// I2C slave 3 control
pub const I2C_SLV3_CTRL: u8 = 0x11;

/// I2C slave 3 data out
pub const I2C_SLV3_DO: u8 = 0x12;

/// I2C slave 4 address
pub const I2C_SLV4_ADDR: u8 = 0x13;

/// I2C slave 4 register
pub const I2C_SLV4_REG: u8 = 0x14;

/// I2C slave 4 control
pub const I2C_SLV4_CTRL: u8 = 0x15;

/// I2C slave 4 data out
pub const I2C_SLV4_DO: u8 = 0x16;

/// I2C slave 4 data in
pub const I2C_SLV4_DI: u8 = 0x17;

// =============================================================================
// WHO_AM_I Values
// =============================================================================

/// ICM-20948 WHO_AM_I expected value
pub const ICM20948_WHO_AM_I_VALUE: u8 = 0xEA;

// =============================================================================
// PWR_MGMT_1 Bit Definitions
// =============================================================================

/// Device reset bit
pub const PWR_MGMT_1_DEVICE_RESET: u8 = 0x80;

/// Sleep mode bit
pub const PWR_MGMT_1_SLEEP: u8 = 0x40;

/// Low power enable bit
pub const PWR_MGMT_1_LP_EN: u8 = 0x20;

/// Temperature disable bit
pub const PWR_MGMT_1_TEMP_DIS: u8 = 0x08;

/// Clock source: auto select best available clock
pub const PWR_MGMT_1_CLKSEL_AUTO: u8 = 0x01;

// =============================================================================
// PWR_MGMT_2 Bit Definitions
// =============================================================================

/// Disable accelerometer X
pub const PWR_MGMT_2_DISABLE_ACCEL_X: u8 = 0x20;

/// Disable accelerometer Y
pub const PWR_MGMT_2_DISABLE_ACCEL_Y: u8 = 0x10;

/// Disable accelerometer Z
pub const PWR_MGMT_2_DISABLE_ACCEL_Z: u8 = 0x08;

/// Disable gyroscope X
pub const PWR_MGMT_2_DISABLE_GYRO_X: u8 = 0x04;

/// Disable gyroscope Y
pub const PWR_MGMT_2_DISABLE_GYRO_Y: u8 = 0x02;

/// Disable gyroscope Z
pub const PWR_MGMT_2_DISABLE_GYRO_Z: u8 = 0x01;

/// Enable all sensors (value to write)
pub const PWR_MGMT_2_ENABLE_ALL: u8 = 0x00;

// =============================================================================
// INT_PIN_CFG Bit Definitions
// =============================================================================

/// Active low interrupt
pub const INT_PIN_CFG_ACTIVE_LOW: u8 = 0x80;

/// Open drain interrupt
pub const INT_PIN_CFG_OPEN_DRAIN: u8 = 0x40;

/// Latch interrupt until cleared
pub const INT_PIN_CFG_LATCH_INT_EN: u8 = 0x20;

/// Clear interrupt by any read
pub const INT_PIN_CFG_INT_ANYRD_2CLEAR: u8 = 0x10;

/// Active low FSYNC
pub const INT_PIN_CFG_ACTL_FSYNC: u8 = 0x08;

/// Enable FSYNC interrupt
pub const INT_PIN_CFG_FSYNC_INT_MODE_EN: u8 = 0x04;

/// Enable I2C bypass mode for direct magnetometer access
pub const INT_PIN_CFG_BYPASS_EN: u8 = 0x02;

// =============================================================================
// USER_CTRL Bit Definitions
// =============================================================================

/// Enable DMP
pub const USER_CTRL_DMP_EN: u8 = 0x80;

/// Enable FIFO
pub const USER_CTRL_FIFO_EN: u8 = 0x40;

/// Enable I2C master
pub const USER_CTRL_I2C_MST_EN: u8 = 0x20;

/// Disable I2C interface
pub const USER_CTRL_I2C_IF_DIS: u8 = 0x10;

/// Reset DMP
pub const USER_CTRL_DMP_RST: u8 = 0x08;

/// Reset FIFO and SRAM
pub const USER_CTRL_SRAM_RST: u8 = 0x04;

/// Reset I2C master
pub const USER_CTRL_I2C_MST_RST: u8 = 0x02;

// =============================================================================
// Gyroscope Configuration (GYRO_CONFIG_1)
// =============================================================================

/// Gyroscope full scale: ±250 °/s
pub const GYRO_FS_SEL_250DPS: u8 = 0x00;

/// Gyroscope full scale: ±500 °/s
pub const GYRO_FS_SEL_500DPS: u8 = 0x02;

/// Gyroscope full scale: ±1000 °/s
pub const GYRO_FS_SEL_1000DPS: u8 = 0x04;

/// Gyroscope full scale: ±2000 °/s
pub const GYRO_FS_SEL_2000DPS: u8 = 0x06;

/// Gyroscope DLPF enable bit
pub const GYRO_FCHOICE: u8 = 0x01;

/// Gyroscope DLPF configuration: 196.6 Hz (NBW: 229.8 Hz)
pub const GYRO_DLPFCFG_196HZ: u8 = 0x00;

/// Gyroscope DLPF configuration: 151.8 Hz (NBW: 187.6 Hz)
pub const GYRO_DLPFCFG_152HZ: u8 = 0x08;

/// Gyroscope DLPF configuration: 119.5 Hz (NBW: 154.3 Hz)
pub const GYRO_DLPFCFG_120HZ: u8 = 0x10;

/// Gyroscope DLPF configuration: 51.2 Hz (NBW: 73.3 Hz)
pub const GYRO_DLPFCFG_51HZ: u8 = 0x18;

/// Gyroscope DLPF configuration: 23.9 Hz (NBW: 35.9 Hz)
pub const GYRO_DLPFCFG_24HZ: u8 = 0x20;

/// Gyroscope DLPF configuration: 11.6 Hz (NBW: 17.8 Hz)
pub const GYRO_DLPFCFG_12HZ: u8 = 0x28;

/// Gyroscope DLPF configuration: 5.7 Hz (NBW: 8.9 Hz)
pub const GYRO_DLPFCFG_6HZ: u8 = 0x30;

/// Gyroscope DLPF configuration: 361.4 Hz (NBW: 376.5 Hz)
pub const GYRO_DLPFCFG_361HZ: u8 = 0x38;

// =============================================================================
// Accelerometer Configuration (ACCEL_CONFIG)
// =============================================================================

/// Accelerometer full scale: ±2 g
pub const ACCEL_FS_SEL_2G: u8 = 0x00;

/// Accelerometer full scale: ±4 g
pub const ACCEL_FS_SEL_4G: u8 = 0x02;

/// Accelerometer full scale: ±8 g
pub const ACCEL_FS_SEL_8G: u8 = 0x04;

/// Accelerometer full scale: ±16 g
pub const ACCEL_FS_SEL_16G: u8 = 0x06;

/// Accelerometer DLPF enable bit
pub const ACCEL_FCHOICE: u8 = 0x01;

/// Accelerometer DLPF configuration: 246 Hz (NBW: 265 Hz)
pub const ACCEL_DLPFCFG_246HZ: u8 = 0x00;

/// Accelerometer DLPF configuration: 111.4 Hz (NBW: 136 Hz)
pub const ACCEL_DLPFCFG_111HZ: u8 = 0x10;

/// Accelerometer DLPF configuration: 50.4 Hz (NBW: 68.8 Hz)
pub const ACCEL_DLPFCFG_50HZ: u8 = 0x18;

/// Accelerometer DLPF configuration: 23.9 Hz (NBW: 34.4 Hz)
pub const ACCEL_DLPFCFG_24HZ: u8 = 0x20;

/// Accelerometer DLPF configuration: 11.5 Hz (NBW: 17.0 Hz)
pub const ACCEL_DLPFCFG_12HZ: u8 = 0x28;

/// Accelerometer DLPF configuration: 5.7 Hz (NBW: 8.3 Hz)
pub const ACCEL_DLPFCFG_6HZ: u8 = 0x30;

/// Accelerometer DLPF configuration: 473 Hz (NBW: 499 Hz)
pub const ACCEL_DLPFCFG_473HZ: u8 = 0x38;

// =============================================================================
// AK09916 Magnetometer Registers
// =============================================================================

/// Device ID (WHO_AM_I) for AK09916
pub const AK09916_WIA2: u8 = 0x01;

/// Status 1 (data ready)
pub const AK09916_ST1: u8 = 0x10;

/// Measurement data: X-axis low byte
pub const AK09916_HXL: u8 = 0x11;

/// Measurement data: X-axis high byte
pub const AK09916_HXH: u8 = 0x12;

/// Measurement data: Y-axis low byte
pub const AK09916_HYL: u8 = 0x13;

/// Measurement data: Y-axis high byte
pub const AK09916_HYH: u8 = 0x14;

/// Measurement data: Z-axis low byte
pub const AK09916_HZL: u8 = 0x15;

/// Measurement data: Z-axis high byte
pub const AK09916_HZH: u8 = 0x16;

/// Status 2 (data overflow)
pub const AK09916_ST2: u8 = 0x18;

/// Control 2 (mode setting)
pub const AK09916_CNTL2: u8 = 0x31;

/// Control 3 (soft reset)
pub const AK09916_CNTL3: u8 = 0x32;

// =============================================================================
// AK09916 WHO_AM_I Values
// =============================================================================

/// AK09916 WHO_AM_I expected value (WIA2 register)
pub const AK09916_WHO_AM_I_VALUE: u8 = 0x09;

// =============================================================================
// AK09916 Status Bit Definitions
// =============================================================================

/// Data ready bit in ST1
pub const AK09916_ST1_DRDY: u8 = 0x01;

/// Data overflow bit in ST1
pub const AK09916_ST1_DOR: u8 = 0x02;

/// Magnetic sensor overflow bit in ST2
pub const AK09916_ST2_HOFL: u8 = 0x08;

// =============================================================================
// AK09916 Mode Definitions (CNTL2)
// =============================================================================

/// Power down mode
pub const AK09916_MODE_POWER_DOWN: u8 = 0x00;

/// Single measurement mode
pub const AK09916_MODE_SINGLE: u8 = 0x01;

/// Continuous mode 1: 10 Hz
pub const AK09916_MODE_CONT_10HZ: u8 = 0x02;

/// Continuous mode 2: 20 Hz
pub const AK09916_MODE_CONT_20HZ: u8 = 0x04;

/// Continuous mode 3: 50 Hz
pub const AK09916_MODE_CONT_50HZ: u8 = 0x06;

/// Continuous mode 4: 100 Hz
pub const AK09916_MODE_CONT_100HZ: u8 = 0x08;

/// Self-test mode
pub const AK09916_MODE_SELF_TEST: u8 = 0x10;

// =============================================================================
// AK09916 Control Bit Definitions
// =============================================================================

/// Soft reset bit in CNTL3
pub const AK09916_CNTL3_SRST: u8 = 0x01;

// =============================================================================
// Sensitivity Values
// =============================================================================

/// Gyroscope sensitivity at ±250 °/s (LSB/°/s)
pub const GYRO_SENSITIVITY_250DPS: f32 = 131.0;

/// Gyroscope sensitivity at ±500 °/s (LSB/°/s)
pub const GYRO_SENSITIVITY_500DPS: f32 = 65.5;

/// Gyroscope sensitivity at ±1000 °/s (LSB/°/s)
pub const GYRO_SENSITIVITY_1000DPS: f32 = 32.8;

/// Gyroscope sensitivity at ±2000 °/s (LSB/°/s)
pub const GYRO_SENSITIVITY_2000DPS: f32 = 16.4;

/// Accelerometer sensitivity at ±2g (LSB/g)
pub const ACCEL_SENSITIVITY_2G: f32 = 16384.0;

/// Accelerometer sensitivity at ±4g (LSB/g)
pub const ACCEL_SENSITIVITY_4G: f32 = 8192.0;

/// Accelerometer sensitivity at ±8g (LSB/g)
pub const ACCEL_SENSITIVITY_8G: f32 = 4096.0;

/// Accelerometer sensitivity at ±16g (LSB/g)
pub const ACCEL_SENSITIVITY_16G: f32 = 2048.0;

/// Magnetometer sensitivity (µT/LSB) for AK09916
/// AK09916 range: ±4912 µT at 16-bit resolution
pub const MAG_SENSITIVITY: f32 = 4912.0 / 32752.0;

/// Temperature sensitivity (LSB/°C)
pub const TEMP_SENSITIVITY: f32 = 333.87;

/// Temperature offset (°C) at 0 LSB - ICM-20948 uses 21°C offset
pub const TEMP_OFFSET: f32 = 21.0;

// =============================================================================
// Unit Conversion Constants
// =============================================================================

/// Degrees to radians conversion factor
pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// Standard gravity in m/s²
pub const GRAVITY: f32 = 9.80665;
