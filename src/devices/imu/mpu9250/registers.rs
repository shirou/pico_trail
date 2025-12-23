//! MPU-9250 and AK8963 Register Definitions
//!
//! Based on MPU-9250 Register Map (RM-MPU-9250A-00 v1.6)
//! and AK8963 Datasheet.

#![allow(dead_code)]

// ============================================================================
// MPU-9250 I2C Address
// ============================================================================

/// MPU-9250 I2C address when AD0 pin is low
pub const MPU9250_ADDR: u8 = 0x68;

/// MPU-9250 I2C address when AD0 pin is high
pub const MPU9250_ADDR_ALT: u8 = 0x69;

/// AK8963 magnetometer I2C address (accessed via bypass mode)
pub const AK8963_ADDR: u8 = 0x0C;

// ============================================================================
// MPU-9250 Registers
// ============================================================================

/// Gyroscope self-test registers
pub const SELF_TEST_X_GYRO: u8 = 0x00;
pub const SELF_TEST_Y_GYRO: u8 = 0x01;
pub const SELF_TEST_Z_GYRO: u8 = 0x02;

/// Accelerometer self-test registers
pub const SELF_TEST_X_ACCEL: u8 = 0x0D;
pub const SELF_TEST_Y_ACCEL: u8 = 0x0E;
pub const SELF_TEST_Z_ACCEL: u8 = 0x0F;

/// Gyroscope offset registers
pub const XG_OFFSET_H: u8 = 0x13;
pub const XG_OFFSET_L: u8 = 0x14;
pub const YG_OFFSET_H: u8 = 0x15;
pub const YG_OFFSET_L: u8 = 0x16;
pub const ZG_OFFSET_H: u8 = 0x17;
pub const ZG_OFFSET_L: u8 = 0x18;

/// Sample rate divider: SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
pub const SMPLRT_DIV: u8 = 0x19;

/// Configuration register (DLPF, FSYNC)
pub const CONFIG: u8 = 0x1A;

/// Gyroscope configuration (full-scale range, self-test)
pub const GYRO_CONFIG: u8 = 0x1B;

/// Accelerometer configuration (full-scale range, self-test)
pub const ACCEL_CONFIG: u8 = 0x1C;

/// Accelerometer configuration 2 (DLPF for accel)
pub const ACCEL_CONFIG_2: u8 = 0x1D;

/// Low power accelerometer ODR control
pub const LP_ACCEL_ODR: u8 = 0x1E;

/// Wake-on-motion threshold
pub const WOM_THR: u8 = 0x1F;

/// FIFO enable register
pub const FIFO_EN: u8 = 0x23;

/// I2C master control
pub const I2C_MST_CTRL: u8 = 0x24;

/// I2C slave 0-4 address registers
pub const I2C_SLV0_ADDR: u8 = 0x25;
pub const I2C_SLV0_REG: u8 = 0x26;
pub const I2C_SLV0_CTRL: u8 = 0x27;
pub const I2C_SLV1_ADDR: u8 = 0x28;
pub const I2C_SLV1_REG: u8 = 0x29;
pub const I2C_SLV1_CTRL: u8 = 0x2A;
pub const I2C_SLV2_ADDR: u8 = 0x2B;
pub const I2C_SLV2_REG: u8 = 0x2C;
pub const I2C_SLV2_CTRL: u8 = 0x2D;
pub const I2C_SLV3_ADDR: u8 = 0x2E;
pub const I2C_SLV3_REG: u8 = 0x2F;
pub const I2C_SLV3_CTRL: u8 = 0x30;
pub const I2C_SLV4_ADDR: u8 = 0x31;
pub const I2C_SLV4_REG: u8 = 0x32;
pub const I2C_SLV4_DO: u8 = 0x33;
pub const I2C_SLV4_CTRL: u8 = 0x34;
pub const I2C_SLV4_DI: u8 = 0x35;

/// I2C master status
pub const I2C_MST_STATUS: u8 = 0x36;

/// Interrupt pin configuration
pub const INT_PIN_CFG: u8 = 0x37;

/// Interrupt enable
pub const INT_ENABLE: u8 = 0x38;

/// Interrupt status
pub const INT_STATUS: u8 = 0x3A;

/// Accelerometer measurements (high byte first)
pub const ACCEL_XOUT_H: u8 = 0x3B;
pub const ACCEL_XOUT_L: u8 = 0x3C;
pub const ACCEL_YOUT_H: u8 = 0x3D;
pub const ACCEL_YOUT_L: u8 = 0x3E;
pub const ACCEL_ZOUT_H: u8 = 0x3F;
pub const ACCEL_ZOUT_L: u8 = 0x40;

/// Temperature measurement (high byte first)
pub const TEMP_OUT_H: u8 = 0x41;
pub const TEMP_OUT_L: u8 = 0x42;

/// Gyroscope measurements (high byte first)
pub const GYRO_XOUT_H: u8 = 0x43;
pub const GYRO_XOUT_L: u8 = 0x44;
pub const GYRO_YOUT_H: u8 = 0x45;
pub const GYRO_YOUT_L: u8 = 0x46;
pub const GYRO_ZOUT_H: u8 = 0x47;
pub const GYRO_ZOUT_L: u8 = 0x48;

/// External sensor data (from I2C slaves)
pub const EXT_SENS_DATA_00: u8 = 0x49;

/// I2C slave data out registers
pub const I2C_SLV0_DO: u8 = 0x63;
pub const I2C_SLV1_DO: u8 = 0x64;
pub const I2C_SLV2_DO: u8 = 0x65;
pub const I2C_SLV3_DO: u8 = 0x66;

/// I2C master delay control
pub const I2C_MST_DELAY_CTRL: u8 = 0x67;

/// Signal path reset
pub const SIGNAL_PATH_RESET: u8 = 0x68;

/// Motion detect control
pub const MOT_DETECT_CTRL: u8 = 0x69;

/// User control register
pub const USER_CTRL: u8 = 0x6A;

/// Power management 1
pub const PWR_MGMT_1: u8 = 0x6B;

/// Power management 2
pub const PWR_MGMT_2: u8 = 0x6C;

/// FIFO count registers
pub const FIFO_COUNTH: u8 = 0x72;
pub const FIFO_COUNTL: u8 = 0x73;

/// FIFO read/write
pub const FIFO_R_W: u8 = 0x74;

/// Device ID (should read 0x71 for MPU-9250)
pub const WHO_AM_I: u8 = 0x75;

/// Accelerometer offset registers
pub const XA_OFFSET_H: u8 = 0x77;
pub const XA_OFFSET_L: u8 = 0x78;
pub const YA_OFFSET_H: u8 = 0x7A;
pub const YA_OFFSET_L: u8 = 0x7B;
pub const ZA_OFFSET_H: u8 = 0x7D;
pub const ZA_OFFSET_L: u8 = 0x7E;

// ============================================================================
// MPU-9250 Register Values
// ============================================================================

/// MPU-9250 WHO_AM_I expected value
pub const MPU9250_WHO_AM_I_VALUE: u8 = 0x71;

/// MPU-6500 WHO_AM_I value (6-axis, no magnetometer)
pub const MPU6500_WHO_AM_I_VALUE: u8 = 0x70;

/// MPU-9255 WHO_AM_I value (9-axis, improved MPU-9250)
pub const MPU9255_WHO_AM_I_VALUE: u8 = 0x73;

/// PWR_MGMT_1 bits
pub const PWR_MGMT_1_H_RESET: u8 = 0x80;
pub const PWR_MGMT_1_SLEEP: u8 = 0x40;
pub const PWR_MGMT_1_CLKSEL_AUTO: u8 = 0x01;

/// INT_PIN_CFG bits
pub const INT_PIN_CFG_BYPASS_EN: u8 = 0x02;

/// USER_CTRL bits
pub const USER_CTRL_I2C_MST_EN: u8 = 0x20;
pub const USER_CTRL_SIG_COND_RST: u8 = 0x01;

// ============================================================================
// Gyroscope Full Scale Range
// ============================================================================

/// Gyroscope full scale range bits (GYRO_CONFIG[4:3])
pub const GYRO_FS_SEL_250DPS: u8 = 0x00;
pub const GYRO_FS_SEL_500DPS: u8 = 0x08;
pub const GYRO_FS_SEL_1000DPS: u8 = 0x10;
pub const GYRO_FS_SEL_2000DPS: u8 = 0x18;

// ============================================================================
// Accelerometer Full Scale Range
// ============================================================================

/// Accelerometer full scale range bits (ACCEL_CONFIG[4:3])
pub const ACCEL_FS_SEL_2G: u8 = 0x00;
pub const ACCEL_FS_SEL_4G: u8 = 0x08;
pub const ACCEL_FS_SEL_8G: u8 = 0x10;
pub const ACCEL_FS_SEL_16G: u8 = 0x18;

// ============================================================================
// Digital Low Pass Filter (DLPF) Configuration
// ============================================================================

/// Gyro DLPF bandwidth settings (CONFIG[2:0])
pub const DLPF_CFG_250HZ: u8 = 0x00; // 8kHz output rate
pub const DLPF_CFG_184HZ: u8 = 0x01; // 1kHz output rate
pub const DLPF_CFG_92HZ: u8 = 0x02;
pub const DLPF_CFG_41HZ: u8 = 0x03;
pub const DLPF_CFG_20HZ: u8 = 0x04;
pub const DLPF_CFG_10HZ: u8 = 0x05;
pub const DLPF_CFG_5HZ: u8 = 0x06;

/// Accelerometer DLPF bandwidth settings (ACCEL_CONFIG_2[2:0])
pub const ACCEL_DLPF_CFG_218HZ: u8 = 0x01;
pub const ACCEL_DLPF_CFG_99HZ: u8 = 0x02;
pub const ACCEL_DLPF_CFG_45HZ: u8 = 0x03;
pub const ACCEL_DLPF_CFG_21HZ: u8 = 0x04;
pub const ACCEL_DLPF_CFG_10HZ: u8 = 0x05;
pub const ACCEL_DLPF_CFG_5HZ: u8 = 0x06;

// ============================================================================
// AK8963 Magnetometer Registers
// ============================================================================

/// AK8963 device ID register
pub const AK8963_WIA: u8 = 0x00;

/// AK8963 information register
pub const AK8963_INFO: u8 = 0x01;

/// AK8963 status register 1 (data ready)
pub const AK8963_ST1: u8 = 0x02;

/// AK8963 measurement data (low byte first, unlike MPU-9250!)
pub const AK8963_HXL: u8 = 0x03;
pub const AK8963_HXH: u8 = 0x04;
pub const AK8963_HYL: u8 = 0x05;
pub const AK8963_HYH: u8 = 0x06;
pub const AK8963_HZL: u8 = 0x07;
pub const AK8963_HZH: u8 = 0x08;

/// AK8963 status register 2 (overflow, read to end measurement)
pub const AK8963_ST2: u8 = 0x09;

/// AK8963 control register 1 (mode, output bit)
pub const AK8963_CNTL1: u8 = 0x0A;

/// AK8963 control register 2 (soft reset)
pub const AK8963_CNTL2: u8 = 0x0B;

/// AK8963 self-test control
pub const AK8963_ASTC: u8 = 0x0C;

/// AK8963 sensitivity adjustment values (read in Fuse ROM access mode)
pub const AK8963_ASAX: u8 = 0x10;
pub const AK8963_ASAY: u8 = 0x11;
pub const AK8963_ASAZ: u8 = 0x12;

// ============================================================================
// AK8963 Register Values
// ============================================================================

/// AK8963 WHO_AM_I expected value
pub const AK8963_WHO_AM_I_VALUE: u8 = 0x48;

/// AK8963 ST1 bits
pub const AK8963_ST1_DRDY: u8 = 0x01; // Data ready

/// AK8963 ST2 bits
pub const AK8963_ST2_HOFL: u8 = 0x08; // Magnetic sensor overflow

/// AK8963 CNTL1 mode settings
pub const AK8963_MODE_POWER_DOWN: u8 = 0x00;
pub const AK8963_MODE_SINGLE_MEASURE: u8 = 0x01;
pub const AK8963_MODE_CONT_MEASURE_1: u8 = 0x02; // 8Hz
pub const AK8963_MODE_CONT_MEASURE_2: u8 = 0x06; // 100Hz
pub const AK8963_MODE_FUSE_ROM: u8 = 0x0F;

/// AK8963 CNTL1 output bit settings
pub const AK8963_BIT_14BIT: u8 = 0x00;
pub const AK8963_BIT_16BIT: u8 = 0x10;

/// AK8963 CNTL2 bits
pub const AK8963_CNTL2_SRST: u8 = 0x01; // Soft reset

// ============================================================================
// Scaling Constants
// ============================================================================

/// Gyroscope sensitivity (LSB/°/s) for each range
pub const GYRO_SENSITIVITY_250DPS: f32 = 131.0;
pub const GYRO_SENSITIVITY_500DPS: f32 = 65.5;
pub const GYRO_SENSITIVITY_1000DPS: f32 = 32.8;
pub const GYRO_SENSITIVITY_2000DPS: f32 = 16.4;

/// Accelerometer sensitivity (LSB/g) for each range
pub const ACCEL_SENSITIVITY_2G: f32 = 16384.0;
pub const ACCEL_SENSITIVITY_4G: f32 = 8192.0;
pub const ACCEL_SENSITIVITY_8G: f32 = 4096.0;
pub const ACCEL_SENSITIVITY_16G: f32 = 2048.0;

/// Magnetometer sensitivity: 4912 µT at 32760 LSB (16-bit mode)
pub const MAG_SENSITIVITY: f32 = 4912.0 / 32760.0; // µT/LSB

/// Temperature sensitivity and offset
pub const TEMP_SENSITIVITY: f32 = 333.87; // LSB/°C
pub const TEMP_OFFSET: f32 = 21.0; // °C at 0 LSB

/// Degrees to radians conversion
pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// Gravity constant (m/s²)
pub const GRAVITY: f32 = 9.80665;
