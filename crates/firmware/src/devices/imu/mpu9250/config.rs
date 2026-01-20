//! MPU-9250 Configuration
//!
//! Configuration structs for gyroscope, accelerometer, and magnetometer settings.

use super::registers;

/// Gyroscope full scale range
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±250 °/s
    Dps250,
    /// ±500 °/s
    Dps500,
    /// ±1000 °/s
    Dps1000,
    /// ±2000 °/s (default for high dynamics)
    #[default]
    Dps2000,
}

impl GyroRange {
    /// Get the register value for this range
    pub fn register_value(self) -> u8 {
        match self {
            GyroRange::Dps250 => registers::GYRO_FS_SEL_250DPS,
            GyroRange::Dps500 => registers::GYRO_FS_SEL_500DPS,
            GyroRange::Dps1000 => registers::GYRO_FS_SEL_1000DPS,
            GyroRange::Dps2000 => registers::GYRO_FS_SEL_2000DPS,
        }
    }

    /// Get the sensitivity (LSB per °/s) for this range
    pub fn sensitivity(self) -> f32 {
        match self {
            GyroRange::Dps250 => registers::GYRO_SENSITIVITY_250DPS,
            GyroRange::Dps500 => registers::GYRO_SENSITIVITY_500DPS,
            GyroRange::Dps1000 => registers::GYRO_SENSITIVITY_1000DPS,
            GyroRange::Dps2000 => registers::GYRO_SENSITIVITY_2000DPS,
        }
    }

    /// Get scale factor to convert raw value to rad/s
    pub fn scale_to_rad_s(self) -> f32 {
        registers::DEG_TO_RAD / self.sensitivity()
    }
}

/// Accelerometer full scale range
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelRange {
    /// ±2 g
    G2,
    /// ±4 g
    G4,
    /// ±8 g (default, good balance of range and resolution)
    #[default]
    G8,
    /// ±16 g
    G16,
}

impl AccelRange {
    /// Get the register value for this range
    pub fn register_value(self) -> u8 {
        match self {
            AccelRange::G2 => registers::ACCEL_FS_SEL_2G,
            AccelRange::G4 => registers::ACCEL_FS_SEL_4G,
            AccelRange::G8 => registers::ACCEL_FS_SEL_8G,
            AccelRange::G16 => registers::ACCEL_FS_SEL_16G,
        }
    }

    /// Get the sensitivity (LSB per g) for this range
    pub fn sensitivity(self) -> f32 {
        match self {
            AccelRange::G2 => registers::ACCEL_SENSITIVITY_2G,
            AccelRange::G4 => registers::ACCEL_SENSITIVITY_4G,
            AccelRange::G8 => registers::ACCEL_SENSITIVITY_8G,
            AccelRange::G16 => registers::ACCEL_SENSITIVITY_16G,
        }
    }

    /// Get scale factor to convert raw value to m/s²
    pub fn scale_to_m_s2(self) -> f32 {
        registers::GRAVITY / self.sensitivity()
    }
}

/// Digital Low Pass Filter configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DlpfConfig {
    /// 250Hz bandwidth (gyro) / 218Hz (accel) - highest bandwidth
    Bw250Hz,
    /// 184Hz bandwidth - default, good balance
    #[default]
    Bw184Hz,
    /// 92Hz bandwidth
    Bw92Hz,
    /// 41Hz bandwidth
    Bw41Hz,
    /// 20Hz bandwidth
    Bw20Hz,
    /// 10Hz bandwidth
    Bw10Hz,
    /// 5Hz bandwidth - lowest bandwidth, most smoothing
    Bw5Hz,
}

impl DlpfConfig {
    /// Get the register value for gyro DLPF
    pub fn gyro_register_value(self) -> u8 {
        match self {
            DlpfConfig::Bw250Hz => registers::DLPF_CFG_250HZ,
            DlpfConfig::Bw184Hz => registers::DLPF_CFG_184HZ,
            DlpfConfig::Bw92Hz => registers::DLPF_CFG_92HZ,
            DlpfConfig::Bw41Hz => registers::DLPF_CFG_41HZ,
            DlpfConfig::Bw20Hz => registers::DLPF_CFG_20HZ,
            DlpfConfig::Bw10Hz => registers::DLPF_CFG_10HZ,
            DlpfConfig::Bw5Hz => registers::DLPF_CFG_5HZ,
        }
    }

    /// Get the register value for accel DLPF
    pub fn accel_register_value(self) -> u8 {
        match self {
            DlpfConfig::Bw250Hz | DlpfConfig::Bw184Hz => registers::ACCEL_DLPF_CFG_218HZ,
            DlpfConfig::Bw92Hz => registers::ACCEL_DLPF_CFG_99HZ,
            DlpfConfig::Bw41Hz => registers::ACCEL_DLPF_CFG_45HZ,
            DlpfConfig::Bw20Hz => registers::ACCEL_DLPF_CFG_21HZ,
            DlpfConfig::Bw10Hz => registers::ACCEL_DLPF_CFG_10HZ,
            DlpfConfig::Bw5Hz => registers::ACCEL_DLPF_CFG_5HZ,
        }
    }
}

/// Magnetometer mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MagMode {
    /// Power down mode
    PowerDown,
    /// Single measurement mode
    SingleMeasure,
    /// Continuous measurement at 8Hz
    Continuous8Hz,
    /// Continuous measurement at 100Hz (default)
    #[default]
    Continuous100Hz,
}

impl MagMode {
    /// Get the register value for this mode (with 16-bit output)
    pub fn register_value(self) -> u8 {
        let mode = match self {
            MagMode::PowerDown => registers::AK8963_MODE_POWER_DOWN,
            MagMode::SingleMeasure => registers::AK8963_MODE_SINGLE_MEASURE,
            MagMode::Continuous8Hz => registers::AK8963_MODE_CONT_MEASURE_1,
            MagMode::Continuous100Hz => registers::AK8963_MODE_CONT_MEASURE_2,
        };
        mode | registers::AK8963_BIT_16BIT
    }
}

/// MPU-9250 driver configuration
#[derive(Debug, Clone, Copy)]
pub struct Mpu9250Config {
    /// Gyroscope full scale range
    pub gyro_range: GyroRange,

    /// Accelerometer full scale range
    pub accel_range: AccelRange,

    /// Gyroscope digital low pass filter
    pub gyro_dlpf: DlpfConfig,

    /// Accelerometer digital low pass filter
    pub accel_dlpf: DlpfConfig,

    /// Sample rate divider: SAMPLE_RATE = 1kHz / (1 + sample_rate_div)
    pub sample_rate_div: u8,

    /// Magnetometer mode
    pub mag_mode: MagMode,

    /// I2C address (0x68 or 0x69 depending on AD0 pin)
    pub i2c_address: u8,
}

impl Default for Mpu9250Config {
    fn default() -> Self {
        Self {
            gyro_range: GyroRange::Dps2000,
            accel_range: AccelRange::G8,
            gyro_dlpf: DlpfConfig::Bw184Hz,
            accel_dlpf: DlpfConfig::Bw184Hz,
            sample_rate_div: 0, // 1kHz ODR
            mag_mode: MagMode::Continuous100Hz,
            i2c_address: registers::MPU9250_ADDR,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gyro_range_scale() {
        let scale = GyroRange::Dps2000.scale_to_rad_s();
        // 2000°/s at 16.4 LSB/°/s, converted to rad/s
        let expected = core::f32::consts::PI / 180.0 / 16.4;
        assert!((scale - expected).abs() < 1e-6);
    }

    #[test]
    fn test_accel_range_scale() {
        let scale = AccelRange::G8.scale_to_m_s2();
        // 8g at 4096 LSB/g, converted to m/s²
        let expected = 9.80665 / 4096.0;
        assert!((scale - expected).abs() < 1e-6);
    }

    #[test]
    fn test_config_default() {
        let config = Mpu9250Config::default();
        assert_eq!(config.gyro_range, GyroRange::Dps2000);
        assert_eq!(config.accel_range, AccelRange::G8);
        assert_eq!(config.mag_mode, MagMode::Continuous100Hz);
        assert_eq!(config.i2c_address, 0x68);
    }

    #[test]
    fn test_mag_mode_register_value() {
        let value = MagMode::Continuous100Hz.register_value();
        // Should be mode 0x06 OR'd with 16-bit flag 0x10
        assert_eq!(value, 0x16);
    }
}
