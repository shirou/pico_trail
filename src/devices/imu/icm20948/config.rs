//! ICM-20948 Configuration
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

/// Digital Low Pass Filter configuration for gyroscope
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroDlpfConfig {
    /// 196.6 Hz bandwidth (NBW: 229.8 Hz)
    Bw197Hz,
    /// 151.8 Hz bandwidth (NBW: 187.6 Hz) - default
    #[default]
    Bw152Hz,
    /// 119.5 Hz bandwidth (NBW: 154.3 Hz)
    Bw120Hz,
    /// 51.2 Hz bandwidth (NBW: 73.3 Hz)
    Bw51Hz,
    /// 23.9 Hz bandwidth (NBW: 35.9 Hz)
    Bw24Hz,
    /// 11.6 Hz bandwidth (NBW: 17.8 Hz)
    Bw12Hz,
    /// 5.7 Hz bandwidth (NBW: 8.9 Hz)
    Bw6Hz,
    /// 361.4 Hz bandwidth (NBW: 376.5 Hz)
    Bw361Hz,
}

impl GyroDlpfConfig {
    /// Get the register value for gyro DLPF (combined with FCHOICE)
    pub fn register_value(self) -> u8 {
        let dlpf_cfg = match self {
            GyroDlpfConfig::Bw197Hz => registers::GYRO_DLPFCFG_196HZ,
            GyroDlpfConfig::Bw152Hz => registers::GYRO_DLPFCFG_152HZ,
            GyroDlpfConfig::Bw120Hz => registers::GYRO_DLPFCFG_120HZ,
            GyroDlpfConfig::Bw51Hz => registers::GYRO_DLPFCFG_51HZ,
            GyroDlpfConfig::Bw24Hz => registers::GYRO_DLPFCFG_24HZ,
            GyroDlpfConfig::Bw12Hz => registers::GYRO_DLPFCFG_12HZ,
            GyroDlpfConfig::Bw6Hz => registers::GYRO_DLPFCFG_6HZ,
            GyroDlpfConfig::Bw361Hz => registers::GYRO_DLPFCFG_361HZ,
        };
        // Enable DLPF by setting FCHOICE bit
        dlpf_cfg | registers::GYRO_FCHOICE
    }
}

/// Digital Low Pass Filter configuration for accelerometer
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelDlpfConfig {
    /// 246 Hz bandwidth (NBW: 265 Hz)
    Bw246Hz,
    /// 111.4 Hz bandwidth (NBW: 136 Hz) - default
    #[default]
    Bw111Hz,
    /// 50.4 Hz bandwidth (NBW: 68.8 Hz)
    Bw50Hz,
    /// 23.9 Hz bandwidth (NBW: 34.4 Hz)
    Bw24Hz,
    /// 11.5 Hz bandwidth (NBW: 17.0 Hz)
    Bw12Hz,
    /// 5.7 Hz bandwidth (NBW: 8.3 Hz)
    Bw6Hz,
    /// 473 Hz bandwidth (NBW: 499 Hz)
    Bw473Hz,
}

impl AccelDlpfConfig {
    /// Get the register value for accel DLPF (combined with FCHOICE)
    pub fn register_value(self) -> u8 {
        let dlpf_cfg = match self {
            AccelDlpfConfig::Bw246Hz => registers::ACCEL_DLPFCFG_246HZ,
            AccelDlpfConfig::Bw111Hz => registers::ACCEL_DLPFCFG_111HZ,
            AccelDlpfConfig::Bw50Hz => registers::ACCEL_DLPFCFG_50HZ,
            AccelDlpfConfig::Bw24Hz => registers::ACCEL_DLPFCFG_24HZ,
            AccelDlpfConfig::Bw12Hz => registers::ACCEL_DLPFCFG_12HZ,
            AccelDlpfConfig::Bw6Hz => registers::ACCEL_DLPFCFG_6HZ,
            AccelDlpfConfig::Bw473Hz => registers::ACCEL_DLPFCFG_473HZ,
        };
        // Enable DLPF by setting FCHOICE bit
        dlpf_cfg | registers::ACCEL_FCHOICE
    }
}

/// Magnetometer mode (AK09916)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MagMode {
    /// Power down mode
    PowerDown,
    /// Single measurement mode
    SingleMeasure,
    /// Continuous measurement at 10Hz
    Continuous10Hz,
    /// Continuous measurement at 20Hz
    Continuous20Hz,
    /// Continuous measurement at 50Hz
    Continuous50Hz,
    /// Continuous measurement at 100Hz (default)
    #[default]
    Continuous100Hz,
}

impl MagMode {
    /// Get the register value for this mode
    pub fn register_value(self) -> u8 {
        match self {
            MagMode::PowerDown => registers::AK09916_MODE_POWER_DOWN,
            MagMode::SingleMeasure => registers::AK09916_MODE_SINGLE,
            MagMode::Continuous10Hz => registers::AK09916_MODE_CONT_10HZ,
            MagMode::Continuous20Hz => registers::AK09916_MODE_CONT_20HZ,
            MagMode::Continuous50Hz => registers::AK09916_MODE_CONT_50HZ,
            MagMode::Continuous100Hz => registers::AK09916_MODE_CONT_100HZ,
        }
    }
}

/// Register bank selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RegisterBank {
    /// Bank 0: User configuration and sensor data
    #[default]
    Bank0,
    /// Bank 1: Self-test data
    Bank1,
    /// Bank 2: Sensor configuration (gyro/accel ranges, DLPF)
    Bank2,
    /// Bank 3: I2C master configuration
    Bank3,
}

impl RegisterBank {
    /// Get the register value for bank selection
    pub fn register_value(self) -> u8 {
        match self {
            RegisterBank::Bank0 => registers::BANK_0,
            RegisterBank::Bank1 => registers::BANK_1,
            RegisterBank::Bank2 => registers::BANK_2,
            RegisterBank::Bank3 => registers::BANK_3,
        }
    }
}

/// ICM-20948 driver configuration
#[derive(Debug, Clone, Copy)]
pub struct Icm20948Config {
    /// Gyroscope full scale range
    pub gyro_range: GyroRange,

    /// Accelerometer full scale range
    pub accel_range: AccelRange,

    /// Gyroscope digital low pass filter
    pub gyro_dlpf: GyroDlpfConfig,

    /// Accelerometer digital low pass filter
    pub accel_dlpf: AccelDlpfConfig,

    /// Gyroscope sample rate divider: ODR = 1.125kHz / (1 + div)
    pub gyro_sample_rate_div: u8,

    /// Accelerometer sample rate divider: ODR = 1.125kHz / (1 + div)
    /// Note: This is a 12-bit value (0-4095), but typically 0-255 is used
    pub accel_sample_rate_div: u16,

    /// Magnetometer mode
    pub mag_mode: MagMode,

    /// I2C address (0x68 or 0x69 depending on AD0 pin)
    pub i2c_address: u8,
}

impl Default for Icm20948Config {
    fn default() -> Self {
        Self {
            gyro_range: GyroRange::Dps2000,
            accel_range: AccelRange::G8,
            gyro_dlpf: GyroDlpfConfig::Bw152Hz,
            accel_dlpf: AccelDlpfConfig::Bw111Hz,
            gyro_sample_rate_div: 0,  // 1.125kHz ODR
            accel_sample_rate_div: 0, // 1.125kHz ODR
            mag_mode: MagMode::Continuous100Hz,
            i2c_address: registers::ICM20948_ADDR,
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
        let config = Icm20948Config::default();
        assert_eq!(config.gyro_range, GyroRange::Dps2000);
        assert_eq!(config.accel_range, AccelRange::G8);
        assert_eq!(config.mag_mode, MagMode::Continuous100Hz);
        assert_eq!(config.i2c_address, 0x68);
    }

    #[test]
    fn test_mag_mode_register_value() {
        assert_eq!(MagMode::Continuous100Hz.register_value(), 0x08);
        assert_eq!(MagMode::PowerDown.register_value(), 0x00);
    }

    #[test]
    fn test_register_bank_values() {
        assert_eq!(RegisterBank::Bank0.register_value(), 0x00);
        assert_eq!(RegisterBank::Bank1.register_value(), 0x10);
        assert_eq!(RegisterBank::Bank2.register_value(), 0x20);
        assert_eq!(RegisterBank::Bank3.register_value(), 0x30);
    }

    #[test]
    fn test_gyro_dlpf_includes_fchoice() {
        // FCHOICE bit (0x01) should be set
        let value = GyroDlpfConfig::Bw152Hz.register_value();
        assert_eq!(value & 0x01, 0x01); // FCHOICE enabled
    }

    #[test]
    fn test_accel_dlpf_includes_fchoice() {
        // FCHOICE bit (0x01) should be set
        let value = AccelDlpfConfig::Bw111Hz.register_value();
        assert_eq!(value & 0x01, 0x01); // FCHOICE enabled
    }
}
