//! Arming Parameter Definitions
//!
//! Defines arming-related parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `ARMING_CHECK` - Pre-arm safety checks bitmask (u32, **visible in GCS**)
//! - `ARMING_OPTIONS` - Arming options bitmask (u32, **visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard arming parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#arming-check-arm-checks-to-perform
//! - https://ardupilot.org/rover/docs/parameters.html#arming-options-arming-options
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::parameters::{ParameterStore, ParamValue};
//! use pico_trail::parameters::arming::ArmingParams;
//! use pico_trail::platform::rp2350::Rp2350Flash;
//!
//! let mut flash = Rp2350Flash::new();
//! let mut store = ParameterStore::load_from_flash(&mut flash).unwrap();
//!
//! // Register arming parameters with defaults
//! ArmingParams::register_defaults(&mut store);
//!
//! // Load arming configuration
//! let arming_config = ArmingParams::from_store(&store);
//! ```

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;

/// ARMING_CHECK bit definitions (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmingCheckBits;

impl ArmingCheckBits {
    /// Check all parameters (0xFFFF = all enabled)
    pub const ALL: u32 = 0xFFFF;
    /// Check barometer
    pub const BARO: u32 = 1 << 0;
    /// Check compass
    pub const COMPASS: u32 = 1 << 1;
    /// Check GPS lock
    pub const GPS: u32 = 1 << 2;
    /// Check INS (accelerometers + gyros)
    pub const INS: u32 = 1 << 3;
    /// Check board voltage
    pub const VOLTAGE: u32 = 1 << 4;
    /// Check battery level
    pub const BATTERY: u32 = 1 << 5;
    /// Check RC channels
    pub const RC: u32 = 1 << 6;
    /// Check parameters have been loaded
    pub const PARAMETERS: u32 = 1 << 7;
    /// Default checks (basic safety)
    pub const DEFAULT: u32 = Self::VOLTAGE | Self::BATTERY | Self::RC | Self::PARAMETERS;
}

/// ARMING_OPTIONS bit definitions (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmingOptionsBits;

impl ArmingOptionsBits {
    /// No special options
    pub const NONE: u32 = 0;
    /// Require GPS lock before arming
    pub const REQUIRE_GPS: u32 = 1 << 0;
    /// Disable pre-arm checks
    pub const DISABLE_PREARM: u32 = 1 << 1;
}

/// ARMING_RUDDER values (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmingRudder {
    /// Disabled
    Disabled = 0,
    /// ArmOnly - only arm
    ArmOnly = 1,
    /// ArmOrDisarm - arm or disarm
    ArmOrDisarm = 2,
}

/// Arming parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct ArmingParams {
    /// Pre-arm check bitmask (ARMING_CHECK)
    pub check_bitmask: u32,
    /// Arming options bitmask (ARMING_OPTIONS)
    pub options_bitmask: u32,
    /// Require arming before motors enabled (ARMING_REQUIRE)
    pub require: bool,
    /// Maximum accelerometer error m/s² (ARMING_ACCTHRESH)
    pub acc_threshold: f32,
    /// Rudder stick arming method (ARMING_RUDDER)
    pub rudder: u8,
}

impl ArmingParams {
    /// Register arming parameters with default values
    ///
    /// Registers all arming parameters in the parameter store with sensible defaults.
    /// Parameters are only registered if they don't already exist.
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<()> {
        // ARMING_CHECK - Default to basic safety checks
        store.register(
            "ARMING_CHECK",
            ParamValue::Int(ArmingCheckBits::DEFAULT as i32),
            ParamFlags::empty(),
        )?;

        // ARMING_OPTIONS - Default to no special options
        store.register(
            "ARMING_OPTIONS",
            ParamValue::Int(ArmingOptionsBits::NONE as i32),
            ParamFlags::empty(),
        )?;

        // ARMING_REQUIRE - Default to required (true = 1)
        store.register("ARMING_REQUIRE", ParamValue::Int(1), ParamFlags::empty())?;

        // ARMING_ACCTHRESH - Default to 0.75 m/s²
        store.register(
            "ARMING_ACCTHRESH",
            ParamValue::Float(0.75),
            ParamFlags::empty(),
        )?;

        // ARMING_RUDDER - Default to ArmOrDisarm (2)
        store.register(
            "ARMING_RUDDER",
            ParamValue::Int(ArmingRudder::ArmOrDisarm as i32),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load arming parameters from parameter store
    ///
    /// Reads arming configuration from the parameter store.
    /// Falls back to defaults if parameters are not found.
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Arming parameters with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        let check_bitmask = match store.get("ARMING_CHECK") {
            Some(ParamValue::Int(v)) => *v as u32,
            Some(ParamValue::Float(v)) => *v as u32,
            _ => ArmingCheckBits::DEFAULT,
        };

        let options_bitmask = match store.get("ARMING_OPTIONS") {
            Some(ParamValue::Int(v)) => *v as u32,
            Some(ParamValue::Float(v)) => *v as u32,
            _ => ArmingOptionsBits::NONE,
        };

        let require = match store.get("ARMING_REQUIRE") {
            Some(ParamValue::Int(v)) => *v != 0,
            Some(ParamValue::Float(v)) => *v != 0.0,
            _ => true,
        };

        let acc_threshold = match store.get("ARMING_ACCTHRESH") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 0.75,
        };

        let rudder = match store.get("ARMING_RUDDER") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => ArmingRudder::ArmOrDisarm as u8,
        };

        Self {
            check_bitmask,
            options_bitmask,
            require,
            acc_threshold,
            rudder,
        }
    }

    /// Check if a specific arming check is enabled
    ///
    /// # Arguments
    ///
    /// * `check_bit` - Bit to check (use ArmingCheckBits constants)
    ///
    /// # Returns
    ///
    /// true if the check is enabled
    pub fn is_check_enabled(&self, check_bit: u32) -> bool {
        (self.check_bitmask & check_bit) != 0
    }

    /// Check if a specific arming option is enabled
    ///
    /// # Arguments
    ///
    /// * `option_bit` - Bit to check (use ArmingOptionsBits constants)
    ///
    /// # Returns
    ///
    /// true if the option is enabled
    pub fn is_option_enabled(&self, option_bit: u32) -> bool {
        (self.options_bitmask & option_bit) != 0
    }

    /// Check if arming configuration is valid
    pub fn is_configured(&self) -> bool {
        // Always considered configured (uses defaults if not set)
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_arming_check_bits() {
        let params = ArmingParams {
            check_bitmask: ArmingCheckBits::VOLTAGE | ArmingCheckBits::BATTERY,
            options_bitmask: ArmingOptionsBits::NONE,
            require: true,
            acc_threshold: 0.75,
            rudder: 0,
        };

        assert!(params.is_check_enabled(ArmingCheckBits::VOLTAGE));
        assert!(params.is_check_enabled(ArmingCheckBits::BATTERY));
        assert!(!params.is_check_enabled(ArmingCheckBits::GPS));
    }

    #[test]
    fn test_arming_options_bits() {
        let params = ArmingParams {
            check_bitmask: ArmingCheckBits::DEFAULT,
            options_bitmask: ArmingOptionsBits::REQUIRE_GPS,
            require: true,
            acc_threshold: 0.75,
            rudder: 0,
        };

        assert!(params.is_option_enabled(ArmingOptionsBits::REQUIRE_GPS));
        assert!(!params.is_option_enabled(ArmingOptionsBits::DISABLE_PREARM));
    }
}
