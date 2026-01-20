//! Board Pin Configuration Parameters
//!
//! Defines board pin configuration parameters for GPIO assignments.
//!
//! # Parameters
//!
//! Motor pins (4 motors × 2 pins = 8 parameters):
//! - `PIN_M1_IN1`, `PIN_M1_IN2` - Motor 1 (left front) control pins
//! - `PIN_M2_IN1`, `PIN_M2_IN2` - Motor 2 (left rear) control pins
//! - `PIN_M3_IN1`, `PIN_M3_IN2` - Motor 3 (right front) control pins
//! - `PIN_M4_IN1`, `PIN_M4_IN2` - Motor 4 (right rear) control pins
//!
//! Optional peripheral pins:
//! - `PIN_BUZZER` - Piezo buzzer GPIO
//! - `PIN_LED` - WS2812 RGB LED data line
//! - `PIN_BATTERY_ADC` - Battery voltage monitoring ADC pin
//!
//! # Custom Parameter Exception
//!
//! **NOTE**: These `PIN_*` parameters are **custom parameters** that do not exist
//! in ArduPilot's standard parameter list. This is an intentional exception to
//! the project's parameter policy (see CLAUDE.md).
//!
//! ## Rationale
//!
//! ArduPilot uses hwdef.dat files for compile-time pin configuration (similar
//! to this project's approach in `boards/*.hwdef`). However, for development
//! and testing flexibility, we provide runtime parameter overrides via `PIN_*`
//! parameters accessible through Mission Planner.
//!
//! ## Usage Pattern
//!
//! - **Production**: Use hwdef files (`boards/*.hwdef`) for pin configuration
//! - **Development/Testing**: Override pins via `PIN_*` parameters in Mission Planner
//! - **Safety**: Pin changes require reboot and validation before hardware use
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::parameters::{ParameterStore, ParamValue};
//! use pico_trail::parameters::board::BoardParams;
//! use pico_trail::platform::rp2350::Rp2350Flash;
//!
//! let mut flash = Rp2350Flash::new();
//! let mut store = ParameterStore::load_from_flash(&mut flash).unwrap();
//!
//! // Register board pin parameters with hwdef defaults
//! BoardParams::register_defaults(&mut store);
//!
//! // Override a motor pin for testing
//! store.set("PIN_M1_IN1", ParamValue::Int(22)).unwrap();
//! ```

use super::storage::ParameterStore;
use crate::platform::Result;

/// Board pin configuration parameters
///
/// This struct provides methods to register and load board pin parameters
/// from the parameter store. Default values come from the hwdef-generated
/// BOARD_CONFIG constant.
pub struct BoardParams;

impl BoardParams {
    /// Register board pin parameters with hwdef defaults
    ///
    /// Registers all PIN_* parameters in the parameter store with default
    /// values from the hwdef-generated BOARD_CONFIG.
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
        // Get hwdef defaults from generated BOARD_CONFIG
        // This is defined in the generated board_config.rs included by platform modules
        use super::storage::{ParamFlags, ParamValue};
        use crate::platform::traits::board::BOARD_CONFIG;

        // Register motor pin parameters (8 parameters)
        // Motor 1 (left front)
        store.register(
            "PIN_M1_IN1",
            ParamValue::Int(BOARD_CONFIG.motors[0].in1.gpio as i32),
            ParamFlags::empty(),
        )?;
        store.register(
            "PIN_M1_IN2",
            ParamValue::Int(BOARD_CONFIG.motors[0].in2.gpio as i32),
            ParamFlags::empty(),
        )?;

        // Motor 2 (left rear)
        store.register(
            "PIN_M2_IN1",
            ParamValue::Int(BOARD_CONFIG.motors[1].in1.gpio as i32),
            ParamFlags::empty(),
        )?;
        store.register(
            "PIN_M2_IN2",
            ParamValue::Int(BOARD_CONFIG.motors[1].in2.gpio as i32),
            ParamFlags::empty(),
        )?;

        // Motor 3 (right front)
        store.register(
            "PIN_M3_IN1",
            ParamValue::Int(BOARD_CONFIG.motors[2].in1.gpio as i32),
            ParamFlags::empty(),
        )?;
        store.register(
            "PIN_M3_IN2",
            ParamValue::Int(BOARD_CONFIG.motors[2].in2.gpio as i32),
            ParamFlags::empty(),
        )?;

        // Motor 4 (right rear)
        store.register(
            "PIN_M4_IN1",
            ParamValue::Int(BOARD_CONFIG.motors[3].in1.gpio as i32),
            ParamFlags::empty(),
        )?;
        store.register(
            "PIN_M4_IN2",
            ParamValue::Int(BOARD_CONFIG.motors[3].in2.gpio as i32),
            ParamFlags::empty(),
        )?;

        // Register optional peripheral pins (3 parameters)
        if let Some(buzzer) = BOARD_CONFIG.buzzer {
            store.register(
                "PIN_BUZZER",
                ParamValue::Int(buzzer.gpio as i32),
                ParamFlags::empty(),
            )?;
        }

        if let Some(led) = BOARD_CONFIG.led {
            store.register(
                "PIN_LED",
                ParamValue::Int(led.gpio as i32),
                ParamFlags::empty(),
            )?;
        }

        if let Some(battery_adc) = BOARD_CONFIG.battery_adc {
            store.register(
                "PIN_BATTERY_ADC",
                ParamValue::Int(battery_adc.gpio as i32),
                ParamFlags::empty(),
            )?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;

    #[test]
    fn test_register_defaults() {
        let mut flash = MockFlash::new();
        let mut store = ParameterStore::load_from_flash(&mut flash).unwrap_or_default();

        // Register board pin parameters
        let result = BoardParams::register_defaults(&mut store);
        assert!(result.is_ok());

        // Verify motor pin parameters registered
        use crate::parameters::storage::ParamValue;
        use crate::platform::traits::board::BOARD_CONFIG;

        // Motor 1
        assert_eq!(
            store.get("PIN_M1_IN1"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[0].in1.gpio as i32))
        );
        assert_eq!(
            store.get("PIN_M1_IN2"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[0].in2.gpio as i32))
        );

        // Motor 2
        assert_eq!(
            store.get("PIN_M2_IN1"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[1].in1.gpio as i32))
        );
        assert_eq!(
            store.get("PIN_M2_IN2"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[1].in2.gpio as i32))
        );

        // Motor 3
        assert_eq!(
            store.get("PIN_M3_IN1"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[2].in1.gpio as i32))
        );
        assert_eq!(
            store.get("PIN_M3_IN2"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[2].in2.gpio as i32))
        );

        // Motor 4
        assert_eq!(
            store.get("PIN_M4_IN1"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[3].in1.gpio as i32))
        );
        assert_eq!(
            store.get("PIN_M4_IN2"),
            Some(&ParamValue::Int(BOARD_CONFIG.motors[3].in2.gpio as i32))
        );

        // Optional peripherals
        if BOARD_CONFIG.buzzer.is_some() {
            assert!(store.get("PIN_BUZZER").is_some());
        }
        if BOARD_CONFIG.led.is_some() {
            assert!(store.get("PIN_LED").is_some());
        }
        if BOARD_CONFIG.battery_adc.is_some() {
            assert!(store.get("PIN_BATTERY_ADC").is_some());
        }
    }

    #[test]
    fn test_idempotent_registration() {
        let mut flash = MockFlash::new();
        let mut store = ParameterStore::load_from_flash(&mut flash).unwrap_or_default();

        // Register twice
        BoardParams::register_defaults(&mut store).unwrap();
        BoardParams::register_defaults(&mut store).unwrap();

        // Should not error (parameters already exist, register() is idempotent)
    }
}
