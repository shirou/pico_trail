//! Board pin configuration types and validation
//!
//! This module defines the types for GPIO pin configuration management,
//! supporting hwdef.dat-style board definitions with build-time code generation.

/// Pin type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinType {
    /// Digital input
    Input,
    /// Digital output (default for actuators)
    Output,
    /// Analog input
    Adc,
}

/// Pull resistor configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullMode {
    /// No pull resistors (default)
    None,
    /// Enable internal pull-up
    PullUp,
    /// Enable internal pull-down
    PullDown,
}

/// Output mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    /// Push-pull output (default)
    PushPull,
    /// Open-drain output
    OpenDrain,
}

/// Pin speed/slew rate configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    /// Low slew rate (reduced EMI)
    Low,
    /// Medium slew rate (default)
    Medium,
    /// High slew rate (fast switching)
    High,
    /// Very high slew rate (maximum speed, not all platforms)
    VeryHigh,
}

/// Pin configuration with modifiers
#[derive(Debug, Clone, Copy)]
pub struct PinConfig {
    /// GPIO pin number
    pub gpio: u8,
    /// Pin type (input, output, ADC)
    pub pin_type: PinType,
    /// Pull resistor configuration
    pub pull: PullMode,
    /// Output mode (push-pull or open-drain)
    pub output_mode: OutputMode,
    /// Pin speed/slew rate
    pub speed: Speed,
}

/// Motor driver H-bridge pin configuration
#[derive(Debug, Clone, Copy)]
pub struct MotorPins {
    /// IN1 pin configuration (direction control)
    pub in1: PinConfig,
    /// IN2 pin configuration (direction control)
    pub in2: PinConfig,
}

/// Board-level pin configuration
///
/// This structure defines GPIO pin assignments for all peripherals.
/// It is typically generated at build time from hwdef.dat files and
/// can be overridden at runtime via parameter store.
#[derive(Debug, Clone)]
pub struct BoardPinConfig {
    /// Motor pin configurations (4 motors for Freenove 4WD)
    pub motors: [MotorPins; 4],
    /// Optional buzzer pin configuration
    pub buzzer: Option<PinConfig>,
    /// Optional WS2812 LED pin configuration
    pub led: Option<PinConfig>,
    /// Optional battery ADC pin configuration
    pub battery_adc: Option<PinConfig>,
}

/// Pin configuration errors
#[derive(Debug, PartialEq, Eq)]
pub enum PinError {
    /// Pin is assigned to multiple functions
    DuplicatePin(u8),
    /// GPIO number is outside valid range for platform
    InvalidGpio(u8),
    /// Failed to parse parameter value (error message)
    ParameterParseError,
    /// Reserved pin used (GPIO number, purpose)
    ReservedPinUsed(u8, &'static str),
    /// Unsupported pin modifier for platform
    UnsupportedModifier(&'static str),
}

impl BoardPinConfig {
    /// Validate pin configuration
    ///
    /// Checks that:
    /// - No duplicate pin assignments
    /// - All GPIO numbers are within valid range for platform
    /// - ADC pins use valid ADC channels (GPIO 26-29 on RP2350)
    ///
    /// # Errors
    ///
    /// Returns `PinError::DuplicatePin` if a pin is used multiple times
    /// Returns `PinError::InvalidGpio` if a GPIO number is invalid
    /// Returns `PinError::InvalidGpio` if ADC pin is not on a valid ADC channel
    pub fn validate(&self) -> Result<(), PinError> {
        use heapless::FnvIndexSet;

        const RP2350_MAX_GPIO: u8 = 29;
        const RP2350_ADC_MIN: u8 = 26;
        const RP2350_ADC_MAX: u8 = 29;

        let mut used_pins = FnvIndexSet::<u8, 32>::new();

        // Helper to check and register a pin
        let mut check_pin = |pin: &PinConfig| -> Result<(), PinError> {
            // Check GPIO range
            if pin.gpio > RP2350_MAX_GPIO {
                return Err(PinError::InvalidGpio(pin.gpio));
            }

            // Check ADC pin validity
            if pin.pin_type == PinType::Adc
                && (pin.gpio < RP2350_ADC_MIN || pin.gpio > RP2350_ADC_MAX)
            {
                return Err(PinError::InvalidGpio(pin.gpio));
            }

            // Check for duplicates
            if !used_pins.insert(pin.gpio).unwrap_or(false) {
                return Err(PinError::DuplicatePin(pin.gpio));
            }

            Ok(())
        };

        // Check motor pins
        for motor in &self.motors {
            check_pin(&motor.in1)?;
            check_pin(&motor.in2)?;
        }

        // Check optional peripherals
        if let Some(ref pin) = self.buzzer {
            check_pin(pin)?;
        }
        if let Some(ref pin) = self.led {
            check_pin(pin)?;
        }
        if let Some(ref pin) = self.battery_adc {
            check_pin(pin)?;
        }

        Ok(())
    }

    /// Load configuration from parameter store with board defaults as fallback
    ///
    /// Checks parameter store for pin overrides (e.g., PIN_M1_IN1).
    /// Falls back to board_default values if no override is set.
    /// Validates the resulting configuration.
    ///
    /// # Errors
    ///
    /// Returns `PinError` if validation fails or parameter parsing fails
    pub fn load(
        params: &crate::parameters::storage::ParameterStore,
        board_default: &Self,
    ) -> Result<Self, PinError> {
        use crate::parameters::storage::ParamValue;

        // Helper to get pin override or use default
        let get_pin = |param_name: &str, default: &PinConfig| -> Result<PinConfig, PinError> {
            if let Some(value) = params.get(param_name) {
                match value {
                    ParamValue::Int(gpio) => {
                        if *gpio < 0 {
                            return Err(PinError::ParameterParseError);
                        }
                        Ok(PinConfig {
                            gpio: *gpio as u8,
                            ..*default
                        })
                    }
                    _ => Err(PinError::ParameterParseError),
                }
            } else {
                Ok(*default)
            }
        };

        // Load motor pins with parameter overrides
        let motors = [
            MotorPins {
                in1: get_pin("PIN_M1_IN1", &board_default.motors[0].in1)?,
                in2: get_pin("PIN_M1_IN2", &board_default.motors[0].in2)?,
            },
            MotorPins {
                in1: get_pin("PIN_M2_IN1", &board_default.motors[1].in1)?,
                in2: get_pin("PIN_M2_IN2", &board_default.motors[1].in2)?,
            },
            MotorPins {
                in1: get_pin("PIN_M3_IN1", &board_default.motors[2].in1)?,
                in2: get_pin("PIN_M3_IN2", &board_default.motors[2].in2)?,
            },
            MotorPins {
                in1: get_pin("PIN_M4_IN1", &board_default.motors[3].in1)?,
                in2: get_pin("PIN_M4_IN2", &board_default.motors[3].in2)?,
            },
        ];

        // Load optional peripheral pins with parameter overrides
        let buzzer = if let Some(ref default_pin) = board_default.buzzer {
            Some(get_pin("PIN_BUZZER", default_pin)?)
        } else {
            None
        };

        let led = if let Some(ref default_pin) = board_default.led {
            Some(get_pin("PIN_LED", default_pin)?)
        } else {
            None
        };

        let battery_adc = if let Some(ref default_pin) = board_default.battery_adc {
            Some(get_pin("PIN_BATTERY_ADC", default_pin)?)
        } else {
            None
        };

        let config = BoardPinConfig {
            motors,
            buzzer,
            led,
            battery_adc,
        };

        // Validate the loaded configuration
        config.validate()?;

        Ok(config)
    }
}

// Include generated board configuration from build.rs
// This file is generated at build time from boards/*.hwdef files
include!(concat!(env!("OUT_DIR"), "/board_config.rs"));

#[cfg(test)]
mod tests {
    use super::*;

    fn create_output_pin(gpio: u8) -> PinConfig {
        PinConfig {
            gpio,
            pin_type: PinType::Output,
            pull: PullMode::None,
            output_mode: OutputMode::PushPull,
            speed: Speed::Medium,
        }
    }

    fn create_adc_pin(gpio: u8) -> PinConfig {
        PinConfig {
            gpio,
            pin_type: PinType::Adc,
            pull: PullMode::None,
            output_mode: OutputMode::PushPull,
            speed: Speed::Medium,
        }
    }

    #[test]
    fn test_motor_pins_creation() {
        let pins = MotorPins {
            in1: create_output_pin(18),
            in2: create_output_pin(19),
        };
        assert_eq!(pins.in1.gpio, 18);
        assert_eq!(pins.in2.gpio, 19);
        assert_eq!(pins.in1.pin_type, PinType::Output);
    }

    #[test]
    fn test_board_pin_config_creation() {
        let config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: Some(create_output_pin(2)),
            led: Some(create_output_pin(16)),
            battery_adc: Some(create_adc_pin(26)),
        };

        assert_eq!(config.motors.len(), 4);
        assert!(config.buzzer.is_some());
        assert_eq!(config.buzzer.unwrap().gpio, 2);
    }

    #[test]
    fn test_validate_valid_config() {
        let config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: Some(create_output_pin(2)),
            led: Some(create_output_pin(16)),
            battery_adc: Some(create_adc_pin(26)),
        };

        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_validate_duplicate_motor_pins() {
        let config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(18), // Duplicate!
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: None,
            led: None,
            battery_adc: None,
        };

        assert_eq!(config.validate(), Err(PinError::DuplicatePin(18)));
    }

    #[test]
    fn test_validate_duplicate_motor_and_peripheral() {
        let config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: Some(create_output_pin(18)), // Duplicate with motor!
            led: None,
            battery_adc: None,
        };

        assert_eq!(config.validate(), Err(PinError::DuplicatePin(18)));
    }

    #[test]
    fn test_validate_invalid_gpio_number() {
        let config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(35), // Invalid GPIO (max is 29)
                },
            ],
            buzzer: None,
            led: None,
            battery_adc: None,
        };

        assert_eq!(config.validate(), Err(PinError::InvalidGpio(35)));
    }

    #[test]
    fn test_validate_invalid_adc_pin() {
        let config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: None,
            led: None,
            battery_adc: Some(create_adc_pin(10)), // Invalid ADC pin (must be 26-29)
        };

        assert_eq!(config.validate(), Err(PinError::InvalidGpio(10)));
    }

    #[test]
    fn test_validate_valid_adc_pins() {
        // Test all valid ADC pins (26-29)
        for gpio in 26..=29 {
            let config = BoardPinConfig {
                motors: [
                    MotorPins {
                        in1: create_output_pin(18),
                        in2: create_output_pin(19),
                    },
                    MotorPins {
                        in1: create_output_pin(20),
                        in2: create_output_pin(21),
                    },
                    MotorPins {
                        in1: create_output_pin(6),
                        in2: create_output_pin(7),
                    },
                    MotorPins {
                        in1: create_output_pin(8),
                        in2: create_output_pin(9),
                    },
                ],
                buzzer: None,
                led: None,
                battery_adc: Some(create_adc_pin(gpio)),
            };

            assert!(
                config.validate().is_ok(),
                "GPIO {} should be valid for ADC",
                gpio
            );
        }
    }

    #[test]
    fn test_load_with_defaults() {
        use crate::parameters::storage::ParameterStore;

        let default_config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(19),
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: Some(create_output_pin(2)),
            led: Some(create_output_pin(16)),
            battery_adc: Some(create_adc_pin(26)),
        };

        let params = ParameterStore::default();
        let loaded = BoardPinConfig::load(&params, &default_config).unwrap();

        assert_eq!(loaded.motors[0].in1.gpio, 18);
        assert_eq!(loaded.motors[0].in2.gpio, 19);
        assert_eq!(loaded.buzzer.unwrap().gpio, 2);
        assert_eq!(loaded.led.unwrap().gpio, 16);
        assert_eq!(loaded.battery_adc.unwrap().gpio, 26);
    }

    #[test]
    fn test_load_detects_invalid_config() {
        use crate::parameters::storage::ParameterStore;

        // Create config with duplicate pins
        let invalid_config = BoardPinConfig {
            motors: [
                MotorPins {
                    in1: create_output_pin(18),
                    in2: create_output_pin(18), // Duplicate!
                },
                MotorPins {
                    in1: create_output_pin(20),
                    in2: create_output_pin(21),
                },
                MotorPins {
                    in1: create_output_pin(6),
                    in2: create_output_pin(7),
                },
                MotorPins {
                    in1: create_output_pin(8),
                    in2: create_output_pin(9),
                },
            ],
            buzzer: None,
            led: None,
            battery_adc: None,
        };

        let params = ParameterStore::default();
        // Load should fail due to validation
        let result = BoardPinConfig::load(&params, &invalid_config);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), PinError::DuplicatePin(18));
    }
}
