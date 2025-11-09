//! RP2350 motor driver implementation
//!
//! This module provides RP2350-specific motor driver support using PWM for H-bridge control.
//! It integrates with the motor_driver library to provide platform-specific PWM pin implementations.
//!
//! ## Usage Example - Single Motor
//!
//! ```no_run
//! use pico_trail::libraries::motor_driver::Motor;
//! use pico_trail::platform::rp2350::init_motor;
//! use pico_trail::init_motor_from_slice;
//! use rp235x_hal::pwm::Slices;
//!
//! // Initialize PWM slices from peripherals
//! let mut pwm_slices = Slices::new(peripherals.PWM, &mut peripherals.RESETS);
//!
//! // Method 1: Using helper macro (recommended)
//! let mut motor1 = init_motor_from_slice!(pwm_slices.pwm1, pins.gpio18, pins.gpio19);
//!
//! // Method 2: Manual initialization
//! let _pin1 = pins.gpio20.into_function::<rp235x_hal::gpio::FunctionPwm>();
//! let _pin2 = pins.gpio21.into_function::<rp235x_hal::gpio::FunctionPwm>();
//! let slice2 = pico_trail::platform::rp2350::init_motor_pwm_slice(pwm_slices.pwm2.into_mode());
//! let mut motor2 = init_motor(slice2);
//!
//! // Control motors
//! motor1.set_speed(0.75)?; // 75% forward
//! motor2.set_speed(-0.5)?; // 50% reverse
//! motor1.brake()?;         // Brake
//! motor2.stop()?;          // Coast
//! ```
//!
//! ## Usage Example - Freenove 4WD Car (4 Motors)
//!
//! ```no_run
//! use pico_trail::libraries::motor_driver::Motor;
//! use pico_trail::init_motor_from_slice;
//! use rp235x_hal::pwm::Slices;
//!
//! // Initialize PWM slices
//! let mut pwm_slices = Slices::new(peripherals.PWM, &mut peripherals.RESETS);
//!
//! // Initialize all 4 motors for Freenove 4WD Car using macro
//! let mut motor1 = init_motor_from_slice!(pwm_slices.pwm1, pins.gpio18, pins.gpio19);
//! let mut motor2 = init_motor_from_slice!(pwm_slices.pwm2, pins.gpio20, pins.gpio21);
//! let mut motor3 = init_motor_from_slice!(pwm_slices.pwm3, pins.gpio6, pins.gpio7);
//! let mut motor4 = init_motor_from_slice!(pwm_slices.pwm4, pins.gpio8, pins.gpio9);
//!
//! // Control motors (differential drive example)
//! // Left side: motor1 + motor2, Right side: motor3 + motor4
//! motor1.set_speed(0.5)?;  // Left front forward
//! motor2.set_speed(0.5)?;  // Left rear forward
//! motor3.set_speed(0.5)?;  // Right front forward
//! motor4.set_speed(0.5)?;  // Right rear forward
//! ```
//!
//! ## Pin Mapping for Freenove 4WD Car
//!
//! The Freenove 4WD Car uses 4 motors with the following GPIO pin assignments:
//!
//! | Motor | IN1 GPIO | IN2 GPIO | PWM Slice |
//! |-------|----------|----------|-----------|
//! | M1    | 18       | 19       | Slice 1   |
//! | M2    | 20       | 21       | Slice 2   |
//! | M3    | 6        | 7        | Slice 3   |
//! | M4    | 8        | 9        | Slice 4   |

#[cfg(feature = "pico2_w")]
use crate::libraries::motor_driver::{HBridgeMotor, MotorError, PwmPin};
#[cfg(feature = "pico2_w")]
use core::cell::RefCell;
#[cfg(feature = "pico2_w")]
use embedded_hal_1::pwm::SetDutyCycle;
#[cfg(feature = "pico2_w")]
use rp235x_hal::pwm::{Slice, SliceId};

#[cfg(feature = "pico2_w")]
extern crate alloc;
#[cfg(feature = "pico2_w")]
use alloc::rc::Rc;

/// PWM channel identifier (A or B)
#[cfg(feature = "pico2_w")]
#[derive(Debug, Clone, Copy)]
pub enum PwmChannel {
    /// Channel A of the PWM slice
    A,
    /// Channel B of the PWM slice
    B,
}

/// RP2350 PWM pin wrapper for motor control
///
/// Wraps an rp235x-hal PWM slice and channel to implement the `PwmPin` trait.
/// Uses Rc<RefCell> for sharing the slice between two channels (IN1 and IN2 of the H-bridge).
#[cfg(feature = "pico2_w")]
pub struct Rp2350PwmPin<S: SliceId> {
    slice: Rc<RefCell<Slice<S, rp235x_hal::pwm::FreeRunning>>>,
    channel: PwmChannel,
}

#[cfg(feature = "pico2_w")]
impl<S: SliceId> PwmPin for Rp2350PwmPin<S> {
    fn set_duty(&mut self, duty: f32) -> Result<(), MotorError> {
        if !(0.0..=1.0).contains(&duty) {
            return Err(MotorError::InvalidSpeed);
        }

        let mut slice = self.slice.borrow_mut();

        // Get the PWM top value to calculate duty cycle
        let top = slice.get_top();
        let compare = (duty * top as f32) as u16;

        // Set duty cycle on the appropriate channel
        match self.channel {
            PwmChannel::A => {
                slice
                    .channel_a
                    .set_duty_cycle(compare)
                    .map_err(|_| MotorError::HardwareFault)?;
            }
            PwmChannel::B => {
                slice
                    .channel_b
                    .set_duty_cycle(compare)
                    .map_err(|_| MotorError::HardwareFault)?;
            }
        }

        Ok(())
    }
}

/// Motor PWM frequency for H-bridge control (500 Hz - matches Freenove reference)
#[cfg(feature = "pico2_w")]
pub const MOTOR_PWM_FREQ_HZ: u32 = 500;

/// Initialize a single H-bridge motor from a PWM slice
///
/// Creates two PWM pin wrappers (IN1 and IN2) from a single PWM slice
/// and returns an HBridgeMotor instance.
///
/// # Arguments
///
/// * `slice` - Configured PWM slice (use `init_motor_pwm_slice()` first)
///
/// # Returns
///
/// H-bridge motor instance ready for use
///
/// # Example
///
/// ```no_run
/// let slice1 = init_motor_pwm_slice(pwm_slices.pwm1.into_mode());
/// let mut motor1 = init_motor(slice1);
/// motor1.set_speed(0.5)?;
/// ```
#[cfg(feature = "pico2_w")]
pub fn init_motor<S: SliceId>(
    slice: Slice<S, rp235x_hal::pwm::FreeRunning>,
) -> HBridgeMotor<Rp2350PwmPin<S>, Rp2350PwmPin<S>> {
    let slice = Rc::new(RefCell::new(slice));

    let in1 = Rp2350PwmPin {
        slice: slice.clone(),
        channel: PwmChannel::A,
    };

    let in2 = Rp2350PwmPin {
        slice,
        channel: PwmChannel::B,
    };

    HBridgeMotor::new(in1, in2)
}

/// Initialize PWM slice for motor control
///
/// Configures a PWM slice with appropriate frequency for motor control.
/// Uses 500 Hz PWM frequency to match Freenove reference implementation.
///
/// # Arguments
///
/// * `slice` - PWM slice to configure
///
/// # Returns
///
/// Configured PWM slice in FreeRunning mode
#[cfg(feature = "pico2_w")]
pub fn init_motor_pwm_slice<S: SliceId>(
    mut slice: Slice<S, rp235x_hal::pwm::FreeRunning>,
) -> Slice<S, rp235x_hal::pwm::FreeRunning> {
    // Calculate divider and top for desired frequency
    // PWM frequency = SYS_CLOCK / (DIV * (TOP + 1))
    // For 500 Hz with good resolution:
    // TOP = 999, DIV = 250 -> freq = 125_000_000 / (250 * 1000) = 500 Hz
    let top: u16 = 999;
    let div_int: u8 = 250;

    slice.set_div_int(div_int);
    slice.set_top(top);
    slice.enable();

    slice
}

/// Helper macro to initialize a motor from PWM slice, GPIO pins, and slice accessor
///
/// This reduces boilerplate when initializing multiple motors.
///
/// # Example
///
/// ```ignore
/// let motor1 = init_motor_from_slice!(pwm_slices.pwm1, pins.gpio18, pins.gpio19);
/// ```
#[cfg(feature = "pico2_w")]
#[macro_export]
macro_rules! init_motor_from_slice {
    ($slice:expr, $pin1:expr, $pin2:expr) => {{
        // Configure GPIO pins for PWM function
        let _ = $pin1.into_function::<rp235x_hal::gpio::FunctionPwm>();
        let _ = $pin2.into_function::<rp235x_hal::gpio::FunctionPwm>();

        // Initialize and configure PWM slice
        let slice = $crate::platform::rp2350::init_motor_pwm_slice($slice.into_mode());

        // Create motor from slice
        $crate::platform::rp2350::init_motor(slice)
    }};
}

// Embassy-RP PWM wrapper for motor control
#[cfg(feature = "pico2_w")]
pub struct EmbassyPwmPin {
    pub output: core::cell::RefCell<embassy_rp::pwm::PwmOutput<'static>>,
}

#[cfg(feature = "pico2_w")]
impl PwmPin for EmbassyPwmPin {
    fn set_duty(&mut self, duty: f32) -> Result<(), MotorError> {
        use embedded_hal_1::pwm::SetDutyCycle;

        if !(0.0..=1.0).contains(&duty) {
            return Err(MotorError::InvalidSpeed);
        }

        let mut output = self.output.borrow_mut();
        let max_duty = output.max_duty_cycle();
        let duty_value = (duty * max_duty as f32) as u16;

        crate::log_info!(
            "PWM set_duty: duty={}, value={}/{}",
            duty,
            duty_value,
            max_duty
        );

        match output.set_duty_cycle(duty_value) {
            Ok(_) => {
                crate::log_info!("PWM set_duty SUCCESS");
            }
            Err(_) => {
                crate::log_error!("PWM set_duty FAILED");
                return Err(MotorError::HardwareFault);
            }
        }

        Ok(())
    }
}

/// Initialize a motor from Embassy-RP PWM instance
///
/// Creates an H-bridge motor from an Embassy-RP PWM instance.
/// The PWM must be configured with both channels (A and B) for IN1 and IN2 control.
///
/// # Arguments
///
/// * `pwm` - Embassy-RP PWM instance (must be 'static lifetime)
///
/// # Returns
///
/// H-bridge motor instance ready for use
///
/// # Example
///
/// ```no_run
/// use embassy_rp::pwm::{Config, Pwm};
/// use pico_trail::platform::rp2350::init_motor_embassy;
///
/// let mut config = Config::default();
/// config.top = 999;  // 500 Hz @ 125 MHz (matches Freenove)
/// config.divider = 250.into();
///
/// let pwm = Pwm::new_output_ab(
///     peripherals.PWM_SLICE1,
///     peripherals.PIN_18,
///     peripherals.PIN_19,
///     config,
/// );
///
/// let mut motor = init_motor_embassy(pwm);
/// motor.set_speed(0.5)?;
/// ```
#[cfg(feature = "pico2_w")]
pub fn init_motor_embassy(
    pwm: embassy_rp::pwm::Pwm<'static>,
) -> HBridgeMotor<EmbassyPwmPin, EmbassyPwmPin> {
    // Split PWM into two independent channels (A and B)
    let (output_a, output_b) = pwm.split();

    let in1 = EmbassyPwmPin {
        output: core::cell::RefCell::new(output_a.expect("PWM channel A should be available")),
    };

    let in2 = EmbassyPwmPin {
        output: core::cell::RefCell::new(output_b.expect("PWM channel B should be available")),
    };

    HBridgeMotor::new(in1, in2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_pwm_frequency_calculation() {
        // Verify our PWM frequency calculation is correct
        const SYS_CLOCK: u32 = 125_000_000;
        const TOP: u32 = 999;
        const DIV: u32 = 250;

        let calculated_freq = SYS_CLOCK / (DIV * (TOP + 1));

        // Should be exactly 500 Hz
        assert_eq!(calculated_freq, 500);
    }
}
