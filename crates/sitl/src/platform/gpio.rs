//! Simulated GPIO peripheral for SITL.
//!
//! Tracks pin state (high/low) and direction (input/output) for
//! simulated digital I/O.

/// GPIO pin direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioDirection {
    Input,
    Output,
}

/// Simulated GPIO pin with state tracking.
#[derive(Debug)]
pub struct SitlGpio {
    pin: u8,
    state: bool,
    direction: GpioDirection,
}

impl SitlGpio {
    /// Create a new GPIO pin configured as output (default low).
    pub fn new_output(pin: u8) -> Self {
        Self {
            pin,
            state: false,
            direction: GpioDirection::Output,
        }
    }

    /// Create a new GPIO pin configured as input.
    pub fn new_input(pin: u8) -> Self {
        Self {
            pin,
            state: false,
            direction: GpioDirection::Input,
        }
    }

    /// Set pin high.
    pub fn set_high(&mut self) {
        self.state = true;
    }

    /// Set pin low.
    pub fn set_low(&mut self) {
        self.state = false;
    }

    /// Toggle pin state.
    pub fn toggle(&mut self) {
        self.state = !self.state;
    }

    /// Read pin state.
    pub fn read(&self) -> bool {
        self.state
    }

    /// Set pin direction.
    pub fn set_direction(&mut self, direction: GpioDirection) {
        self.direction = direction;
    }

    /// Get pin direction.
    pub fn direction(&self) -> GpioDirection {
        self.direction
    }

    /// Get the pin number.
    pub fn pin(&self) -> u8 {
        self.pin
    }

    /// Inject a state value (for simulating external input).
    pub fn inject_state(&mut self, state: bool) {
        self.state = state;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_pin() {
        let mut gpio = SitlGpio::new_output(5);
        assert_eq!(gpio.pin(), 5);
        assert_eq!(gpio.direction(), GpioDirection::Output);
        assert!(!gpio.read());

        gpio.set_high();
        assert!(gpio.read());

        gpio.set_low();
        assert!(!gpio.read());
    }

    #[test]
    fn test_toggle() {
        let mut gpio = SitlGpio::new_output(0);
        assert!(!gpio.read());
        gpio.toggle();
        assert!(gpio.read());
        gpio.toggle();
        assert!(!gpio.read());
    }

    #[test]
    fn test_input_pin() {
        let mut gpio = SitlGpio::new_input(3);
        assert_eq!(gpio.direction(), GpioDirection::Input);
        assert!(!gpio.read());

        gpio.inject_state(true);
        assert!(gpio.read());
    }

    #[test]
    fn test_direction_change() {
        let mut gpio = SitlGpio::new_output(0);
        assert_eq!(gpio.direction(), GpioDirection::Output);

        gpio.set_direction(GpioDirection::Input);
        assert_eq!(gpio.direction(), GpioDirection::Input);
    }
}
