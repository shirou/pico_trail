//! Mock GPIO implementation for testing

use crate::platform::{
    Result,
    error::{GpioError, PlatformError},
    traits::{GpioInterface, GpioMode},
};

/// Mock GPIO implementation
///
/// Tracks pin state (high/low) and mode for test verification.
#[derive(Debug)]
pub struct MockGpio {
    state: bool,
    mode: GpioMode,
}

impl MockGpio {
    /// Create a new mock GPIO in output mode
    pub fn new_output() -> Self {
        Self {
            state: false,
            mode: GpioMode::OutputPushPull,
        }
    }

    /// Create a new mock GPIO in input mode
    pub fn new_input() -> Self {
        Self {
            state: false,
            mode: GpioMode::Input,
        }
    }

    /// Set the input state (for simulating input pin reads)
    pub fn set_input_state(&mut self, high: bool) {
        self.state = high;
    }
}

impl GpioInterface for MockGpio {
    fn set_high(&mut self) -> Result<()> {
        match self.mode {
            GpioMode::OutputPushPull | GpioMode::OutputOpenDrain => {
                self.state = true;
                Ok(())
            }
            _ => Err(PlatformError::Gpio(GpioError::InvalidMode)),
        }
    }

    fn set_low(&mut self) -> Result<()> {
        match self.mode {
            GpioMode::OutputPushPull | GpioMode::OutputOpenDrain => {
                self.state = false;
                Ok(())
            }
            _ => Err(PlatformError::Gpio(GpioError::InvalidMode)),
        }
    }

    fn toggle(&mut self) -> Result<()> {
        match self.mode {
            GpioMode::OutputPushPull | GpioMode::OutputOpenDrain => {
                self.state = !self.state;
                Ok(())
            }
            _ => Err(PlatformError::Gpio(GpioError::InvalidMode)),
        }
    }

    fn read(&self) -> bool {
        self.state
    }

    fn set_mode(&mut self, mode: GpioMode) -> Result<()> {
        self.mode = mode;
        Ok(())
    }

    fn mode(&self) -> GpioMode {
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_gpio_output() {
        let mut gpio = MockGpio::new_output();
        assert!(!gpio.read());

        gpio.set_high().unwrap();
        assert!(gpio.read());

        gpio.set_low().unwrap();
        assert!(!gpio.read());
    }

    #[test]
    fn test_mock_gpio_toggle() {
        let mut gpio = MockGpio::new_output();
        assert!(!gpio.read());

        gpio.toggle().unwrap();
        assert!(gpio.read());

        gpio.toggle().unwrap();
        assert!(!gpio.read());
    }

    #[test]
    fn test_mock_gpio_input() {
        let mut gpio = MockGpio::new_input();
        assert!(!gpio.read());

        // Simulate external signal
        gpio.set_input_state(true);
        assert!(gpio.read());

        // Input mode should not allow set_high/set_low
        assert!(gpio.set_high().is_err());
        assert!(gpio.set_low().is_err());
        assert!(gpio.toggle().is_err());
    }

    #[test]
    fn test_mock_gpio_mode() {
        let mut gpio = MockGpio::new_output();
        assert_eq!(gpio.mode(), GpioMode::OutputPushPull);

        gpio.set_mode(GpioMode::Input).unwrap();
        assert_eq!(gpio.mode(), GpioMode::Input);
    }
}
