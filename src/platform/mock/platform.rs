//! Mock Platform implementation for testing

use crate::platform::{
    error::PlatformError,
    traits::{I2cConfig, Platform, PwmConfig, SpiConfig, UartConfig},
    Result,
};

use super::{MockGpio, MockI2c, MockPwm, MockSpi, MockTimer, MockUart};
use std::vec::Vec;

/// Mock Platform implementation
///
/// Provides mock peripheral implementations for hardware-free testing.
///
/// # Example
///
/// ```
/// use pico_trail::platform::mock::MockPlatform;
/// use pico_trail::platform::traits::{Platform, UartInterface};
///
/// let mut platform = MockPlatform::new();
/// let mut uart = platform.create_uart(0, Default::default()).unwrap();
/// uart.write(b"Hello").unwrap();
/// ```
#[derive(Debug)]
pub struct MockPlatform {
    timer: MockTimer,
    uart_count: u8,
    i2c_count: u8,
    spi_count: u8,
    gpio_allocated: Vec<u8>,
}

impl MockPlatform {
    /// Create a new mock platform
    pub fn new() -> Self {
        Self {
            timer: MockTimer::new(),
            uart_count: 0,
            i2c_count: 0,
            spi_count: 0,
            gpio_allocated: Vec::new(),
        }
    }

    /// Maximum number of UART peripherals
    pub const MAX_UARTS: u8 = 2;

    /// Maximum number of I2C peripherals
    pub const MAX_I2CS: u8 = 2;

    /// Maximum number of SPI peripherals
    pub const MAX_SPIS: u8 = 2;

    /// Maximum GPIO pin number
    pub const MAX_GPIO: u8 = 29;
}

impl Default for MockPlatform {
    fn default() -> Self {
        Self::new()
    }
}

impl Platform for MockPlatform {
    type Uart = MockUart;
    type I2c = MockI2c;
    type Spi = MockSpi;
    type Pwm = MockPwm;
    type Gpio = MockGpio;
    type Timer = MockTimer;

    fn init() -> Result<Self> {
        Ok(Self::new())
    }

    fn system_clock_hz(&self) -> u32 {
        125_000_000 // Simulated 125 MHz system clock
    }

    fn create_uart(&mut self, uart_id: u8, config: UartConfig) -> Result<Self::Uart> {
        if uart_id >= Self::MAX_UARTS {
            return Err(PlatformError::ResourceUnavailable);
        }
        self.uart_count += 1;
        Ok(MockUart::new(config))
    }

    fn create_i2c(&mut self, i2c_id: u8, config: I2cConfig) -> Result<Self::I2c> {
        if i2c_id >= Self::MAX_I2CS {
            return Err(PlatformError::ResourceUnavailable);
        }
        self.i2c_count += 1;
        Ok(MockI2c::new(config))
    }

    fn create_spi(&mut self, spi_id: u8, config: SpiConfig) -> Result<Self::Spi> {
        if spi_id >= Self::MAX_SPIS {
            return Err(PlatformError::ResourceUnavailable);
        }
        self.spi_count += 1;
        Ok(MockSpi::new(config))
    }

    fn create_pwm(&mut self, pin: u8, config: PwmConfig) -> Result<Self::Pwm> {
        if pin > Self::MAX_GPIO {
            return Err(PlatformError::ResourceUnavailable);
        }
        Ok(MockPwm::new(config))
    }

    fn create_gpio(&mut self, pin: u8) -> Result<Self::Gpio> {
        if pin > Self::MAX_GPIO {
            return Err(PlatformError::ResourceUnavailable);
        }
        if self.gpio_allocated.contains(&pin) {
            return Err(PlatformError::ResourceUnavailable);
        }
        self.gpio_allocated.push(pin);
        Ok(MockGpio::new_output())
    }

    fn timer(&self) -> &Self::Timer {
        &self.timer
    }

    fn timer_mut(&mut self) -> &mut Self::Timer {
        &mut self.timer
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::traits::{GpioInterface, TimerInterface, UartInterface};

    #[test]
    fn test_mock_platform_init() {
        let platform = MockPlatform::init().unwrap();
        assert_eq!(platform.system_clock_hz(), 125_000_000);
    }

    #[test]
    fn test_mock_platform_uart() {
        let mut platform = MockPlatform::new();
        let mut uart0 = platform.create_uart(0, UartConfig::default()).unwrap();
        uart0.write(b"test").unwrap();

        // Creating another UART should work
        let _uart1 = platform.create_uart(1, UartConfig::default()).unwrap();

        // Invalid UART ID should fail
        assert!(platform.create_uart(10, UartConfig::default()).is_err());
    }

    #[test]
    fn test_mock_platform_gpio() {
        let mut platform = MockPlatform::new();
        let mut gpio0 = platform.create_gpio(0).unwrap();
        gpio0.set_high().unwrap();

        // Same GPIO should not be allocatable twice
        assert!(platform.create_gpio(0).is_err());

        // Different GPIO should work
        let _gpio1 = platform.create_gpio(1).unwrap();

        // Invalid GPIO should fail
        assert!(platform.create_gpio(100).is_err());
    }

    #[test]
    fn test_mock_platform_timer() {
        let mut platform = MockPlatform::new();
        platform.timer_mut().delay_us(1000).unwrap();
        assert_eq!(platform.timer().now_us(), 1000);
    }
}
