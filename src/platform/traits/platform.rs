//! Root platform trait
//!
//! This module defines the root Platform trait that aggregates all peripheral interfaces.

use super::{
    GpioInterface, I2cConfig, I2cInterface, PwmConfig, PwmInterface, SpiConfig, SpiInterface,
    TimerInterface, UartConfig, UartInterface,
};
use crate::platform::Result;

/// Root platform trait
///
/// This trait aggregates all platform-specific peripheral interfaces and provides
/// platform initialization and configuration.
///
/// Platform implementations must provide concrete types for each peripheral interface
/// via associated types, enabling zero-cost abstractions through compile-time dispatch.
///
/// # Example
///
/// ```ignore
/// // Platform implementation
/// pub struct Rp2350Platform {
///     // Platform state
/// }
///
/// impl Platform for Rp2350Platform {
///     type Uart = Rp2350Uart;
///     type I2c = Rp2350I2c;
///     // ... other associated types
///
///     fn init() -> Self {
///         // Initialize clocks, peripherals, etc.
///         Self { /* ... */ }
///     }
///
///     fn create_uart(&mut self, config: UartConfig) -> Result<Self::Uart> {
///         // Create and configure UART instance
///     }
///
///     // ... other methods
/// }
/// ```
pub trait Platform: Sized {
    /// UART peripheral type
    type Uart: UartInterface;

    /// I2C peripheral type
    type I2c: I2cInterface;

    /// SPI peripheral type
    type Spi: SpiInterface;

    /// PWM peripheral type
    type Pwm: PwmInterface;

    /// GPIO peripheral type
    type Gpio: GpioInterface;

    /// Timer peripheral type
    type Timer: TimerInterface;

    /// Initialize the platform
    ///
    /// This method performs platform-specific initialization including:
    /// - Clock configuration
    /// - Peripheral initialization
    /// - System setup
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::InitializationFailed` if initialization fails.
    fn init() -> Result<Self>;

    /// Get system clock frequency in Hz
    ///
    /// Returns the main system clock frequency used for peripheral timing calculations.
    fn system_clock_hz(&self) -> u32;

    /// Create a UART peripheral instance
    ///
    /// # Arguments
    ///
    /// * `uart_id` - Platform-specific UART identifier (e.g., 0 for UART0, 1 for UART1)
    /// * `config` - UART configuration
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::ResourceUnavailable` if the UART is already in use
    /// or the UART ID is invalid.
    fn create_uart(&mut self, uart_id: u8, config: UartConfig) -> Result<Self::Uart>;

    /// Create an I2C peripheral instance
    ///
    /// # Arguments
    ///
    /// * `i2c_id` - Platform-specific I2C identifier (e.g., 0 for I2C0, 1 for I2C1)
    /// * `config` - I2C configuration
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::ResourceUnavailable` if the I2C bus is already in use
    /// or the I2C ID is invalid.
    fn create_i2c(&mut self, i2c_id: u8, config: I2cConfig) -> Result<Self::I2c>;

    /// Create an SPI peripheral instance
    ///
    /// # Arguments
    ///
    /// * `spi_id` - Platform-specific SPI identifier (e.g., 0 for SPI0, 1 for SPI1)
    /// * `config` - SPI configuration
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::ResourceUnavailable` if the SPI bus is already in use
    /// or the SPI ID is invalid.
    fn create_spi(&mut self, spi_id: u8, config: SpiConfig) -> Result<Self::Spi>;

    /// Create a PWM peripheral instance
    ///
    /// # Arguments
    ///
    /// * `pin` - GPIO pin number for PWM output
    /// * `config` - PWM configuration
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::ResourceUnavailable` if the pin does not support PWM,
    /// is already in use, or the pin number is invalid.
    fn create_pwm(&mut self, pin: u8, config: PwmConfig) -> Result<Self::Pwm>;

    /// Create a GPIO peripheral instance
    ///
    /// # Arguments
    ///
    /// * `pin` - GPIO pin number
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::ResourceUnavailable` if the pin is already in use
    /// or the pin number is invalid.
    fn create_gpio(&mut self, pin: u8) -> Result<Self::Gpio>;

    /// Get timer instance
    ///
    /// Returns a reference to the platform timer for delays and timing operations.
    fn timer(&self) -> &Self::Timer;

    /// Get mutable timer instance
    fn timer_mut(&mut self) -> &mut Self::Timer;

    /// Read battery voltage from ADC
    ///
    /// Returns raw ADC value (0-4095 for 12-bit ADC on RP2350).
    /// This method reads from the battery voltage sensing pin (typically GPIO 26/ADC0)
    /// with voltage divider circuit.
    ///
    /// # Returns
    ///
    /// Raw 12-bit ADC value (0-4095) representing voltage at ADC pin (0-3.3V range)
    ///
    /// # Implementation Notes
    ///
    /// - RP2350: Reads GPIO 26 (ADC0) via embassy-rp Adc driver with 5-sample averaging
    /// - MockPlatform: Returns configurable test value
    fn read_battery_adc(&mut self) -> u16;
}
