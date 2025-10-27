//! GPIO interface trait
//!
//! This module defines the GPIO (General Purpose Input/Output) interface that platform implementations must provide.

use crate::platform::Result;

/// GPIO pin mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioMode {
    /// Input mode (high impedance)
    Input,
    /// Input mode with pull-up resistor
    InputPullUp,
    /// Input mode with pull-down resistor
    InputPullDown,
    /// Output mode (push-pull)
    OutputPushPull,
    /// Output mode (open-drain)
    OutputOpenDrain,
}

/// GPIO interface trait
///
/// Platform implementations must provide this interface for GPIO control.
///
/// # Safety Invariants
///
/// - GPIO pin must be initialized before use
/// - Only one owner per GPIO pin instance
/// - No concurrent access to the same GPIO pin from multiple contexts
/// - Pin number must be valid for the platform
pub trait GpioInterface {
    /// Set GPIO pin high (logic level 1)
    ///
    /// Only valid in output modes.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Gpio(GpioError::InvalidMode)` if the pin
    /// is not configured as an output.
    fn set_high(&mut self) -> Result<()>;

    /// Set GPIO pin low (logic level 0)
    ///
    /// Only valid in output modes.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Gpio(GpioError::InvalidMode)` if the pin
    /// is not configured as an output.
    fn set_low(&mut self) -> Result<()>;

    /// Toggle GPIO pin state
    ///
    /// Only valid in output modes.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Gpio(GpioError::InvalidMode)` if the pin
    /// is not configured as an output.
    fn toggle(&mut self) -> Result<()>;

    /// Read GPIO pin state
    ///
    /// Returns `true` if the pin is high, `false` if low.
    ///
    /// Valid in both input and output modes.
    fn read(&self) -> bool;

    /// Set GPIO pin mode
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Gpio` if the mode cannot be set.
    fn set_mode(&mut self, mode: GpioMode) -> Result<()>;

    /// Get current GPIO pin mode
    fn mode(&self) -> GpioMode;
}
