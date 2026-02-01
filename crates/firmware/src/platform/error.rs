//! Platform error types
//!
//! This module defines error types for platform operations.

use core::fmt;

/// Result type for platform operations
pub type Result<T> = core::result::Result<T, PlatformError>;

/// Platform-level errors
///
/// All platform implementations map their HAL-specific errors to these variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformError {
    /// UART operation failed
    Uart(UartError),
    /// I2C operation failed
    I2c(I2cError),
    /// SPI operation failed
    Spi(SpiError),
    /// PWM operation failed
    Pwm(PwmError),
    /// GPIO operation failed
    Gpio(GpioError),
    /// Timer operation failed
    Timer(TimerError),
    /// Flash operation failed
    Flash(FlashError),
    /// Platform initialization failed
    InitializationFailed,
    /// Invalid configuration provided
    InvalidConfig,
    /// Resource not available
    ResourceUnavailable,
}

/// UART-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// Write operation failed
    WriteFailed,
    /// Read operation failed
    ReadFailed,
    /// Timeout occurred
    Timeout,
    /// Invalid baud rate
    InvalidBaudRate,
    /// Framing error
    FramingError,
    /// Parity error
    ParityError,
    /// Overrun error
    Overrun,
}

/// I2C-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// Bus error occurred
    BusError,
    /// No acknowledgment received
    Nack,
    /// Arbitration lost
    ArbitrationLost,
    /// Timeout occurred
    Timeout,
    /// Invalid address
    InvalidAddress,
}

/// SPI-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    /// Transfer failed
    TransferFailed,
    /// Timeout occurred
    Timeout,
    /// Mode fault
    ModeFault,
    /// Overrun error
    Overrun,
}

/// PWM-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    /// Invalid duty cycle value
    InvalidDutyCycle,
    /// Invalid frequency
    InvalidFrequency,
    /// Channel not available
    ChannelUnavailable,
}

/// GPIO-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioError {
    /// Invalid pin number
    InvalidPin,
    /// Invalid mode for operation
    InvalidMode,
    /// Pin already in use
    PinInUse,
    /// Hardware error during GPIO operation
    HardwareError,
}

/// Timer-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerError {
    /// Timer overflow
    Overflow,
    /// Invalid duration
    InvalidDuration,
}

/// Flash-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    /// Erase operation failed
    EraseFailed,
    /// Write operation failed
    WriteFailed,
    /// Read operation failed
    ReadFailed,
    /// Invalid address (out of bounds)
    InvalidAddress,
    /// Verify failed (data mismatch after write)
    VerifyFailed,
    /// Flash is busy
    Busy,
}

impl fmt::Display for PlatformError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PlatformError::Uart(e) => write!(f, "UART error: {:?}", e),
            PlatformError::I2c(e) => write!(f, "I2C error: {:?}", e),
            PlatformError::Spi(e) => write!(f, "SPI error: {:?}", e),
            PlatformError::Pwm(e) => write!(f, "PWM error: {:?}", e),
            PlatformError::Gpio(e) => write!(f, "GPIO error: {:?}", e),
            PlatformError::Timer(e) => write!(f, "Timer error: {:?}", e),
            PlatformError::Flash(e) => write!(f, "Flash error: {:?}", e),
            PlatformError::InitializationFailed => write!(f, "Platform initialization failed"),
            PlatformError::InvalidConfig => write!(f, "Invalid configuration"),
            PlatformError::ResourceUnavailable => write!(f, "Resource not available"),
        }
    }
}

// From implementations for error conversion
impl From<FlashError> for PlatformError {
    fn from(error: FlashError) -> Self {
        PlatformError::Flash(error)
    }
}

impl From<pico_trail_core::parameters::ParameterError> for PlatformError {
    fn from(e: pico_trail_core::parameters::ParameterError) -> Self {
        match e {
            pico_trail_core::parameters::ParameterError::InvalidConfig => {
                PlatformError::InvalidConfig
            }
            pico_trail_core::parameters::ParameterError::StoreFull => PlatformError::InvalidConfig,
            pico_trail_core::parameters::ParameterError::ReadOnly => PlatformError::InvalidConfig,
        }
    }
}
