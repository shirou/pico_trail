//! Platform abstraction traits
//!
//! This module defines the traits that platform implementations must provide.

pub mod board;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod platform;
pub mod pwm;
pub mod spi;
pub mod timer;
pub mod uart;

// Re-export trait interfaces
pub use board::{
    BoardPinConfig, MotorPins, OutputMode, PinConfig, PinError, PinType, PullMode, Speed,
    BOARD_CONFIG,
};
pub use flash::FlashInterface;
pub use gpio::{GpioInterface, GpioMode};
pub use i2c::{I2cConfig, I2cInterface};
pub use platform::Platform;
pub use pwm::{PwmConfig, PwmInterface};
pub use spi::{SpiConfig, SpiInterface, SpiMode};
pub use timer::TimerInterface;
pub use uart::{AsyncUartInterface, UartConfig, UartInterface, UartParity, UartStopBits};
