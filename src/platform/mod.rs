//! Platform abstraction layer
//!
//! This module provides hardware abstraction for different microcontroller platforms.
//! All platform-specific code must be isolated to this module per NFR-nmmu0.

pub mod error;
pub mod traits;

// Re-export commonly used types
pub use error::{PlatformError, Result};
pub use traits::{
    GpioInterface, I2cInterface, Platform, PwmInterface, SpiInterface, TimerInterface,
    UartInterface,
};
