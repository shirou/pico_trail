//! Embassy-RP UART wrapper for UartInterface trait
//!
//! This module provides a wrapper around embassy-rp's UART that implements
//! the UartInterface trait.

use crate::platform::{
    error::{PlatformError, UartError},
    traits::{UartConfig, UartInterface},
    Result,
};

/// Embassy-RP UART wrapper
///
/// Wraps `embassy_rp::uart::Uart` to implement the `UartInterface` trait.
pub struct EmbassyUart<'a, T: embassy_rp::uart::Instance, M: embassy_rp::uart::Mode> {
    uart: embassy_rp::uart::Uart<'a, T, M>,
}

impl<'a, T: embassy_rp::uart::Instance, M: embassy_rp::uart::Mode> EmbassyUart<'a, T, M> {
    /// Create a new Embassy UART wrapper
    ///
    /// # Arguments
    ///
    /// * `uart` - The embassy-rp UART instance
    /// * `_config` - UART configuration (ignored, as UART is already configured)
    pub fn new(uart: embassy_rp::uart::Uart<'a, T, M>, _config: UartConfig) -> Self {
        Self { uart }
    }
}

impl<'a, T: embassy_rp::uart::Instance, M: embassy_rp::uart::Mode> UartInterface
    for EmbassyUart<'a, T, M>
{
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        // Embassy UART is async, so we can't implement sync read
        // Return error for now - GPS driver will need to be adapted for async
        Err(PlatformError::Uart(UartError::ReadFailed))
    }

    fn write(&mut self, _data: &[u8]) -> Result<usize> {
        // Embassy UART is async, so we can't implement sync write
        Err(PlatformError::Uart(UartError::WriteFailed))
    }

    fn flush(&mut self) -> Result<()> {
        Ok(())
    }
}
