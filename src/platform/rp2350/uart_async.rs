//! Async UART wrapper for Embassy-RP
//!
//! This module provides a wrapper around embassy-rp's UART that implements
//! the AsyncUartInterface trait for use with async GPS drivers.

use crate::platform::{
    error::{PlatformError, UartError},
    traits::{AsyncUartInterface, UartConfig},
    Result,
};

/// Embassy-RP async UART wrapper
///
/// Wraps `embassy_rp::uart::Uart` to implement the `AsyncUartInterface` trait.
pub struct Rp2350AsyncUart<'a, T: embassy_rp::uart::Instance, M: embassy_rp::uart::Mode> {
    uart: embassy_rp::uart::Uart<'a, T, M>,
}

impl<'a, T: embassy_rp::uart::Instance, M: embassy_rp::uart::Mode> Rp2350AsyncUart<'a, T, M> {
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

impl<'a, T: embassy_rp::uart::Instance, M: embassy_rp::uart::Mode> AsyncUartInterface
    for Rp2350AsyncUart<'a, T, M>
{
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        use embedded_io_async::Read;

        self.uart
            .read(buffer)
            .await
            .map_err(|_| PlatformError::Uart(UartError::ReadFailed))
    }

    async fn write(&mut self, data: &[u8]) -> Result<usize> {
        use embedded_io_async::Write;

        self.uart
            .write(data)
            .await
            .map_err(|_| PlatformError::Uart(UartError::WriteFailed))
    }

    async fn flush(&mut self) -> Result<()> {
        use embedded_io_async::Write;

        self.uart
            .flush()
            .await
            .map_err(|_| PlatformError::Uart(UartError::WriteFailed))
    }
}
