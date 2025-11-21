//! UART interface trait
//!
//! This module defines the UART communication interface that platform implementations must provide.

use crate::platform::Result;

/// UART configuration
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    /// Baud rate in bits per second
    pub baud_rate: u32,
    /// Data bits (typically 8)
    pub data_bits: u8,
    /// Parity mode
    pub parity: UartParity,
    /// Stop bits
    pub stop_bits: UartStopBits,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: 8,
            parity: UartParity::None,
            stop_bits: UartStopBits::One,
        }
    }
}

/// UART parity modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartParity {
    /// No parity
    None,
    /// Even parity
    Even,
    /// Odd parity
    Odd,
}

/// UART stop bits
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartStopBits {
    /// One stop bit
    One,
    /// Two stop bits
    Two,
}

/// UART interface trait
///
/// Platform implementations must provide this interface for UART communication.
///
/// # Safety Invariants
///
/// - UART peripheral must be initialized before use
/// - Only one owner per UART peripheral instance
/// - No concurrent access to the same UART from multiple contexts
pub trait UartInterface {
    /// Write data to UART
    ///
    /// Returns the number of bytes written.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart` if the write operation fails.
    fn write(&mut self, data: &[u8]) -> Result<usize>;

    /// Read data from UART
    ///
    /// Reads up to `buffer.len()` bytes into the provided buffer.
    /// Returns the number of bytes actually read.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart` if the read operation fails.
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;

    /// Set UART baud rate
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart(UartError::InvalidBaudRate)` if the baud rate
    /// cannot be achieved with the current clock configuration.
    fn set_baud_rate(&mut self, baud: u32) -> Result<()>;

    /// Check if data is available to read
    ///
    /// Returns `true` if at least one byte can be read without blocking.
    fn available(&self) -> bool;

    /// Flush transmit buffer
    ///
    /// Blocks until all pending transmit data has been sent.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart` if the flush operation fails.
    fn flush(&mut self) -> Result<()>;
}

/// Async UART interface trait
///
/// Asynchronous version of UartInterface for use with async/await runtimes.
/// Designed for Embassy-RP and other async embedded frameworks.
///
/// # Safety Invariants
///
/// - UART peripheral must be initialized before use
/// - Only one owner per UART peripheral instance
/// - No concurrent access to the same UART from multiple contexts
#[allow(async_fn_in_trait)]
pub trait AsyncUartInterface {
    /// Write data to UART (async)
    ///
    /// Returns the number of bytes written.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart` if the write operation fails.
    async fn write(&mut self, data: &[u8]) -> Result<usize>;

    /// Read data from UART (async)
    ///
    /// Reads up to `buffer.len()` bytes into the provided buffer.
    /// Returns the number of bytes actually read.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart` if the read operation fails.
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;

    /// Flush transmit buffer (async)
    ///
    /// Waits until all pending transmit data has been sent.
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Uart` if the flush operation fails.
    async fn flush(&mut self) -> Result<()>;
}
