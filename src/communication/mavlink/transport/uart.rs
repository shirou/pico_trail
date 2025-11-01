//! UART Transport Implementation
//!
//! Provides MAVLink transport over UART (serial) interface.
//!
//! # Overview
//!
//! UART is the primary transport for MAVLink communication, used for:
//! - Ground Control Station connections (via USB/FTDI)
//! - Telemetry radio links (e.g., SiK Radio, RFD900)
//! - Debug console output
//! - Firmware upload and parameter configuration
//!
//! # Hardware
//!
//! On Raspberry Pi Pico 2 W:
//! - UART0: GPIO 0 (TX), GPIO 1 (RX) - Default MAVLink port
//! - UART1: GPIO 4 (TX), GPIO 5 (RX) - Available for secondary devices
//!
//! # Configuration
//!
//! Standard MAVLink UART settings:
//! - Baud rate: 115200 (default), 57600 (telemetry radios), 921600 (high-speed)
//! - Data bits: 8
//! - Parity: None
//! - Stop bits: 1
//! - Flow control: None (MAVLink handles flow control in protocol)
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::communication::mavlink::transport::{MavlinkTransport, uart::UartTransport};
//!
//! // Create UART transport from Embassy UART peripheral
//! let mut transport = UartTransport::new(uart_rx, uart_tx);
//!
//! // Use as MavlinkTransport
//! let mut buf = [0u8; 256];
//! let n = transport.read(&mut buf).await?;
//! ```

use super::{MavlinkTransport, TransportError};
use embedded_io_async::{Read, Write};

/// UART transport for MAVLink communication
///
/// Wraps Embassy UART peripheral types and implements the `MavlinkTransport` trait.
///
/// # Type Parameters
///
/// - `R`: UART receive type implementing `embedded_io_async::Read`
/// - `W`: UART transmit type implementing `embedded_io_async::Write`
///
/// # Design
///
/// This implementation is generic over UART types to support:
/// - Different platforms (RP2040, RP2350, STM32, etc.)
/// - Buffered vs unbuffered UART
/// - DMA vs interrupt-driven I/O
///
/// # Example Types
///
/// On RP2350 with Embassy:
/// - `R = embassy_rp::uart::BufferedUartRx<'static>`
/// - `W = embassy_rp::uart::BufferedUartTx<'static>`
pub struct UartTransport<R, W> {
    /// UART receiver
    rx: R,
    /// UART transmitter
    tx: W,
}

impl<R, W> UartTransport<R, W> {
    /// Create a new UART transport
    ///
    /// # Arguments
    ///
    /// * `rx` - UART receiver implementing `embedded_io_async::Read`
    /// * `tx` - UART transmitter implementing `embedded_io_async::Write`
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use embassy_rp::uart::{Config, BufferedUart};
    ///
    /// let uart = BufferedUart::new(
    ///     uart0,
    ///     irq,
    ///     tx_pin,
    ///     rx_pin,
    ///     tx_buffer,
    ///     rx_buffer,
    ///     config,
    /// );
    /// let (rx, tx) = uart.split();
    ///
    /// let transport = UartTransport::new(rx, tx);
    /// ```
    pub fn new(rx: R, tx: W) -> Self {
        Self { rx, tx }
    }

    /// Split the transport back into receiver and transmitter
    ///
    /// Useful if you need direct access to UART peripherals.
    ///
    /// # Returns
    ///
    /// Tuple of (receiver, transmitter)
    pub fn split(self) -> (R, W) {
        (self.rx, self.tx)
    }
}

impl<R, W> MavlinkTransport for UartTransport<R, W>
where
    R: Read,
    W: Write,
{
    async fn available(&self) -> usize {
        // Embassy UART doesn't expose available bytes count
        // Return 0 to indicate "unknown" - caller will use blocking read
        0
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        // Read at least 1 byte, up to buf.len() bytes
        // Embassy's embedded_io_async::Read blocks until data is available
        self.rx.read(buf).await.map_err(|_| TransportError::IoError)
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        // Write all bytes from buffer
        // Embassy's embedded_io_async::Write::write_all ensures all bytes are written
        self.tx
            .write_all(buf)
            .await
            .map_err(|_| TransportError::IoError)?;

        // Return number of bytes written
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), TransportError> {
        // Flush UART transmit buffer
        // Ensures all buffered data is transmitted before returning
        self.tx.flush().await.map_err(|_| TransportError::IoError)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock UART receiver for testing
    struct MockUartRx {
        data: &'static [u8],
        pos: usize,
    }

    impl MockUartRx {
        fn new(data: &'static [u8]) -> Self {
            Self { data, pos: 0 }
        }
    }

    impl embedded_io_async::ErrorType for MockUartRx {
        type Error = core::convert::Infallible;
    }

    impl Read for MockUartRx {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.pos >= self.data.len() {
                // No more data - return Ok(0) to simulate blocking read
                return Ok(0);
            }

            let remaining = &self.data[self.pos..];
            let to_read = core::cmp::min(buf.len(), remaining.len());
            buf[..to_read].copy_from_slice(&remaining[..to_read]);
            self.pos += to_read;
            Ok(to_read)
        }
    }

    /// Mock UART transmitter for testing
    struct MockUartTx {
        buffer: heapless::Vec<u8, 256>,
    }

    impl MockUartTx {
        fn new() -> Self {
            Self {
                buffer: heapless::Vec::new(),
            }
        }

        fn written(&self) -> &[u8] {
            &self.buffer
        }
    }

    impl embedded_io_async::ErrorType for MockUartTx {
        type Error = core::convert::Infallible;
    }

    impl Write for MockUartTx {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, core::convert::Infallible> {
            self.buffer.extend_from_slice(buf).unwrap();
            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), core::convert::Infallible> {
            Ok(())
        }
    }

    #[tokio::test]
    async fn test_uart_transport_creation() {
        let rx = MockUartRx::new(b"");
        let tx = MockUartTx::new();
        let _transport = UartTransport::new(rx, tx);
    }

    #[tokio::test]
    async fn test_uart_transport_read() {
        let rx = MockUartRx::new(b"Hello MAVLink");
        let tx = MockUartTx::new();
        let mut transport = UartTransport::new(rx, tx);

        let mut buf = [0u8; 32];
        let n = transport.read(&mut buf).await.unwrap();
        assert_eq!(n, 13);
        assert_eq!(&buf[..n], b"Hello MAVLink");
    }

    #[tokio::test]
    async fn test_uart_transport_write() {
        let rx = MockUartRx::new(b"");
        let tx = MockUartTx::new();
        let mut transport = UartTransport::new(rx, tx);

        let data = b"Test message";
        let n = transport.write(data).await.unwrap();
        assert_eq!(n, data.len());

        let (_, tx) = transport.split();
        assert_eq!(tx.written(), data);
    }

    #[tokio::test]
    async fn test_uart_transport_available() {
        let rx = MockUartRx::new(b"data");
        let tx = MockUartTx::new();
        let transport = UartTransport::new(rx, tx);

        // Should return 0 (unknown/unsupported)
        assert_eq!(transport.available().await, 0);
    }

    #[tokio::test]
    async fn test_uart_transport_flush() {
        let rx = MockUartRx::new(b"");
        let tx = MockUartTx::new();
        let mut transport = UartTransport::new(rx, tx);

        // Should not error
        transport.flush().await.unwrap();
    }

    #[tokio::test]
    async fn test_uart_transport_partial_read() {
        let rx = MockUartRx::new(b"1234567890");
        let tx = MockUartTx::new();
        let mut transport = UartTransport::new(rx, tx);

        // Read fewer bytes than available
        let mut buf = [0u8; 5];
        let n = transport.read(&mut buf).await.unwrap();
        assert_eq!(n, 5);
        assert_eq!(&buf, b"12345");

        // Read remaining bytes
        let mut buf = [0u8; 10];
        let n = transport.read(&mut buf).await.unwrap();
        assert_eq!(n, 5);
        assert_eq!(&buf[..n], b"67890");
    }
}
