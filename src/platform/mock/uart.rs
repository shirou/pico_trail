//! Mock UART implementation for testing

use crate::platform::{
    Result,
    traits::{UartConfig, UartInterface},
};
use core::cell::RefCell;
use std::vec::Vec;

/// Mock UART implementation
///
/// Provides in-memory buffers for transmit and receive data,
/// allowing unit tests to verify UART operations without hardware.
///
/// # Example
///
/// ```
/// use pico_trail::platform::mock::MockUart;
/// use pico_trail::platform::traits::UartInterface;
///
/// let mut uart = MockUart::new(Default::default());
///
/// // Write data
/// uart.write(b"Hello").unwrap();
///
/// // Verify transmitted data
/// assert_eq!(uart.tx_buffer(), b"Hello");
///
/// // Inject received data for testing
/// uart.inject_rx_data(b"World");
/// let mut buf = [0u8; 5];
/// uart.read(&mut buf).unwrap();
/// assert_eq!(&buf, b"World");
/// ```
#[derive(Debug)]
pub struct MockUart {
    config: UartConfig,
    tx_buffer: RefCell<Vec<u8>>,
    rx_buffer: RefCell<Vec<u8>>,
}

impl MockUart {
    /// Create a new mock UART
    pub fn new(config: UartConfig) -> Self {
        Self {
            config,
            tx_buffer: RefCell::new(Vec::new()),
            rx_buffer: RefCell::new(Vec::new()),
        }
    }

    /// Get transmitted data (for test verification)
    pub fn tx_buffer(&self) -> Vec<u8> {
        self.tx_buffer.borrow().clone()
    }

    /// Clear transmit buffer
    pub fn clear_tx_buffer(&mut self) {
        self.tx_buffer.borrow_mut().clear();
    }

    /// Inject receive data (for test setup)
    pub fn inject_rx_data(&mut self, data: &[u8]) {
        self.rx_buffer.borrow_mut().extend_from_slice(data);
    }

    /// Get current baud rate
    pub fn baud_rate(&self) -> u32 {
        self.config.baud_rate
    }
}

impl UartInterface for MockUart {
    fn write(&mut self, data: &[u8]) -> Result<usize> {
        self.tx_buffer.borrow_mut().extend_from_slice(data);
        Ok(data.len())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let mut rx = self.rx_buffer.borrow_mut();
        let to_read = core::cmp::min(buffer.len(), rx.len());

        buffer[..to_read].copy_from_slice(&rx[..to_read]);
        rx.drain(..to_read);

        Ok(to_read)
    }

    fn set_baud_rate(&mut self, baud: u32) -> Result<()> {
        self.config.baud_rate = baud;
        Ok(())
    }

    fn available(&self) -> bool {
        !self.rx_buffer.borrow().is_empty()
    }

    fn flush(&mut self) -> Result<()> {
        // Mock implementation - nothing to flush
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_uart_write() {
        let mut uart = MockUart::new(UartConfig::default());
        let written = uart.write(b"Hello, World!").unwrap();
        assert_eq!(written, 13);
        assert_eq!(uart.tx_buffer(), b"Hello, World!");
    }

    #[test]
    fn test_mock_uart_read() {
        let mut uart = MockUart::new(UartConfig::default());
        uart.inject_rx_data(b"Test Data");

        let mut buffer = [0u8; 4];
        let read = uart.read(&mut buffer).unwrap();
        assert_eq!(read, 4);
        assert_eq!(&buffer, b"Test");

        // Read remaining data
        let mut buffer2 = [0u8; 10];
        let read2 = uart.read(&mut buffer2).unwrap();
        assert_eq!(read2, 5);
        assert_eq!(&buffer2[..5], b" Data");
    }

    #[test]
    fn test_mock_uart_available() {
        let mut uart = MockUart::new(UartConfig::default());
        assert!(!uart.available());

        uart.inject_rx_data(b"X");
        assert!(uart.available());

        let mut buf = [0u8; 1];
        uart.read(&mut buf).unwrap();
        assert!(!uart.available());
    }

    #[test]
    fn test_mock_uart_baud_rate() {
        let mut uart = MockUart::new(UartConfig::default());
        assert_eq!(uart.baud_rate(), 115200);

        uart.set_baud_rate(9600).unwrap();
        assert_eq!(uart.baud_rate(), 9600);
    }
}
