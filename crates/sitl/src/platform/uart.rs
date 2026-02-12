//! Simulated UART peripheral for SITL.
//!
//! Provides in-memory ring buffers for TX/RX, enabling MAVLink message
//! flow between the autopilot and the simulation bridge.

use std::collections::VecDeque;

/// Simulated UART with in-memory ring buffers.
#[derive(Debug)]
pub struct SitlUart {
    baud_rate: u32,
    tx_buffer: VecDeque<u8>,
    rx_buffer: VecDeque<u8>,
    capacity: usize,
}

impl SitlUart {
    /// Default buffer capacity in bytes.
    const DEFAULT_CAPACITY: usize = 4096;

    /// Create a new SITL UART with the given baud rate.
    pub fn new(baud_rate: u32) -> Self {
        Self {
            baud_rate,
            tx_buffer: VecDeque::with_capacity(Self::DEFAULT_CAPACITY),
            rx_buffer: VecDeque::with_capacity(Self::DEFAULT_CAPACITY),
            capacity: Self::DEFAULT_CAPACITY,
        }
    }

    /// Write data to the TX buffer, returning bytes written.
    pub fn write(&mut self, data: &[u8]) -> usize {
        let available = self.capacity.saturating_sub(self.tx_buffer.len());
        let to_write = data.len().min(available);
        self.tx_buffer.extend(&data[..to_write]);
        to_write
    }

    /// Read data from the RX buffer, returning bytes read.
    pub fn read(&mut self, buffer: &mut [u8]) -> usize {
        let to_read = buffer.len().min(self.rx_buffer.len());
        for byte in buffer.iter_mut().take(to_read) {
            *byte = self.rx_buffer.pop_front().unwrap();
        }
        to_read
    }

    /// Check if data is available in the RX buffer.
    pub fn available(&self) -> bool {
        !self.rx_buffer.is_empty()
    }

    /// Inject data into the RX buffer (simulating received data).
    pub fn inject_rx_data(&mut self, data: &[u8]) {
        self.rx_buffer.extend(data);
    }

    /// Drain the TX buffer (simulating data being sent).
    pub fn drain_tx(&mut self) -> Vec<u8> {
        self.tx_buffer.drain(..).collect()
    }

    /// Get the current baud rate.
    pub fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    /// Set the baud rate.
    pub fn set_baud_rate(&mut self, baud_rate: u32) {
        self.baud_rate = baud_rate;
    }

    /// Flush the TX buffer (no-op in simulation, data is already buffered).
    pub fn flush(&mut self) {
        // No-op: data is immediately available in the buffer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_and_drain() {
        let mut uart = SitlUart::new(115200);
        let written = uart.write(b"Hello");
        assert_eq!(written, 5);
        let data = uart.drain_tx();
        assert_eq!(data, b"Hello");
    }

    #[test]
    fn test_inject_and_read() {
        let mut uart = SitlUart::new(115200);
        assert!(!uart.available());

        uart.inject_rx_data(b"World");
        assert!(uart.available());

        let mut buf = [0u8; 5];
        let read = uart.read(&mut buf);
        assert_eq!(read, 5);
        assert_eq!(&buf, b"World");
        assert!(!uart.available());
    }

    #[test]
    fn test_partial_read() {
        let mut uart = SitlUart::new(115200);
        uart.inject_rx_data(b"Hello World");

        let mut buf = [0u8; 5];
        let read = uart.read(&mut buf);
        assert_eq!(read, 5);
        assert_eq!(&buf, b"Hello");
        assert!(uart.available());
    }

    #[test]
    fn test_baud_rate() {
        let mut uart = SitlUart::new(9600);
        assert_eq!(uart.baud_rate(), 9600);
        uart.set_baud_rate(115200);
        assert_eq!(uart.baud_rate(), 115200);
    }
}
