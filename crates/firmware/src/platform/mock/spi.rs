//! Mock SPI implementation for testing

use crate::platform::{
    traits::{SpiConfig, SpiInterface},
    Result,
};
use core::cell::RefCell;
use std::vec::Vec;

/// SPI transaction type for logging
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SpiTransaction {
    /// Transfer (full-duplex)
    Transfer { write: Vec<u8>, read: Vec<u8> },
    /// Write only
    Write { data: Vec<u8> },
    /// Read only
    Read { len: usize },
}

/// Mock SPI implementation
///
/// Records all transactions for test verification and allows
/// pre-programming expected read data.
#[derive(Debug)]
pub struct MockSpi {
    config: SpiConfig,
    transactions: RefCell<Vec<SpiTransaction>>,
    read_data: RefCell<Vec<u8>>,
}

impl MockSpi {
    /// Create a new mock SPI
    pub fn new(config: SpiConfig) -> Self {
        Self {
            config,
            transactions: RefCell::new(Vec::new()),
            read_data: RefCell::new(Vec::new()),
        }
    }

    /// Get transaction log (for test verification)
    pub fn transactions(&self) -> Vec<SpiTransaction> {
        self.transactions.borrow().clone()
    }

    /// Clear transaction log
    pub fn clear_transactions(&mut self) {
        self.transactions.borrow_mut().clear();
    }

    /// Set data to return for read operations
    pub fn set_read_data(&mut self, data: &[u8]) {
        *self.read_data.borrow_mut() = data.to_vec();
    }

    /// Get current frequency
    pub fn frequency(&self) -> u32 {
        self.config.frequency
    }
}

impl SpiInterface for MockSpi {
    fn transfer(&mut self, write_buffer: &[u8], read_buffer: &mut [u8]) -> Result<()> {
        let mut read_data = self.read_data.borrow_mut();
        let to_read = core::cmp::min(read_buffer.len(), read_data.len());
        read_buffer[..to_read].copy_from_slice(&read_data[..to_read]);
        read_data.drain(..to_read);

        self.transactions
            .borrow_mut()
            .push(SpiTransaction::Transfer {
                write: write_buffer.to_vec(),
                read: read_buffer.to_vec(),
            });

        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.transactions.borrow_mut().push(SpiTransaction::Write {
            data: data.to_vec(),
        });
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<()> {
        let mut read_data = self.read_data.borrow_mut();
        let to_read = core::cmp::min(buffer.len(), read_data.len());
        buffer[..to_read].copy_from_slice(&read_data[..to_read]);
        read_data.drain(..to_read);

        self.transactions
            .borrow_mut()
            .push(SpiTransaction::Read { len: buffer.len() });

        Ok(())
    }

    fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        self.config.frequency = frequency;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_spi_write() {
        let mut spi = MockSpi::new(SpiConfig::default());
        spi.write(&[0x01, 0x02, 0x03]).unwrap();

        let transactions = spi.transactions();
        assert_eq!(transactions.len(), 1);
        assert_eq!(
            transactions[0],
            SpiTransaction::Write {
                data: vec![0x01, 0x02, 0x03]
            }
        );
    }

    #[test]
    fn test_mock_spi_read() {
        let mut spi = MockSpi::new(SpiConfig::default());
        spi.set_read_data(&[0xAA, 0xBB, 0xCC]);

        let mut buffer = [0u8; 3];
        spi.read(&mut buffer).unwrap();

        assert_eq!(buffer, [0xAA, 0xBB, 0xCC]);

        let transactions = spi.transactions();
        assert_eq!(transactions.len(), 1);
        assert_eq!(transactions[0], SpiTransaction::Read { len: 3 });
    }

    #[test]
    fn test_mock_spi_transfer() {
        let mut spi = MockSpi::new(SpiConfig::default());
        spi.set_read_data(&[0x12, 0x34]);

        let mut read_buf = [0u8; 2];
        spi.transfer(&[0xA0, 0xB0], &mut read_buf).unwrap();

        assert_eq!(read_buf, [0x12, 0x34]);

        let transactions = spi.transactions();
        assert_eq!(transactions.len(), 1);
        assert_eq!(
            transactions[0],
            SpiTransaction::Transfer {
                write: vec![0xA0, 0xB0],
                read: vec![0x12, 0x34]
            }
        );
    }

    #[test]
    fn test_mock_spi_frequency() {
        let mut spi = MockSpi::new(SpiConfig::default());
        assert_eq!(spi.frequency(), 1_000_000);

        spi.set_frequency(2_000_000).unwrap();
        assert_eq!(spi.frequency(), 2_000_000);
    }
}
