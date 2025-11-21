//! Mock I2C implementation for testing

use crate::platform::{
    traits::{I2cConfig, I2cInterface},
    Result,
};
use core::cell::RefCell;
use std::vec::Vec;

/// I2C transaction type for logging
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum I2cTransaction {
    /// Write transaction
    Write { addr: u8, data: Vec<u8> },
    /// Read transaction
    Read { addr: u8, len: usize },
    /// Write-Read transaction
    WriteRead {
        addr: u8,
        write_data: Vec<u8>,
        read_len: usize,
    },
}

/// Mock I2C implementation
///
/// Records all transactions for test verification and allows
/// pre-programming expected read data.
#[derive(Debug)]
pub struct MockI2c {
    config: I2cConfig,
    transactions: RefCell<Vec<I2cTransaction>>,
    read_data: RefCell<Vec<u8>>,
}

impl MockI2c {
    /// Create a new mock I2C
    pub fn new(config: I2cConfig) -> Self {
        Self {
            config,
            transactions: RefCell::new(Vec::new()),
            read_data: RefCell::new(Vec::new()),
        }
    }

    /// Get transaction log (for test verification)
    pub fn transactions(&self) -> Vec<I2cTransaction> {
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

impl I2cInterface for MockI2c {
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<()> {
        self.transactions.borrow_mut().push(I2cTransaction::Write {
            addr,
            data: data.to_vec(),
        });
        Ok(())
    }

    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<()> {
        self.transactions.borrow_mut().push(I2cTransaction::Read {
            addr,
            len: buffer.len(),
        });

        let mut read_data = self.read_data.borrow_mut();
        let to_read = core::cmp::min(buffer.len(), read_data.len());
        buffer[..to_read].copy_from_slice(&read_data[..to_read]);
        read_data.drain(..to_read);

        Ok(())
    }

    async fn write_read(
        &mut self,
        addr: u8,
        write_data: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<()> {
        self.transactions
            .borrow_mut()
            .push(I2cTransaction::WriteRead {
                addr,
                write_data: write_data.to_vec(),
                read_len: read_buffer.len(),
            });

        let mut read_data = self.read_data.borrow_mut();
        let to_read = core::cmp::min(read_buffer.len(), read_data.len());
        read_buffer[..to_read].copy_from_slice(&read_data[..to_read]);
        read_data.drain(..to_read);

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

    #[tokio::test]
    async fn test_mock_i2c_write() {
        let mut i2c = MockI2c::new(I2cConfig::default());
        i2c.write(0x50, &[0x01, 0x02, 0x03]).await.unwrap();

        let transactions = i2c.transactions();
        assert_eq!(transactions.len(), 1);
        assert_eq!(
            transactions[0],
            I2cTransaction::Write {
                addr: 0x50,
                data: vec![0x01, 0x02, 0x03]
            }
        );
    }

    #[tokio::test]
    async fn test_mock_i2c_read() {
        let mut i2c = MockI2c::new(I2cConfig::default());
        i2c.set_read_data(&[0xAA, 0xBB, 0xCC]);

        let mut buffer = [0u8; 3];
        i2c.read(0x51, &mut buffer).await.unwrap();

        assert_eq!(buffer, [0xAA, 0xBB, 0xCC]);

        let transactions = i2c.transactions();
        assert_eq!(transactions.len(), 1);
        assert_eq!(transactions[0], I2cTransaction::Read { addr: 0x51, len: 3 });
    }

    #[tokio::test]
    async fn test_mock_i2c_write_read() {
        let mut i2c = MockI2c::new(I2cConfig::default());
        i2c.set_read_data(&[0x12, 0x34]);

        let mut read_buf = [0u8; 2];
        i2c.write_read(0x52, &[0xA0], &mut read_buf).await.unwrap();

        assert_eq!(read_buf, [0x12, 0x34]);

        let transactions = i2c.transactions();
        assert_eq!(transactions.len(), 1);
        assert_eq!(
            transactions[0],
            I2cTransaction::WriteRead {
                addr: 0x52,
                write_data: vec![0xA0],
                read_len: 2
            }
        );
    }

    #[test]
    fn test_mock_i2c_frequency() {
        let mut i2c = MockI2c::new(I2cConfig::default());
        assert_eq!(i2c.frequency(), 100_000);

        i2c.set_frequency(400_000).unwrap();
        assert_eq!(i2c.frequency(), 400_000);
    }
}
