//! RP2350 Flash implementation
//!
//! This module provides Flash storage support for RP2350 using ROM functions.
//!
//! # Flash Layout
//!
//! ```text
//! [Firmware]           0x000000 - 0x040000 (256 KB) - PROTECTED
//! [Parameter Block 0]  0x040000 - 0x041000 (4 KB)
//! [Parameter Block 1]  0x041000 - 0x042000 (4 KB)
//! [Parameter Block 2]  0x042000 - 0x043000 (4 KB)
//! [Parameter Block 3]  0x043000 - 0x044000 (4 KB)
//! [Mission Storage]    0x044000 - 0x046000 (8 KB)
//! [Log Storage]        0x046000 - 0x400000 (~3.7 MB)
//! ```
//!
//! # Safety
//!
//! Flash operations use unsafe ROM functions and must:
//! - Disable interrupts during operations (XIP inaccessible)
//! - Not access XIP memory during erase/write
//! - Validate addresses are not in firmware region

use crate::platform::{error::FlashError, traits::FlashInterface, Result};
use rp235x_hal::rom_data;

/// Minimum firmware size (protect first 256 KB)
const FIRMWARE_SIZE: u32 = 0x40000; // 256 KB

/// Flash block size (minimum erase unit)
const BLOCK_SIZE: u32 = 4096; // 4 KB

/// Flash sector erase command (0x20 for 4KB sector)
const SECTOR_ERASE_CMD: u8 = 0x20;

/// Total Flash capacity for Pico 2 W
const FLASH_CAPACITY: u32 = 4 * 1024 * 1024; // 4 MB

/// RP2350 Flash implementation
///
/// Provides Flash read/write/erase operations using RP2350 ROM functions.
///
/// # Important
///
/// - Flash operations are blocking (can take 100ms+)
/// - XIP is inaccessible during erase/write (other core will fault)
/// - Firmware region (0x000000-0x03FFFF) is protected from writes
///
/// # Example
///
/// ```no_run
/// use pico_trail::platform::rp2350::Rp2350Flash;
/// use pico_trail::platform::traits::FlashInterface;
///
/// let mut flash = Rp2350Flash::new();
///
/// // Erase parameter block 0 (4 KB at 0x040000)
/// flash.erase(0x040000, 4096).unwrap();
///
/// // Write parameter data
/// let data = [0x50, 0x41, 0x52, 0x41]; // "PARA" magic
/// flash.write(0x040000, &data).unwrap();
///
/// // Read back
/// let mut buf = [0u8; 4];
/// flash.read(0x040000, &mut buf).unwrap();
/// assert_eq!(buf, data);
/// ```
pub struct Rp2350Flash;

impl Rp2350Flash {
    /// Create a new RP2350 Flash instance
    pub fn new() -> Self {
        Self
    }

    /// Check if address is in writable region (not firmware)
    fn is_writable(&self, address: u32) -> bool {
        (FIRMWARE_SIZE..FLASH_CAPACITY).contains(&address)
    }

    /// Check if address is block-aligned
    fn is_block_aligned(&self, address: u32) -> bool {
        address.is_multiple_of(BLOCK_SIZE)
    }

    /// Execute Flash operation with XIP disabled
    ///
    /// # Safety
    ///
    /// - Interrupts are disabled during operation
    /// - XIP is inaccessible (other core will fault if accessing Flash)
    /// - Must not access XIP memory in the closure
    unsafe fn with_xip_disabled<F, R>(&mut self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        // SAFETY: Use critical section to prevent interrupts during Flash operation
        // This is essential because XIP will be disabled and any interrupt trying
        // to access Flash-based code will cause a fault
        cortex_m::interrupt::free(|_cs| {
            // SAFETY: Prepare Flash for serial operations
            // This must be called before any erase/program operations
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();

            // Execute the operation
            let result = f();

            // SAFETY: Flush cache to make changes visible
            rom_data::flash_flush_cache();

            // SAFETY: Restore XIP mode for normal operation
            rom_data::flash_enter_cmd_xip();

            result
        })
    }
}

impl Default for Rp2350Flash {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashInterface for Rp2350Flash {
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<()> {
        // Validate address range
        if address >= FLASH_CAPACITY {
            return Err(FlashError::InvalidAddress.into());
        }

        if address as usize + buf.len() > FLASH_CAPACITY as usize {
            return Err(FlashError::InvalidAddress.into());
        }

        // SAFETY: Flash is memory-mapped at XIP_BASE (0x10000000)
        // Reading from XIP is safe and doesn't require disabling XIP
        const XIP_BASE: usize = 0x10000000;
        let flash_ptr = (XIP_BASE + address as usize) as *const u8;

        // SAFETY: We validated the address range above
        unsafe {
            core::ptr::copy_nonoverlapping(flash_ptr, buf.as_mut_ptr(), buf.len());
        }

        Ok(())
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<()> {
        // Validate address is in writable region
        if !self.is_writable(address) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate write doesn't exceed Flash capacity
        if address as usize + data.len() > FLASH_CAPACITY as usize {
            return Err(FlashError::InvalidAddress.into());
        }

        // Note: Write alignment is 256 bytes, but we don't enforce it here
        // to allow flexibility. The ROM function will handle unaligned writes.

        // SAFETY: Execute Flash write with XIP disabled
        // XIP must be disabled because write operations use direct mode
        unsafe {
            self.with_xip_disabled(|| {
                // SAFETY: ROM function validates alignment and performs write
                // addr: offset from Flash start (not XIP address)
                // data: pointer to data buffer
                // count: number of bytes to write
                rom_data::flash_range_program(address, data.as_ptr(), data.len());
            });
        }

        Ok(())
    }

    fn erase(&mut self, address: u32, size: u32) -> Result<()> {
        // Validate address is in writable region
        if !self.is_writable(address) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate address is block-aligned
        if !self.is_block_aligned(address) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate size is multiple of block size
        if !size.is_multiple_of(BLOCK_SIZE) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate erase doesn't exceed Flash capacity
        if address + size > FLASH_CAPACITY {
            return Err(FlashError::InvalidAddress.into());
        }

        // SAFETY: Execute Flash erase with XIP disabled
        unsafe {
            self.with_xip_disabled(|| {
                // SAFETY: ROM function performs erase operation
                // addr: offset from Flash start
                // count: number of bytes to erase (must be multiple of 4096)
                // block_size: 4096 (use sector erase)
                // block_cmd: 0x20 (4KB sector erase command)
                rom_data::flash_range_erase(address, size as usize, BLOCK_SIZE, SECTOR_ERASE_CMD);
            });
        }

        Ok(())
    }

    fn block_size(&self) -> u32 {
        BLOCK_SIZE
    }

    fn capacity(&self) -> u32 {
        FLASH_CAPACITY
    }
}
