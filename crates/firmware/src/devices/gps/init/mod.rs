//! GPS vendor-specific initialization modules
//!
//! This module provides compile-time selectable initialization routines
//! for different GPS vendors. Use the appropriate feature flag to enable
//! the desired vendor support.
//!
//! # Available Vendors
//!
//! - `gps-ublox` - u-blox NEO-M8N and compatible modules

#[cfg(feature = "gps-ublox")]
pub mod ublox;
