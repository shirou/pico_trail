//! Flash interface trait
//!
//! Re-exports the platform-agnostic FlashInterface trait from core.
//! Platform implementations (Rp2350Flash, MockFlash) are in the firmware crate.

pub use pico_trail_core::traits::flash::{FlashError, FlashInterface};
