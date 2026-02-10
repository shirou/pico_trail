//! Mode Manager
//!
//! Re-exports `ModeExecutor` from the core crate as `ModeManager` for backward
//! compatibility with existing firmware code.
//!
//! ## References
//!
//! - ADR-w9zpl-control-mode-architecture: Trait-based mode architecture
//! - FR-q2sjt-control-mode-framework: Mode framework requirements

pub use pico_trail_core::mode::ModeExecutor as ModeManager;
