//! Example tasks demonstrating Embassy task usage with scheduler
//!
//! This module contains sample tasks that demonstrate how to use the
//! scheduler infrastructure with Embassy async tasks. Each task shows
//! the pattern of:
//! 1. Registering task metadata
//! 2. Using Ticker for periodic execution
//! 3. Measuring execution time
//! 4. Updating statistics
//!
//! These tasks are examples and placeholders. Real implementations will
//! interact with actual hardware peripherals.

#[cfg(feature = "pico2_w")]
pub mod examples;
