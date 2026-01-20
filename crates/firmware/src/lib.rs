#![cfg_attr(not(test), no_std)]

//! pico_trail_firmware - Embassy firmware for pico_trail autopilot
//!
//! This crate provides Embassy async wrappers and RP2350-specific
//! implementations for the core business logic.
//!
//! # Design Principles
//!
//! - **Embassy tasks**: Async tasks for control and communication
//! - **Platform implementations**: TimeSource, storage, and HAL bindings
//! - **defmt formatters**: External Format impls for core types
//! - **Device drivers**: Hardware-specific drivers for sensors and actuators

// Platform abstraction layer (NFR-nmmu0: Platform Code Isolation)
pub mod platform;

// Device drivers using platform abstraction
pub mod devices;

// Core systems (FR-5inw2: Task Scheduler) - contains firmware-specific core code
// and re-exports from pico_trail_core
pub mod core;

// Communication protocols (FR-gpzpz: MAVLink Protocol)
pub mod communication;

// Parameter storage for WiFi configuration and runtime settings
pub mod parameters;

// Subsystems (FR-eyuh8: AHRS, Navigation, Control)
pub mod subsystems;

// Common libraries (ArduPilot libraries/ equivalent)
pub mod libraries;

// Rover vehicle implementation (ArduPilot Rover/ equivalent)
pub mod rover;

// Note: Logging macros (log_info!, log_warn!, log_error!, log_debug!, log_trace!)
// are exported at crate root via #[macro_export] in core::logging
