#![cfg_attr(not(test), no_std)]

//! pico_trail - Embedded autopilot system for Raspberry Pi Pico W/2W
//!
//! This library provides platform abstraction, device drivers, and autopilot subsystems
//! for rover and boat autonomous navigation.

// Platform abstraction layer (NFR-nmmu0: Platform Code Isolation)
pub mod platform;

// Device drivers using platform abstraction
pub mod devices;

// Core systems (FR-5inw2: Task Scheduler)
pub mod core;

// Communication protocols (FR-gpzpz: MAVLink Protocol)
pub mod communication;

// Future modules (to be implemented in subsequent phases):
// pub mod subsystems;    // Subsystems (AHRS, Navigation, Control)
// pub mod vehicle;       // Vehicle logic and flight modes
