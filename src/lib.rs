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

// Parameter storage for WiFi configuration and runtime settings
pub mod parameters;

// Subsystems (FR-eyuh8: AHRS, Navigation, Control)
pub mod subsystems;

// Common libraries (ArduPilot libraries/ equivalent)
// RC input, servo output, and other vehicle-agnostic functionality
pub mod libraries;

// Rover vehicle implementation (ArduPilot Rover/ equivalent)
pub mod rover;
