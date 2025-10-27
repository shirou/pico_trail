#![no_std]

//! pico_trail - Embedded autopilot system for Raspberry Pi Pico W/2W
//!
//! This library provides platform abstraction, device drivers, and autopilot subsystems
//! for rover and boat autonomous navigation.

// Platform abstraction layer (NFR-nmmu0: Platform Code Isolation)
pub mod platform;

// Future modules (to be implemented in subsequent phases):
// pub mod devices;       // Device drivers (GPS, IMU, Motor, Servo)
// pub mod subsystems;    // Subsystems (AHRS, Navigation, Control)
// pub mod vehicle;       // Vehicle logic and flight modes
// pub mod communication; // MAVLink protocol and telemetry
// pub mod core;          // Core systems (Scheduler, Parameters, Logger)
