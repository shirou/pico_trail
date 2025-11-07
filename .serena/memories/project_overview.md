# Project Overview

## Purpose

pico_trail is an embedded autopilot system for Raspberry Pi Pico W and Pico 2 W, targeting rover and boat autonomous navigation. Built with embedded Rust for memory safety and real-time performance, it implements a subset of ArduPilot/Pixhawk functionality optimized for resource-constrained microcontrollers.

## Key Features

- Autonomous waypoint-based navigation with S-curve path planning
- MAVLink 2.0 compatible (QGroundControl, Mission Planner)
- WiFi-enabled UDP transport for wireless telemetry (Pico 2 W)
- Real-time control with 50Hz minimum performance
- Memory-safe Rust implementation
- Platform abstraction for portability
- Safety features: GPS/RC failsafe, battery monitoring, geofencing

## Tech Stack

- **Language**: Rust (embedded no_std)
- **Async Framework**: Embassy (executor, time, sync, net)
- **Protocol**: MAVLink 2.0 (mavlink crate)
- **Math Libraries**: nalgebra, micromath
- **HAL**: rp235x-hal for RP2350, embedded-hal traits
- **WiFi Driver**: cyw43 for CYW43439 chip (Pico W/2W)
- **Target**: thumbv8m.main-none-eabihf (Cortex-M33 for RP2350)

## Target Platforms

- Raspberry Pi Pico 2 W (RP2350, 4MB flash, 520KB RAM) - Primary target
- Raspberry Pi Pico W (RP2040, 2MB flash, 264KB RAM) - Planned

## Development Focus

- Memory safety over micro-optimizations
- Real-time performance (50Hz control loops)
- MAVLink compatibility
- Rover/boat focus (no aerial drone features)
