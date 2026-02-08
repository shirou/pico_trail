//! Vehicle type definitions
//!
//! Defines the `VehicleType` trait and its implementations for different
//! vehicle types (ground rover, surface boat).

use super::state::FlightMode;

/// MAVLink MAV_TYPE values (from MAVLink specification)
pub mod mav_type {
    /// Ground rover
    pub const GROUND_ROVER: u8 = 10;
    /// Surface boat
    pub const SURFACE_BOAT: u8 = 11;
}

/// MAVLink MAV_AUTOPILOT values (from MAVLink specification)
pub mod mav_autopilot {
    /// Generic autopilot
    pub const GENERIC: u8 = 0;
}

/// Vehicle type trait
///
/// Defines the vehicle type for MAVLink identification and flight mode mapping.
pub trait VehicleType: Clone + Copy {
    type FlightMode: FlightModeOps + Clone + Copy + PartialEq + Default;

    /// MAVLink vehicle type (MAV_TYPE enum value)
    fn mav_type() -> u8;

    /// MAVLink autopilot type (MAV_AUTOPILOT enum value)
    fn autopilot_type() -> u8 {
        mav_autopilot::GENERIC
    }

    /// Human-readable vehicle name
    fn name() -> &'static str;
}

/// Flight mode operations trait
///
/// Provides MAVLink-compatible mode conversion operations.
pub trait FlightModeOps {
    /// Convert from MAVLink custom mode number
    fn from_custom_mode(mode: u32) -> Option<Self>
    where
        Self: Sized;

    /// Convert to MAVLink custom mode number
    fn to_custom_mode(&self) -> u32;

    /// Convert to MAVLink base_mode flags
    fn to_base_mode_flags(&self) -> u8;

    /// Get human-readable name
    fn as_str(&self) -> &'static str;
}

impl FlightModeOps for FlightMode {
    fn from_custom_mode(mode: u32) -> Option<Self> {
        FlightMode::from_custom_mode(mode)
    }

    fn to_custom_mode(&self) -> u32 {
        FlightMode::to_custom_mode(*self)
    }

    fn to_base_mode_flags(&self) -> u8 {
        FlightMode::to_base_mode_flags(*self)
    }

    fn as_str(&self) -> &'static str {
        FlightMode::as_str(*self)
    }
}

/// Ground rover vehicle type
#[derive(Clone, Copy)]
pub struct GroundRover;

impl VehicleType for GroundRover {
    type FlightMode = FlightMode;

    fn mav_type() -> u8 {
        mav_type::GROUND_ROVER
    }

    fn name() -> &'static str {
        "Rover"
    }
}

/// Surface boat vehicle type
#[derive(Clone, Copy)]
pub struct SurfaceBoat;

impl VehicleType for SurfaceBoat {
    type FlightMode = FlightMode;

    fn mav_type() -> u8 {
        mav_type::SURFACE_BOAT
    }

    fn name() -> &'static str {
        "Boat"
    }
}

/// Convert VehicleType::mav_type() to MAVLink MavType enum
pub fn to_mav_type<V: VehicleType>() -> mavlink::common::MavType {
    use num_traits::FromPrimitive;
    mavlink::common::MavType::from_u8(V::mav_type())
        .unwrap_or(mavlink::common::MavType::MAV_TYPE_GENERIC)
}

/// Convert VehicleType::autopilot_type() to MAVLink MavAutopilot enum
pub fn to_mav_autopilot<V: VehicleType>() -> mavlink::common::MavAutopilot {
    use num_traits::FromPrimitive;
    mavlink::common::MavAutopilot::from_u8(V::autopilot_type())
        .unwrap_or(mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC)
}
