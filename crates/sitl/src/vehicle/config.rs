use crate::types::VehicleId;

/// Type of vehicle to simulate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleType {
    Rover,
    Boat,
}

/// Geographic position for initial vehicle placement.
#[derive(Debug, Clone, Copy)]
pub struct GeoPosition {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
}

/// Configuration for spawning a new simulated vehicle.
#[derive(Debug, Clone)]
pub struct VehicleConfig {
    /// Vehicle identifier (must be unique).
    pub id: VehicleId,
    /// MAVLink port for this vehicle (default: 14550 + id).
    pub mavlink_port: u16,
    /// Type of vehicle to simulate.
    pub vehicle_type: VehicleType,
    /// Initial position (optional).
    pub initial_position: Option<GeoPosition>,
}

impl VehicleConfig {
    /// Create a new vehicle configuration with default MAVLink port.
    pub fn new(id: VehicleId, vehicle_type: VehicleType) -> Self {
        Self {
            id,
            mavlink_port: 14550 + id.0 as u16,
            vehicle_type,
            initial_position: None,
        }
    }
}
