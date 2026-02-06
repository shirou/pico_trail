pub mod config;

pub use config::{GeoPosition, VehicleConfig, VehicleType};

use crate::types::VehicleId;

/// A live simulated vehicle instance.
#[derive(Debug)]
pub struct VehicleInstance {
    /// Vehicle identifier.
    pub id: VehicleId,
    /// Configuration used to spawn this vehicle.
    pub config: VehicleConfig,
    /// Name of the adapter this vehicle is assigned to (if any).
    pub assigned_adapter: Option<String>,
}

impl VehicleInstance {
    pub fn new(config: VehicleConfig) -> Self {
        Self {
            id: config.id,
            config,
            assigned_adapter: None,
        }
    }
}
