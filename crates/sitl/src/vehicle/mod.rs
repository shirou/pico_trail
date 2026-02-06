pub mod config;

pub use config::{GeoPosition, VehicleConfig, VehicleType};

use crate::platform::SitlPlatform;
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
    /// Simulated platform for this vehicle.
    pub platform: SitlPlatform,
}

impl VehicleInstance {
    pub fn new(config: VehicleConfig) -> Self {
        let platform = SitlPlatform::new(config.id);
        Self {
            id: config.id,
            config,
            assigned_adapter: None,
            platform,
        }
    }
}
