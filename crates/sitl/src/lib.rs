pub mod adapter;
pub mod bridge;
pub mod error;
pub mod gcs;
pub mod platform;
pub mod types;
pub mod vehicle;

pub use adapter::{LightweightAdapter, LightweightConfig, SimulatorAdapter, SimulatorCapabilities};
pub use bridge::{SitlBridge, TimeCoordinator, TimeMode};
pub use error::SimulatorError;
pub use gcs::GcsLink;
pub use platform::SitlPlatform;
pub use types::{ActuatorCommands, SensorData, VehicleId};
pub use vehicle::{VehicleConfig, VehicleInstance, VehicleType};
