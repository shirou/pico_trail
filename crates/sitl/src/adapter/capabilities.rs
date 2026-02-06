/// Describes which sensor types a simulator can provide.
#[derive(Debug, Clone)]
pub struct SensorCapabilities {
    pub imu: bool,
    pub gps: bool,
    pub compass: bool,
    pub barometer: bool,
}

impl Default for SensorCapabilities {
    fn default() -> Self {
        Self {
            imu: true,
            gps: true,
            compass: true,
            barometer: true,
        }
    }
}

/// Describes the overall capabilities of a simulator backend.
#[derive(Debug, Clone)]
pub struct SimulatorCapabilities {
    /// Which sensor types are available.
    pub sensors: SensorCapabilities,
    /// Maximum simulation update rate in Hz.
    pub max_rate_hz: u32,
    /// Whether the simulator supports multiple vehicles.
    pub multi_vehicle: bool,
    /// Whether the simulator provides terrain data.
    pub terrain: bool,
    /// Whether the simulator provides wind simulation.
    pub wind: bool,
}

impl Default for SimulatorCapabilities {
    fn default() -> Self {
        Self {
            sensors: SensorCapabilities::default(),
            max_rate_hz: 400,
            multi_vehicle: false,
            terrain: false,
            wind: false,
        }
    }
}
