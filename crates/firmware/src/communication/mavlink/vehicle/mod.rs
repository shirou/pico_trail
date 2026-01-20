mod ground_rover;
mod surface_boat;

pub use ground_rover::GroundRover;
pub use surface_boat::SurfaceBoat;

use mavlink::common::{MavAutopilot, MavType};

pub trait VehicleType: Clone + Copy {
    type FlightMode: FlightModeOps + Clone + Copy + PartialEq + Default;

    fn mav_type() -> MavType;

    fn autopilot_type() -> MavAutopilot {
        MavAutopilot::MAV_AUTOPILOT_GENERIC
    }

    fn name() -> &'static str;
}

pub trait FlightModeOps {
    fn from_custom_mode(mode: u32) -> Option<Self>
    where
        Self: Sized;

    fn to_custom_mode(&self) -> u32;

    fn to_base_mode_flags(&self) -> u8;

    fn as_str(&self) -> &'static str;
}
