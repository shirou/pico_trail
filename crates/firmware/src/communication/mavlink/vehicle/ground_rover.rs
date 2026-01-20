use super::VehicleType;
use crate::communication::mavlink::state::FlightMode;
use mavlink::common::MavType;

#[derive(Clone, Copy)]
pub struct GroundRover;

impl VehicleType for GroundRover {
    type FlightMode = FlightMode;

    fn mav_type() -> MavType {
        MavType::MAV_TYPE_GROUND_ROVER
    }

    fn name() -> &'static str {
        "Rover"
    }
}
