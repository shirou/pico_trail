use super::VehicleType;
use crate::communication::mavlink::state::FlightMode;
use mavlink::common::MavType;

#[derive(Clone, Copy)]
pub struct SurfaceBoat;

impl VehicleType for SurfaceBoat {
    type FlightMode = FlightMode;

    fn mav_type() -> MavType {
        MavType::MAV_TYPE_SURFACE_BOAT
    }

    fn name() -> &'static str {
        "Boat"
    }
}
