//! Telemetry message builders converting SensorData to MAVLink messages.

use crate::types::{GpsData, GpsFixType, ImuData, SensorData};
use mavlink::common::*;

/// Build a HEARTBEAT message for a ground rover.
pub fn build_heartbeat() -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GROUND_ROVER,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

/// Build an ATTITUDE message from IMU data.
pub fn build_attitude(imu: &ImuData, time_boot_ms: u32) -> MavMessage {
    // Derive roll/pitch from accelerometer (simple tilt estimation)
    let ax = imu.accel_mss[0] as f64;
    let ay = imu.accel_mss[1] as f64;
    let az = imu.accel_mss[2] as f64;
    let roll = ay.atan2(az) as f32;
    let pitch = (-ax).atan2((ay * ay + az * az).sqrt()) as f32;

    MavMessage::ATTITUDE(ATTITUDE_DATA {
        time_boot_ms,
        roll,
        pitch,
        yaw: 0.0, // No magnetometer heading in basic IMU
        rollspeed: imu.gyro_rads[0],
        pitchspeed: imu.gyro_rads[1],
        yawspeed: imu.gyro_rads[2],
    })
}

/// Build a GPS_RAW_INT message from GPS data.
pub fn build_gps_raw_int(gps: &GpsData, time_us: u64) -> MavMessage {
    MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
        time_usec: time_us,
        lat: (gps.lat_deg * 1e7) as i32,
        lon: (gps.lon_deg * 1e7) as i32,
        alt: (gps.alt_m * 1000.0) as i32,
        eph: (gps.hdop * 100.0) as u16,
        epv: u16::MAX, // Unknown
        vel: (gps.speed_ms * 100.0) as u16,
        cog: (gps.course_deg * 100.0) as u16,
        fix_type: gps_fix_to_mav(gps.fix_type),
        satellites_visible: gps.satellites,
    })
}

/// Build a GLOBAL_POSITION_INT message from GPS data (used by Mission Planner for map display).
pub fn build_global_position_int(
    gps: &GpsData,
    heading_cdeg: u16,
    time_boot_ms: u32,
) -> MavMessage {
    MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
        time_boot_ms,
        lat: (gps.lat_deg * 1e7) as i32,
        lon: (gps.lon_deg * 1e7) as i32,
        alt: (gps.alt_m * 1000.0) as i32,
        relative_alt: (gps.alt_m * 1000.0) as i32,
        vx: 0,
        vy: 0,
        vz: 0,
        hdg: heading_cdeg,
    })
}

/// Build a SYS_STATUS message with battery voltage.
pub fn build_sys_status(battery_mv: u16) -> MavMessage {
    MavMessage::SYS_STATUS(SYS_STATUS_DATA {
        onboard_control_sensors_present: MavSysStatusSensor::empty(),
        onboard_control_sensors_enabled: MavSysStatusSensor::empty(),
        onboard_control_sensors_health: MavSysStatusSensor::empty(),
        load: 0,
        voltage_battery: battery_mv,
        current_battery: -1,
        battery_remaining: -1,
        drop_rate_comm: 0,
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
    })
}

/// Build telemetry messages from sensor data for a given simulation time.
pub fn build_telemetry(sensors: &SensorData) -> TelemetrySet {
    let time_boot_ms = (sensors.timestamp_us / 1000) as u32;

    let attitude = sensors
        .imu
        .as_ref()
        .map(|imu| build_attitude(imu, time_boot_ms));

    let (gps_raw, global_pos) = match &sensors.gps {
        Some(gps) => {
            let heading_cdeg = (gps.course_deg * 100.0) as u16;
            (
                Some(build_gps_raw_int(gps, sensors.timestamp_us)),
                Some(build_global_position_int(gps, heading_cdeg, time_boot_ms)),
            )
        }
        None => (None, None),
    };

    TelemetrySet {
        attitude,
        gps_raw,
        global_pos,
    }
}

/// A set of telemetry messages built from sensor data.
pub struct TelemetrySet {
    pub attitude: Option<MavMessage>,
    pub gps_raw: Option<MavMessage>,
    pub global_pos: Option<MavMessage>,
}

/// Convert our GpsFixType to the mavlink common GpsFixType.
fn gps_fix_to_mav(fix: GpsFixType) -> mavlink::common::GpsFixType {
    match fix {
        GpsFixType::NoFix => mavlink::common::GpsFixType::GPS_FIX_TYPE_NO_FIX,
        GpsFixType::Fix2D => mavlink::common::GpsFixType::GPS_FIX_TYPE_2D_FIX,
        GpsFixType::Fix3D => mavlink::common::GpsFixType::GPS_FIX_TYPE_3D_FIX,
        GpsFixType::DGps => mavlink::common::GpsFixType::GPS_FIX_TYPE_DGPS,
        GpsFixType::RtkFloat => mavlink::common::GpsFixType::GPS_FIX_TYPE_RTK_FLOAT,
        GpsFixType::RtkFixed => mavlink::common::GpsFixType::GPS_FIX_TYPE_RTK_FIXED,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::VehicleId;

    #[test]
    fn test_build_heartbeat() {
        let msg = build_heartbeat();
        match msg {
            MavMessage::HEARTBEAT(data) => {
                assert_eq!(data.mavtype, MavType::MAV_TYPE_GROUND_ROVER);
                assert_eq!(data.system_status, MavState::MAV_STATE_ACTIVE);
            }
            _ => panic!("Expected HEARTBEAT"),
        }
    }

    #[test]
    fn test_build_attitude() {
        // Level orientation with z-up (positive 9.81 m/s²)
        let imu = ImuData {
            accel_mss: [0.0, 0.0, 9.81],
            gyro_rads: [0.01, -0.02, 0.03],
            temperature_c: 25.0,
        };
        let msg = build_attitude(&imu, 1000);
        match msg {
            MavMessage::ATTITUDE(data) => {
                assert_eq!(data.time_boot_ms, 1000);
                assert!((data.roll).abs() < 0.01);
                assert!((data.pitch).abs() < 0.01);
                assert!((data.rollspeed - 0.01).abs() < 1e-5);
            }
            _ => panic!("Expected ATTITUDE"),
        }
    }

    #[test]
    fn test_build_gps_raw_int() {
        let gps = GpsData {
            lat_deg: 35.681236,
            lon_deg: 139.767125,
            alt_m: 40.0,
            speed_ms: 1.5,
            course_deg: 90.0,
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
            hdop: 1.2,
        };
        let msg = build_gps_raw_int(&gps, 1_000_000);
        match msg {
            MavMessage::GPS_RAW_INT(data) => {
                assert_eq!(data.lat, 356812360);
                assert_eq!(data.lon, 1397671250);
                assert_eq!(data.alt, 40000);
                assert_eq!(data.vel, 150);
                assert_eq!(data.satellites_visible, 12);
            }
            _ => panic!("Expected GPS_RAW_INT"),
        }
    }

    #[test]
    fn test_build_telemetry() {
        let sensors = SensorData {
            timestamp_us: 500_000,
            vehicle_id: VehicleId(1),
            imu: Some(ImuData {
                accel_mss: [0.0, 0.0, -9.81],
                gyro_rads: [0.0, 0.0, 0.0],
                temperature_c: 25.0,
            }),
            gps: Some(GpsData {
                lat_deg: 35.0,
                lon_deg: 139.0,
                alt_m: 10.0,
                speed_ms: 0.0,
                course_deg: 0.0,
                fix_type: GpsFixType::Fix3D,
                satellites: 10,
                hdop: 1.0,
            }),
            compass: None,
            barometer: None,
        };
        let set = build_telemetry(&sensors);
        assert!(set.attitude.is_some());
        assert!(set.gps_raw.is_some());
        assert!(set.global_pos.is_some());
    }
}
