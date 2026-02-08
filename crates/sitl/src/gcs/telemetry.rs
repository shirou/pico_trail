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

/// Build an ATTITUDE message from attitude quaternion and IMU data.
///
/// If a quaternion [w, x, y, z] is provided, Euler angles are derived from it.
/// Otherwise, roll/pitch are estimated from accelerometer data with yaw=0.
pub fn build_attitude(
    imu: &ImuData,
    attitude_quat: Option<&[f32; 4]>,
    time_boot_ms: u32,
) -> MavMessage {
    let (roll, pitch, yaw) = if let Some(q) = attitude_quat {
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let sin_pitch = 2.0 * (w * y - z * x);
        let pitch = if sin_pitch.abs() >= 1.0 {
            sin_pitch.signum() * core::f32::consts::FRAC_PI_2
        } else {
            sin_pitch.asin()
        };
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        (roll, pitch, yaw)
    } else {
        // Fallback: derive roll/pitch from accelerometer (no yaw)
        let ax = imu.accel_mss[0] as f64;
        let ay = imu.accel_mss[1] as f64;
        let az = imu.accel_mss[2] as f64;
        let roll = ay.atan2(az) as f32;
        let pitch = (-ax).atan2((ay * ay + az * az).sqrt()) as f32;
        (roll, pitch, 0.0)
    };

    MavMessage::ATTITUDE(ATTITUDE_DATA {
        time_boot_ms,
        roll,
        pitch,
        yaw,
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
        // v2 extensions
        alt_ellipsoid: 0,
        h_acc: 0,
        v_acc: 0,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: 0,
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
        // v2 extensions
        onboard_control_sensors_present_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended: MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended: MavSysStatusSensorExtended::empty(),
    })
}

/// Build telemetry messages from sensor data for a given simulation time.
pub fn build_telemetry(sensors: &SensorData) -> TelemetrySet {
    let time_boot_ms = (sensors.timestamp_us / 1000) as u32;

    let attitude = sensors
        .imu
        .as_ref()
        .map(|imu| build_attitude(imu, sensors.attitude_quat.as_ref(), time_boot_ms));

    // Derive heading from quaternion yaw if available, otherwise from GPS course
    let heading_from_quat = sensors.attitude_quat.as_ref().map(|q| {
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let yaw_rad = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        let yaw_deg = yaw_rad.to_degrees();
        let yaw_deg = if yaw_deg < 0.0 {
            yaw_deg + 360.0
        } else {
            yaw_deg
        };
        (yaw_deg * 100.0) as u16
    });

    let (gps_raw, global_pos) = match &sensors.gps {
        Some(gps) => {
            let heading_cdeg = heading_from_quat.unwrap_or((gps.course_deg * 100.0) as u16);
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
        let imu = ImuData {
            accel_mss: [0.0, 0.0, 9.81],
            gyro_rads: [0.01, -0.02, 0.03],
            temperature_c: 25.0,
        };

        // With quaternion: identity = level, facing north
        let quat = [1.0, 0.0, 0.0, 0.0_f32];
        let msg = build_attitude(&imu, Some(&quat), 1000);
        match msg {
            MavMessage::ATTITUDE(data) => {
                assert_eq!(data.time_boot_ms, 1000);
                assert!(data.roll.abs() < 0.01);
                assert!(data.pitch.abs() < 0.01);
                assert!(data.yaw.abs() < 0.01);
                assert!((data.rollspeed - 0.01).abs() < 1e-5);
            }
            _ => panic!("Expected ATTITUDE"),
        }

        // Without quaternion: fallback to accel-derived attitude
        let msg = build_attitude(&imu, None, 2000);
        match msg {
            MavMessage::ATTITUDE(data) => {
                assert_eq!(data.time_boot_ms, 2000);
                assert!(data.roll.abs() < 0.01);
                assert!(data.pitch.abs() < 0.01);
                assert_eq!(data.yaw, 0.0); // No yaw without quaternion
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
            attitude_quat: None,
        };
        let set = build_telemetry(&sensors);
        assert!(set.attitude.is_some());
        assert!(set.gps_raw.is_some());
        assert!(set.global_pos.is_some());
    }
}
