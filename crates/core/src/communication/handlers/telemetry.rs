//! MAVLink Telemetry Streaming
//!
//! Implements periodic telemetry message streaming to Ground Control Stations.

use crate::autopilot::state::SystemState;
use crate::autopilot::vehicle::VehicleType;
use crate::mission::{MissionState, MISSION_SEQUENCER, MISSION_STORAGE};
use crate::navigation::GpsFixType as DeviceGpsFixType;
use crate::traits::sync::SharedState;
use core::marker::PhantomData;
use mavlink::common::{
    GpsFixType, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction, MavBatteryMode,
    MavBatteryType, MavMessage, MavModeFlag, MavState, MavSysStatusSensor,
    MavSysStatusSensorExtended, ATTITUDE_DATA, BATTERY_STATUS_DATA, GLOBAL_POSITION_INT_DATA,
    GPS_RAW_INT_DATA, HEARTBEAT_DATA, MISSION_CURRENT_DATA, SYS_STATUS_DATA,
};
#[allow(unused_imports)]
use micromath::F32Ext;

fn degrees_to_deg_e7(degrees: f32) -> i32 {
    ((degrees as f64) * 1e7) as i32
}

fn meters_to_mm(meters: f32) -> i32 {
    (meters * 1000.0) as i32
}

fn mps_to_cms(mps: f32) -> u16 {
    (mps * 100.0).clamp(0.0, u16::MAX as f32) as u16
}

fn mps_to_cms_i16(mps: f32) -> i16 {
    (mps * 100.0).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

fn degrees_to_cdeg(degrees: f32) -> u16 {
    (degrees * 100.0).clamp(0.0, 35999.0) as u16
}

fn radians_to_cdeg(radians: f32) -> u16 {
    let degrees = radians * 180.0 / core::f32::consts::PI;
    let normalized = if degrees < 0.0 {
        degrees + 360.0
    } else if degrees >= 360.0 {
        degrees - 360.0
    } else {
        degrees
    };
    (normalized * 100.0).clamp(0.0, 35999.0) as u16
}

fn normalize_yaw(yaw: f32) -> f32 {
    let mut result = yaw;
    while result > core::f32::consts::PI {
        result -= 2.0 * core::f32::consts::PI;
    }
    while result < -core::f32::consts::PI {
        result += 2.0 * core::f32::consts::PI;
    }
    result
}

fn convert_gps_fix_type(fix_type: DeviceGpsFixType) -> GpsFixType {
    match fix_type {
        DeviceGpsFixType::NoFix => GpsFixType::GPS_FIX_TYPE_NO_FIX,
        DeviceGpsFixType::Fix2D => GpsFixType::GPS_FIX_TYPE_2D_FIX,
        DeviceGpsFixType::Fix3D => GpsFixType::GPS_FIX_TYPE_3D_FIX,
        DeviceGpsFixType::DGps => GpsFixType::GPS_FIX_TYPE_DGPS,
        DeviceGpsFixType::RtkFloat => GpsFixType::GPS_FIX_TYPE_RTK_FLOAT,
        DeviceGpsFixType::RtkFixed => GpsFixType::GPS_FIX_TYPE_RTK_FIXED,
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    rate_hz: u32,
    last_send_us: u64,
}

impl StreamConfig {
    const fn new(rate_hz: u32) -> Self {
        Self {
            rate_hz,
            last_send_us: u64::MAX,
        }
    }

    fn should_send(&self, current_time_us: u64) -> bool {
        if self.rate_hz == 0 {
            return false;
        }

        let interval_us = 1_000_000 / self.rate_hz as u64;

        if self.last_send_us == u64::MAX {
            return true;
        }

        current_time_us.saturating_sub(self.last_send_us) >= interval_us
    }

    fn mark_sent(&mut self, timestamp_us: u64) {
        self.last_send_us = timestamp_us;
    }
}

pub struct TelemetryStreamer<V: VehicleType> {
    _system_id: u8,
    _component_id: u8,
    heartbeat: StreamConfig,
    attitude: StreamConfig,
    gps: StreamConfig,
    global_position: StreamConfig,
    sys_status: StreamConfig,
    battery_status: StreamConfig,
    mission_current: StreamConfig,
    _vehicle: PhantomData<V>,
}

impl<V: VehicleType> TelemetryStreamer<V> {
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            _system_id: system_id,
            _component_id: component_id,
            heartbeat: StreamConfig::new(1),
            attitude: StreamConfig::new(2),
            gps: StreamConfig::new(2),
            global_position: StreamConfig::new(2),
            sys_status: StreamConfig::new(1),
            battery_status: StreamConfig::new(2),
            mission_current: StreamConfig::new(1),
            _vehicle: PhantomData,
        }
    }

    pub fn update_rates(&mut self, sr_extra1: u32, sr_position: u32) {
        self.attitude.rate_hz = sr_extra1;
        self.sys_status.rate_hz = sr_extra1.min(1);
        self.gps.rate_hz = sr_position;
        self.global_position.rate_hz = sr_position;
    }

    pub fn update(
        &mut self,
        state: &SystemState,
        current_time_us: u64,
    ) -> heapless::Vec<MavMessage, 8> {
        let mut messages = heapless::Vec::new();

        if self.heartbeat.should_send(current_time_us) {
            if let Some(msg) = self.build_heartbeat(state) {
                let _ = messages.push(msg);
                self.heartbeat.mark_sent(current_time_us);
            }
        }

        if self.attitude.should_send(current_time_us) {
            if let Some(msg) = self.build_attitude(state) {
                let _ = messages.push(msg);
                self.attitude.mark_sent(current_time_us);
            }
        }

        if self.gps.should_send(current_time_us) {
            if let Some(msg) = self.build_gps(state) {
                let _ = messages.push(msg);
                self.gps.mark_sent(current_time_us);
            }
        }

        if self.global_position.should_send(current_time_us) {
            if let Some(msg) = self.build_global_position_int(state) {
                let _ = messages.push(msg);
                self.global_position.mark_sent(current_time_us);
            }
        }

        if self.sys_status.should_send(current_time_us) {
            if let Some(msg) = self.build_sys_status(state) {
                let _ = messages.push(msg);
                self.sys_status.mark_sent(current_time_us);
            }
        }

        if self.battery_status.should_send(current_time_us) {
            if let Some(msg) = self.build_battery_status(state) {
                let _ = messages.push(msg);
                self.battery_status.mark_sent(current_time_us);
            }
        }

        if self.mission_current.should_send(current_time_us) {
            let _ = messages.push(Self::build_mission_current());
            self.mission_current.mark_sent(current_time_us);
        }

        messages
    }

    fn build_heartbeat(&self, state: &SystemState) -> Option<MavMessage> {
        let mut base_mode = MavModeFlag::from_bits_truncate(state.mode.to_base_mode_flags());

        if state.is_armed() {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }

        Some(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: state.mode.to_custom_mode(),
            mavtype: crate::autopilot::vehicle::to_mav_type::<V>(),
            autopilot: crate::autopilot::vehicle::to_mav_autopilot::<V>(),
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        }))
    }

    fn build_attitude(&self, state: &SystemState) -> Option<MavMessage> {
        let corrected_yaw = normalize_yaw(state.attitude.yaw + state.compass_yaw_offset);

        Some(MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: (state.uptime_us / 1000) as u32,
            roll: state.attitude.roll,
            pitch: state.attitude.pitch,
            yaw: corrected_yaw,
            rollspeed: state.attitude.rollspeed,
            pitchspeed: state.attitude.pitchspeed,
            yawspeed: state.attitude.yawspeed,
        }))
    }

    fn build_gps(&self, state: &SystemState) -> Option<MavMessage> {
        let (lat, lon, alt, vel, cog, fix_type, satellites) = match state.gps_position {
            Some(pos) => (
                degrees_to_deg_e7(pos.latitude),
                degrees_to_deg_e7(pos.longitude),
                meters_to_mm(pos.altitude),
                mps_to_cms(pos.speed),
                pos.course_over_ground
                    .map(degrees_to_cdeg)
                    .unwrap_or(u16::MAX),
                convert_gps_fix_type(pos.fix_type),
                pos.satellites,
            ),
            None => (0, 0, 0, 0, 0, GpsFixType::GPS_FIX_TYPE_NO_FIX, 0),
        };

        Some(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: state.uptime_us,
            lat,
            lon,
            alt,
            eph: 9999,
            epv: 9999,
            vel,
            cog,
            fix_type,
            satellites_visible: satellites,
            alt_ellipsoid: alt,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
        }))
    }

    fn build_global_position_int(&self, state: &SystemState) -> Option<MavMessage> {
        let hdg = if state.attitude.healthy {
            let corrected_yaw = state.attitude.yaw + state.compass_yaw_offset;
            radians_to_cdeg(corrected_yaw)
        } else {
            state
                .gps_position
                .and_then(|pos| pos.course_over_ground)
                .map(degrees_to_cdeg)
                .unwrap_or(u16::MAX)
        };

        let (lat, lon, alt, vx, vy, vz) = match state.gps_position {
            Some(pos) => {
                let (vx, vy) = match pos.course_over_ground {
                    Some(cog) => {
                        let cog_rad = cog * core::f32::consts::PI / 180.0;
                        let vx = pos.speed * cog_rad.cos();
                        let vy = pos.speed * cog_rad.sin();
                        (mps_to_cms_i16(vx), mps_to_cms_i16(vy))
                    }
                    None => (0, 0),
                };

                (
                    degrees_to_deg_e7(pos.latitude),
                    degrees_to_deg_e7(pos.longitude),
                    meters_to_mm(pos.altitude),
                    vx,
                    vy,
                    0i16,
                )
            }
            None => (0, 0, 0, 0, 0, 0),
        };

        Some(MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            time_boot_ms: (state.uptime_us / 1000) as u32,
            lat,
            lon,
            alt,
            relative_alt: match (&state.home_position, &state.gps_position) {
                (Some(home), Some(gps)) => ((gps.altitude - home.altitude) * 1000.0) as i32,
                _ => 0,
            },
            vx,
            vy,
            vz,
            hdg,
        }))
    }

    fn build_sys_status(&self, state: &SystemState) -> Option<MavMessage> {
        let voltage_battery = (state.battery.voltage * 1000.0) as u16;
        let current_battery = (state.battery.current * 100.0) as i16;

        let sensors_present = MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO
            | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG
            | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS
            | MavSysStatusSensor::MAV_SYS_STATUS_AHRS
            | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY;

        let sensors_enabled = sensors_present;
        let sensors_health = sensors_present;

        Some(MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_present: sensors_present,
            onboard_control_sensors_enabled: sensors_enabled,
            onboard_control_sensors_health: sensors_health,
            load: (state.cpu_load * 10.0) as u16,
            voltage_battery,
            current_battery,
            battery_remaining: state.battery.remaining_percent as i8,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_enabled_extended: MavSysStatusSensorExtended::empty(),
            onboard_control_sensors_health_extended: MavSysStatusSensorExtended::empty(),
        }))
    }

    fn build_battery_status(&self, state: &SystemState) -> Option<MavMessage> {
        let pack_voltage_mv = (state.battery.voltage * 1000.0) as u16;

        let mut voltages = [u16::MAX; 10];
        voltages[0] = pack_voltage_mv;

        Some(MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA {
            id: 0,
            battery_function: MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL,
            mavtype: MavBatteryType::MAV_BATTERY_TYPE_LIPO,
            temperature: i16::MAX,
            voltages,
            current_battery: -1,
            current_consumed: -1,
            energy_consumed: -1,
            battery_remaining: state.battery.remaining_percent as i8,
            charge_state: MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_UNDEFINED,
            voltages_ext: [0; 4],
            mode: MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN,
            fault_bitmask: MavBatteryFault::empty(),
            time_remaining: 0,
        }))
    }

    pub fn build_mission_current() -> MavMessage {
        let (seq, core_state) =
            MISSION_SEQUENCER.with(|seq| (seq.current_nav_index(), seq.state()));

        let total = MISSION_STORAGE.with(|storage| storage.count());

        let mission_state = match core_state {
            MissionState::Idle => {
                if total > 0 {
                    mavlink::common::MissionState::MISSION_STATE_NOT_STARTED
                } else {
                    mavlink::common::MissionState::MISSION_STATE_NO_MISSION
                }
            }
            MissionState::Running => mavlink::common::MissionState::MISSION_STATE_ACTIVE,
            MissionState::Completed => mavlink::common::MissionState::MISSION_STATE_COMPLETE,
        };

        let mission_mode = match core_state {
            MissionState::Running => 1,
            _ => 0,
        };

        MavMessage::MISSION_CURRENT(MISSION_CURRENT_DATA {
            seq,
            total,
            mission_state,
            mission_mode,
            mission_id: 0,
            fence_id: 0,
            rally_points_id: 0,
        })
    }

    pub fn build_mission_item_reached(seq: u16) -> MavMessage {
        MavMessage::MISSION_ITEM_REACHED(mavlink::common::MISSION_ITEM_REACHED_DATA { seq })
    }

    pub fn get_rates(&self) -> (u32, u32, u32, u32, u32) {
        (
            self.heartbeat.rate_hz,
            self.attitude.rate_hz,
            self.gps.rate_hz,
            self.sys_status.rate_hz,
            self.battery_status.rate_hz,
        )
    }

    pub fn build_protocol_version() -> MavMessage {
        use super::command::CommandHandler;
        MavMessage::PROTOCOL_VERSION(CommandHandler::<V>::create_protocol_version_message())
    }

    pub fn build_autopilot_version() -> MavMessage {
        use super::command::CommandHandler;
        MavMessage::AUTOPILOT_VERSION(CommandHandler::<V>::create_autopilot_version_message())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autopilot::vehicle::GroundRover;
    use mavlink::common::MavAutopilot;

    #[test]
    fn test_stream_config_should_send() {
        let mut config = StreamConfig::new(10);
        assert!(config.should_send(0));
        config.mark_sent(0);
        assert!(!config.should_send(50_000));
        assert!(config.should_send(100_000));
    }

    #[test]
    fn test_stream_config_disabled() {
        let config = StreamConfig::new(0);
        assert!(!config.should_send(0));
        assert!(!config.should_send(1_000_000));
    }

    #[test]
    fn test_telemetry_streamer_creation() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let (hb, att, gps, sys, bat) = streamer.get_rates();
        assert_eq!(hb, 1);
        assert_eq!(att, 2);
        assert_eq!(gps, 2);
        assert_eq!(sys, 1);
        assert_eq!(bat, 2);
    }

    #[test]
    fn test_update_rates() {
        let mut streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        streamer.update_rates(20, 10);
        let (hb, att, gps, sys, bat) = streamer.get_rates();
        assert_eq!(hb, 1);
        assert_eq!(att, 20);
        assert_eq!(gps, 10);
        assert_eq!(sys, 1);
        assert_eq!(bat, 2);
    }

    #[test]
    fn test_build_heartbeat() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new();

        let msg = streamer.build_heartbeat(&state).unwrap();
        if let MavMessage::HEARTBEAT(data) = msg {
            assert_eq!(
                data.mavtype,
                crate::autopilot::vehicle::to_mav_type::<GroundRover>()
            );
            assert_eq!(data.autopilot, MavAutopilot::MAV_AUTOPILOT_GENERIC);
            assert!(!data
                .base_mode
                .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED));
        } else {
            panic!("Expected HEARTBEAT message");
        }
    }

    #[test]
    fn test_build_attitude() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();
        state.uptime_us = 5_000_000;

        let msg = streamer.build_attitude(&state).unwrap();
        if let MavMessage::ATTITUDE(data) = msg {
            assert_eq!(data.time_boot_ms, 5000);
            assert_eq!(data.roll, 0.0);
            assert_eq!(data.pitch, 0.0);
            assert_eq!(data.yaw, 0.0);
        } else {
            panic!("Expected ATTITUDE message");
        }
    }

    #[test]
    fn test_build_gps_with_no_fix() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new();

        let msg = streamer.build_gps(&state).unwrap();
        if let MavMessage::GPS_RAW_INT(data) = msg {
            assert_eq!(data.fix_type, GpsFixType::GPS_FIX_TYPE_NO_FIX);
            assert_eq!(data.lat, 0);
            assert_eq!(data.lon, 0);
        } else {
            panic!("Expected GPS_RAW_INT message");
        }
    }

    #[test]
    fn test_build_gps_with_valid_position() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 10.5,
            course_over_ground: Some(84.4),
            fix_type: DGpsFixType::Fix3D,
            satellites: 8,
        };
        state.update_gps(position, 1000000);

        let msg = streamer.build_gps(&state).unwrap();
        if let MavMessage::GPS_RAW_INT(data) = msg {
            assert_eq!(data.fix_type, GpsFixType::GPS_FIX_TYPE_3D_FIX);
            assert!((data.lat - 481173000).abs() < 100);
            assert!((data.lon - 115166000).abs() < 100);
            assert_eq!(data.alt, 545400);
            assert_eq!(data.vel, 1050);
            assert_eq!(data.cog, 8440);
            assert_eq!(data.satellites_visible, 8);
        } else {
            panic!("Expected GPS_RAW_INT message");
        }
    }

    #[test]
    fn test_build_sys_status() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();
        state.battery.voltage = 12.5;
        state.battery.current = 2.3;
        state.cpu_load = 45.5;

        let msg = streamer.build_sys_status(&state).unwrap();
        if let MavMessage::SYS_STATUS(data) = msg {
            assert_eq!(data.voltage_battery, 12500);
            assert_eq!(data.current_battery, 230);
            assert_eq!(data.load, 455);
        } else {
            panic!("Expected SYS_STATUS message");
        }
    }

    #[test]
    fn test_build_battery_status() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();
        state.battery.voltage = 12.5;
        state.battery.remaining_percent = 97;

        let msg = streamer.build_battery_status(&state).unwrap();
        if let MavMessage::BATTERY_STATUS(data) = msg {
            assert_eq!(data.id, 0);
            assert_eq!(data.voltages[0], 12500);
            assert_eq!(data.battery_remaining, 97);
        } else {
            panic!("Expected BATTERY_STATUS message");
        }
    }

    #[test]
    fn test_update_single_message() {
        let mut streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new();

        let messages = streamer.update(&state, 0);
        assert_eq!(messages.len(), 7);

        let messages = streamer.update(&state, 500_000);
        assert_eq!(messages.len(), 4);
    }

    #[test]
    fn test_convert_gps_fix_type() {
        assert_eq!(
            convert_gps_fix_type(DeviceGpsFixType::NoFix),
            GpsFixType::GPS_FIX_TYPE_NO_FIX
        );
        assert_eq!(
            convert_gps_fix_type(DeviceGpsFixType::Fix3D),
            GpsFixType::GPS_FIX_TYPE_3D_FIX
        );
    }

    #[test]
    fn test_degrees_to_deg_e7_positive() {
        assert_eq!(degrees_to_deg_e7(90.0), 900000000);
        assert_eq!(degrees_to_deg_e7(0.0), 0);
    }

    #[test]
    fn test_meters_to_mm_typical() {
        assert_eq!(meters_to_mm(545.4), 545400);
        assert_eq!(meters_to_mm(0.0), 0);
    }

    #[test]
    fn test_mps_to_cms_typical() {
        assert_eq!(mps_to_cms(1.4), 140);
        assert_eq!(mps_to_cms(0.0), 0);
    }

    #[test]
    fn test_radians_to_cdeg_typical() {
        use core::f32::consts::PI;
        assert_eq!(radians_to_cdeg(0.0), 0);
        assert_eq!(radians_to_cdeg(PI / 2.0), 9000);
        assert_eq!(radians_to_cdeg(PI), 18000);
    }

    #[test]
    fn test_radians_to_cdeg_negative() {
        use core::f32::consts::PI;
        assert_eq!(radians_to_cdeg(-PI / 2.0), 27000);
    }

    #[test]
    fn test_build_mission_current_idle() {
        MISSION_STORAGE.with_mut(|storage| storage.clear());

        let msg = TelemetryStreamer::<GroundRover>::build_mission_current();
        if let MavMessage::MISSION_CURRENT(data) = msg {
            assert_eq!(data.seq, 0);
            assert_eq!(data.total, 0);
            assert_eq!(
                data.mission_state,
                mavlink::common::MissionState::MISSION_STATE_NO_MISSION
            );
        } else {
            panic!("Expected MISSION_CURRENT message");
        }
    }

    #[test]
    fn test_build_mission_item_reached() {
        let msg = TelemetryStreamer::<GroundRover>::build_mission_item_reached(3);
        if let MavMessage::MISSION_ITEM_REACHED(data) = msg {
            assert_eq!(data.seq, 3);
        } else {
            panic!("Expected MISSION_ITEM_REACHED message");
        }
    }

    #[test]
    fn test_stream_config_1hz() {
        let mut config = StreamConfig::new(1);
        config.mark_sent(0);
        assert!(!config.should_send(500_000));
        assert!(config.should_send(1_000_000));
    }

    #[test]
    fn test_degrees_to_deg_e7_negative() {
        // f32 `-34.6037` is actually -34.603698730468750 (7 significant digits)
        // With f64 intermediate: (-34.603698730468750_f64 * 1e7) = -346036987
        assert_eq!(degrees_to_deg_e7(-34.6037), -346036987);
    }

    #[test]
    fn test_meters_to_mm_negative() {
        assert_eq!(meters_to_mm(-100.0), -100000);
    }

    #[test]
    fn test_meters_to_mm_high_altitude() {
        assert_eq!(meters_to_mm(400.0), 400000);
    }

    #[test]
    fn test_mps_to_cms_boundary() {
        let result = mps_to_cms(655.35);
        assert!(result >= 65534, "Expected >= 65534, got {}", result);
        assert_eq!(mps_to_cms(1000.0), 65535);
        assert_eq!(mps_to_cms(-10.0), 0);
    }

    #[test]
    fn test_mps_to_cms_i16_typical() {
        assert_eq!(mps_to_cms_i16(10.0), 1000);
        assert_eq!(mps_to_cms_i16(-10.0), -1000);
        assert_eq!(mps_to_cms_i16(0.0), 0);
    }

    #[test]
    fn test_mps_to_cms_i16_boundary() {
        assert_eq!(mps_to_cms_i16(327.67), 32767);
        assert_eq!(mps_to_cms_i16(500.0), 32767);
        assert_eq!(mps_to_cms_i16(-327.68), -32768);
        assert_eq!(mps_to_cms_i16(-500.0), -32768);
    }

    #[test]
    fn test_degrees_to_cdeg_typical() {
        assert_eq!(degrees_to_cdeg(0.0), 0);
        assert_eq!(degrees_to_cdeg(90.0), 9000);
        assert_eq!(degrees_to_cdeg(180.0), 18000);
        assert_eq!(degrees_to_cdeg(270.0), 27000);
    }

    #[test]
    fn test_degrees_to_cdeg_boundary() {
        assert_eq!(degrees_to_cdeg(359.99), 35999);
        assert_eq!(degrees_to_cdeg(360.0), 35999);
        assert_eq!(degrees_to_cdeg(-10.0), 0);
    }

    #[test]
    fn test_build_gps_with_no_cog() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 0.2,
            course_over_ground: None,
            fix_type: DGpsFixType::Fix3D,
            satellites: 8,
        };
        state.update_gps(position, 1000000);

        let msg = streamer.build_gps(&state).unwrap();
        if let MavMessage::GPS_RAW_INT(data) = msg {
            assert_eq!(data.cog, u16::MAX);
        } else {
            panic!("Expected GPS_RAW_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_no_gps() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new();

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.lat, 0);
            assert_eq!(data.lon, 0);
            assert_eq!(data.alt, 0);
            assert_eq!(data.vx, 0);
            assert_eq!(data.vy, 0);
            assert_eq!(data.vz, 0);
            assert_eq!(data.hdg, u16::MAX);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_with_velocity() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 10.0,
            course_over_ground: Some(90.0),
            fix_type: DGpsFixType::Fix3D,
            satellites: 8,
        };
        state.update_gps(position, 1000000);

        state.update_attitude_direct(
            0.0,
            0.0,
            core::f32::consts::PI / 2.0,
            0.0,
            0.0,
            0.0,
            1000000,
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert!((data.lat - 481173000).abs() < 100);
            assert!((data.lon - 115166000).abs() < 100);
            assert_eq!(data.alt, 545400);
            assert!(data.vx.abs() < 10);
            assert!((data.vy - 1000).abs() < 10);
            assert_eq!(data.vz, 0);
            assert_eq!(data.hdg, 9000);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_heading_north() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 10.0,
            course_over_ground: Some(0.0),
            fix_type: DGpsFixType::Fix3D,
            satellites: 6,
        };
        state.update_gps(position, 1000000);

        state.update_attitude_direct(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000);

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert!((data.vx - 1000).abs() < 10);
            assert!(data.vy.abs() < 10);
            assert_eq!(data.hdg, 0);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_no_cog_no_ahrs() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 0.3,
            course_over_ground: None,
            fix_type: DGpsFixType::Fix2D,
            satellites: 4,
        };
        state.update_gps(position, 1000000);

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.vx, 0);
            assert_eq!(data.vy, 0);
            assert_eq!(data.hdg, u16::MAX);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_ahrs_fallback_to_gps_cog() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 5.0,
            course_over_ground: Some(45.0),
            fix_type: DGpsFixType::Fix3D,
            satellites: 6,
        };
        state.update_gps(position, 1000000);

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.hdg, 4500);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_ahrs_negative_yaw() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 5.0,
            course_over_ground: Some(0.0),
            fix_type: DGpsFixType::Fix3D,
            satellites: 6,
        };
        state.update_gps(position, 1000000);

        state.update_attitude_direct(
            0.0,
            0.0,
            -core::f32::consts::PI / 2.0,
            0.0,
            0.0,
            0.0,
            1000000,
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.hdg, 27000);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_relative_alt_with_home_and_gps() {
        use crate::autopilot::state::HomePosition;
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        state.home_position = Some(HomePosition::new(35.6812, 139.7671, 40.0));
        state.update_gps(
            GpsPosition {
                latitude: 35.6812,
                longitude: 139.7671,
                altitude: 55.0,
                speed: 0.0,
                course_over_ground: None,
                fix_type: DGpsFixType::Fix3D,
                satellites: 10,
            },
            1000000,
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // 55.0 - 40.0 = 15.0m = 15000mm
            assert_eq!(data.relative_alt, 15000);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_relative_alt_negative_difference() {
        use crate::autopilot::state::HomePosition;
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        state.home_position = Some(HomePosition::new(35.6812, 139.7671, 100.0));
        state.update_gps(
            GpsPosition {
                latitude: 35.6812,
                longitude: 139.7671,
                altitude: 80.0,
                speed: 0.0,
                course_over_ground: None,
                fix_type: DGpsFixType::Fix3D,
                satellites: 10,
            },
            1000000,
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // 80.0 - 100.0 = -20.0m = -20000mm
            assert_eq!(data.relative_alt, -20000);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_relative_alt_zero_when_no_home() {
        use crate::navigation::{GpsFixType as DGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        state.update_gps(
            GpsPosition {
                latitude: 35.6812,
                longitude: 139.7671,
                altitude: 55.0,
                speed: 0.0,
                course_over_ground: None,
                fix_type: DGpsFixType::Fix3D,
                satellites: 10,
            },
            1000000,
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.relative_alt, 0);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_relative_alt_zero_when_no_gps() {
        use crate::autopilot::state::HomePosition;

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        state.home_position = Some(HomePosition::new(35.6812, 139.7671, 40.0));

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.relative_alt, 0);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }
}
