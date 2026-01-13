//! MAVLink Telemetry Streaming
//!
//! Implements periodic telemetry message streaming to Ground Control Stations.
//!
//! # Supported Messages
//!
//! - **HEARTBEAT**: System status (1Hz)
//! - **ATTITUDE**: Roll, pitch, yaw (2Hz default, configurable via SR_EXTRA1)
//! - **GPS_RAW_INT**: GPS position (2Hz default, configurable via SR_POSITION)
//! - **GLOBAL_POSITION_INT**: Position (2Hz default, configurable via SR_POSITION)
//! - **SYS_STATUS**: System health (1Hz, configurable via SR_EXTRA1)
//! - **BATTERY_STATUS**: Battery voltage and status (2Hz, hardcoded)
//!
//! # Stream Rate Control
//!
//! Stream rates are controlled by SR_* parameters:
//! - SR_EXTRA1: ATTITUDE, SYS_STATUS rate (Hz)
//! - SR_POSITION: GPS_RAW_INT rate (Hz)
//! - BATTERY_STATUS: Hardcoded at 2Hz (future: configurable via SR0_EXTRA1)

use crate::communication::mavlink::state::SystemState;
use crate::communication::mavlink::vehicle::VehicleType;
use crate::devices::gps::GpsFixType as DeviceGpsFixType;
use core::marker::PhantomData;
use mavlink::common::{
    GpsFixType, MavBatteryChargeState, MavBatteryFault, MavBatteryFunction, MavBatteryMode,
    MavBatteryType, MavMessage, MavModeFlag, MavState, MavSysStatusSensor,
    MavSysStatusSensorExtended, ATTITUDE_DATA, BATTERY_STATUS_DATA, GLOBAL_POSITION_INT_DATA,
    GPS_RAW_INT_DATA, HEARTBEAT_DATA, SYS_STATUS_DATA,
};
#[allow(unused_imports)]
use micromath::F32Ext;

// Unit conversion helper functions for GPS telemetry

/// Convert degrees to degE7 (degrees * 1e7) for MAVLink lat/lon
fn degrees_to_deg_e7(degrees: f32) -> i32 {
    (degrees * 1e7) as i32
}

/// Convert meters to millimeters for MAVLink altitude
fn meters_to_mm(meters: f32) -> i32 {
    (meters * 1000.0) as i32
}

/// Convert m/s to cm/s for MAVLink velocity
fn mps_to_cms(mps: f32) -> u16 {
    (mps * 100.0).clamp(0.0, u16::MAX as f32) as u16
}

/// Convert m/s to cm/s (signed) for MAVLink velocity components
fn mps_to_cms_i16(mps: f32) -> i16 {
    (mps * 100.0).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// Convert degrees to centidegrees for MAVLink COG/heading
fn degrees_to_cdeg(degrees: f32) -> u16 {
    (degrees * 100.0).clamp(0.0, 35999.0) as u16
}

/// Convert radians to centidegrees for MAVLink heading
/// Normalizes negative angles to 0-360 range
fn radians_to_cdeg(radians: f32) -> u16 {
    let degrees = radians * 180.0 / core::f32::consts::PI;
    // Normalize to 0-360 range
    let normalized = if degrees < 0.0 {
        degrees + 360.0
    } else if degrees >= 360.0 {
        degrees - 360.0
    } else {
        degrees
    };
    (normalized * 100.0).clamp(0.0, 35999.0) as u16
}

/// Convert device GPS fix type to MAVLink GPS fix type
fn convert_gps_fix_type(fix_type: DeviceGpsFixType) -> GpsFixType {
    match fix_type {
        DeviceGpsFixType::NoFix => GpsFixType::GPS_FIX_TYPE_NO_FIX,
        DeviceGpsFixType::Fix2D => GpsFixType::GPS_FIX_TYPE_2D_FIX,
        DeviceGpsFixType::Fix3D => GpsFixType::GPS_FIX_TYPE_3D_FIX,
    }
}

/// Stream configuration for a single telemetry message type
#[derive(Debug, Clone, Copy)]
struct StreamConfig {
    /// Target rate in Hz (0 = disabled)
    rate_hz: u32,
    /// Last send timestamp in microseconds
    last_send_us: u64,
}

impl StreamConfig {
    /// Create new stream config with specified rate
    const fn new(rate_hz: u32) -> Self {
        Self {
            rate_hz,
            last_send_us: u64::MAX, // Sentinel value for "never sent"
        }
    }

    /// Check if it's time to send based on rate and elapsed time
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Returns true if enough time has elapsed since last send based on rate.
    fn should_send(&self, current_time_us: u64) -> bool {
        if self.rate_hz == 0 {
            return false; // Disabled
        }

        let interval_us = 1_000_000 / self.rate_hz as u64;

        // If never sent before (sentinel value), always send
        if self.last_send_us == u64::MAX {
            return true;
        }

        // Check if enough time has elapsed
        current_time_us.saturating_sub(self.last_send_us) >= interval_us
    }

    /// Update last send timestamp
    fn mark_sent(&mut self, timestamp_us: u64) {
        self.last_send_us = timestamp_us;
    }
}

pub struct TelemetryStreamer<V: VehicleType> {
    #[allow(dead_code)]
    system_id: u8,
    #[allow(dead_code)]
    component_id: u8,
    heartbeat: StreamConfig,
    attitude: StreamConfig,
    gps: StreamConfig,
    global_position: StreamConfig,
    sys_status: StreamConfig,
    battery_status: StreamConfig,
    _vehicle: PhantomData<V>,
}

impl<V: VehicleType> TelemetryStreamer<V> {
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            system_id,
            component_id,
            heartbeat: StreamConfig::new(1),
            attitude: StreamConfig::new(2),
            gps: StreamConfig::new(2),
            global_position: StreamConfig::new(2),
            sys_status: StreamConfig::new(1),
            battery_status: StreamConfig::new(2),
            _vehicle: PhantomData,
        }
    }

    /// Update stream rates from parameters
    ///
    /// # Arguments
    ///
    /// * `sr_extra1` - Rate for ATTITUDE and SYS_STATUS (Hz)
    /// * `sr_position` - Rate for GPS_RAW_INT and GLOBAL_POSITION_INT (Hz)
    pub fn update_rates(&mut self, sr_extra1: u32, sr_position: u32) {
        self.attitude.rate_hz = sr_extra1;
        self.sys_status.rate_hz = sr_extra1.min(1); // SYS_STATUS max 1Hz
        self.gps.rate_hz = sr_position;
        self.global_position.rate_hz = sr_position;
    }

    /// Generate telemetry messages that are due to be sent
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Vector of messages to send (max 6: HEARTBEAT, ATTITUDE, GPS_RAW_INT,
    /// GLOBAL_POSITION_INT, SYS_STATUS, BATTERY_STATUS)
    pub fn update(
        &mut self,
        state: &SystemState,
        current_time_us: u64,
    ) -> heapless::Vec<MavMessage, 6> {
        let mut messages = heapless::Vec::new();

        // HEARTBEAT (always at 1Hz)
        if self.heartbeat.should_send(current_time_us) {
            if let Some(msg) = self.build_heartbeat(state) {
                let _ = messages.push(msg);
                self.heartbeat.mark_sent(current_time_us);
            }
        }

        // ATTITUDE (SR_EXTRA1 rate)
        if self.attitude.should_send(current_time_us) {
            if let Some(msg) = self.build_attitude(state) {
                let _ = messages.push(msg);
                self.attitude.mark_sent(current_time_us);
            }
        }

        // GPS_RAW_INT (SR_POSITION rate)
        if self.gps.should_send(current_time_us) {
            if let Some(msg) = self.build_gps(state) {
                let _ = messages.push(msg);
                self.gps.mark_sent(current_time_us);
            }
        }

        // GLOBAL_POSITION_INT (SR_POSITION rate)
        if self.global_position.should_send(current_time_us) {
            if let Some(msg) = self.build_global_position_int(state) {
                let _ = messages.push(msg);
                self.global_position.mark_sent(current_time_us);
            }
        }

        // SYS_STATUS (SR_EXTRA1 rate, max 1Hz)
        if self.sys_status.should_send(current_time_us) {
            if let Some(msg) = self.build_sys_status(state) {
                let _ = messages.push(msg);
                self.sys_status.mark_sent(current_time_us);
            }
        }

        // BATTERY_STATUS (hardcoded 2Hz)
        if self.battery_status.should_send(current_time_us) {
            if let Some(msg) = self.build_battery_status(state) {
                let _ = messages.push(msg);
                self.battery_status.mark_sent(current_time_us);
            }
        }

        messages
    }

    /// Build HEARTBEAT message
    fn build_heartbeat(&self, state: &SystemState) -> Option<MavMessage> {
        // Get base mode flags from flight mode (includes CUSTOM_MODE_ENABLED)
        let mut base_mode = MavModeFlag::from_bits_truncate(state.mode.to_base_mode_flags());

        // Add armed flag if armed
        if state.is_armed() {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }

        Some(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: state.mode.to_custom_mode(),
            mavtype: V::mav_type(),
            autopilot: V::autopilot_type(),
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        }))
    }

    /// Build ATTITUDE message
    ///
    /// Returns roll, pitch, yaw and angular rates from AHRS.
    /// All angles are in radians, angular rates in rad/s.
    fn build_attitude(&self, state: &SystemState) -> Option<MavMessage> {
        Some(MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: (state.uptime_us / 1000) as u32,
            roll: state.attitude.roll,
            pitch: state.attitude.pitch,
            yaw: state.attitude.yaw,
            rollspeed: state.attitude.rollspeed,
            pitchspeed: state.attitude.pitchspeed,
            yawspeed: state.attitude.yawspeed,
        }))
    }

    /// Build GPS_RAW_INT message
    ///
    /// Builds GPS_RAW_INT message with real GPS data from SystemState.
    /// Returns zeros for position if no GPS fix is available.
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
            eph: 9999, // HDOP not available (future enhancement)
            epv: 9999, // VDOP not available (future enhancement)
            vel,
            cog,
            fix_type,
            satellites_visible: satellites,
            alt_ellipsoid: alt, // Same as MSL alt for now
            h_acc: 0,           // Horizontal accuracy not available
            v_acc: 0,           // Vertical accuracy not available
            vel_acc: 0,         // Velocity accuracy not available
            hdg_acc: 0,         // Heading accuracy not available
            yaw: 0,             // Yaw not available from GPS
        }))
    }

    /// Build GLOBAL_POSITION_INT message
    ///
    /// Builds GLOBAL_POSITION_INT message with position and velocity data.
    /// Velocity components (vx, vy) are computed from speed and COG.
    /// Heading (hdg) is from AHRS yaw when available, otherwise from GPS COG.
    /// Returns zeros if no GPS fix is available.
    fn build_global_position_int(&self, state: &SystemState) -> Option<MavMessage> {
        // Get heading from AHRS if healthy, otherwise fall back to GPS COG
        let hdg = if state.attitude.healthy {
            radians_to_cdeg(state.attitude.yaw)
        } else {
            // Fall back to GPS COG if AHRS is not available
            state
                .gps_position
                .and_then(|pos| pos.course_over_ground)
                .map(degrees_to_cdeg)
                .unwrap_or(u16::MAX)
        };

        let (lat, lon, alt, vx, vy, vz) = match state.gps_position {
            Some(pos) => {
                // Compute velocity components from speed and COG
                let (vx, vy) = match pos.course_over_ground {
                    Some(cog) => {
                        // Convert COG to radians for sin/cos
                        let cog_rad = cog * core::f32::consts::PI / 180.0;
                        // vx = North velocity, vy = East velocity
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
                    0i16, // vz (vertical rate) not available from GPS
                )
            }
            None => (0, 0, 0, 0, 0, 0),
        };

        Some(MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            time_boot_ms: (state.uptime_us / 1000) as u32,
            lat,
            lon,
            alt,
            relative_alt: 0, // Home position not implemented
            vx,
            vy,
            vz,
            hdg,
        }))
    }

    /// Build SYS_STATUS message
    fn build_sys_status(&self, state: &SystemState) -> Option<MavMessage> {
        // Convert voltage to millivolts
        let voltage_battery = (state.battery.voltage * 1000.0) as u16;
        // Convert current to 10mA units (centi-amps)
        let current_battery = (state.battery.current * 100.0) as i16;

        Some(MavMessage::SYS_STATUS(SYS_STATUS_DATA {
            onboard_control_sensors_present: MavSysStatusSensor::empty(),
            onboard_control_sensors_enabled: MavSysStatusSensor::empty(),
            onboard_control_sensors_health: MavSysStatusSensor::empty(),
            load: (state.cpu_load * 10.0) as u16, // 0.1% units
            voltage_battery,
            current_battery,
            battery_remaining: state.battery.remaining_percent as i8, // Estimated from voltage
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            onboard_control_sensors_present_extended: MavSysStatusSensorExtended::empty(), // MAVLink v2 extension
            onboard_control_sensors_enabled_extended: MavSysStatusSensorExtended::empty(), // MAVLink v2 extension
            onboard_control_sensors_health_extended: MavSysStatusSensorExtended::empty(), // MAVLink v2 extension
        }))
    }

    /// Build BATTERY_STATUS message
    ///
    /// Implements MAVLink BATTERY_STATUS (#147) message for ground control station
    /// battery monitoring. Provides voltage and estimated remaining percentage with
    /// placeholder values for unavailable fields (current, temperature, per-cell voltages).
    ///
    /// # Message Field Population
    ///
    /// - `id`: 0 (first battery)
    /// - `battery_function`: MAV_BATTERY_FUNCTION_ALL (all uses)
    /// - `type_`: MAV_BATTERY_TYPE_LIPO (LiPo battery)
    /// - `temperature`: INT16_MAX (unknown, no temperature sensor)
    /// - `voltages[0]`: Pack voltage in millivolts
    /// - `voltages[1..9]`: UINT16_MAX (unknown, no per-cell monitoring)
    /// - `current_battery`: -1 (unknown, no current sensor)
    /// - `current_consumed`: -1 (unknown, no current integration)
    /// - `energy_consumed`: -1 (unknown, no energy integration)
    /// - `battery_remaining`: Estimated percentage (0-100) based on voltage
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state containing battery voltage and remaining percentage
    ///
    /// # Returns
    ///
    /// MAVLink BATTERY_STATUS message
    fn build_battery_status(&self, state: &SystemState) -> Option<MavMessage> {
        // Convert voltage to millivolts for MAVLink message
        let pack_voltage_mv = (state.battery.voltage * 1000.0) as u16;

        // Initialize voltages array with UINT16_MAX (unknown/unavailable)
        let mut voltages = [u16::MAX; 10];
        voltages[0] = pack_voltage_mv; // Only pack voltage available

        Some(MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA {
            id: 0, // First battery
            battery_function: MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL,
            mavtype: MavBatteryType::MAV_BATTERY_TYPE_LIPO,
            temperature: i16::MAX, // Unknown (no temperature sensor)
            voltages,
            current_battery: -1,  // Unknown (no current sensor)
            current_consumed: -1, // Unknown (no current integration)
            energy_consumed: -1,  // Unknown (no energy integration)
            battery_remaining: state.battery.remaining_percent as i8, // Estimated from voltage
            charge_state: MavBatteryChargeState::MAV_BATTERY_CHARGE_STATE_UNDEFINED, // MAVLink v2 extension: unknown
            voltages_ext: [0; 4], // MAVLink v2 extension: extended cell voltages
            mode: MavBatteryMode::MAV_BATTERY_MODE_UNKNOWN, // MAVLink v2 extension: unknown
            fault_bitmask: MavBatteryFault::empty(), // MAVLink v2 extension: no faults
            time_remaining: 0,    // MAVLink v2 extension: unknown
        }))
    }

    /// Get current stream rates for debugging
    pub fn get_rates(&self) -> (u32, u32, u32, u32, u32) {
        (
            self.heartbeat.rate_hz,
            self.attitude.rate_hz,
            self.gps.rate_hz,
            self.sys_status.rate_hz,
            self.battery_status.rate_hz,
        )
    }

    /// Build PROTOCOL_VERSION message
    ///
    /// This is sent once in response to MAV_CMD_REQUEST_MESSAGE with param1=300.
    pub fn build_protocol_version() -> MavMessage {
        use crate::communication::mavlink::handlers::command::CommandHandler;
        MavMessage::PROTOCOL_VERSION(CommandHandler::<V>::create_protocol_version_message())
    }

    /// Build AUTOPILOT_VERSION message
    ///
    /// This is sent once in response to:
    /// - MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (command 520)
    /// - MAV_CMD_REQUEST_MESSAGE with param1=148
    pub fn build_autopilot_version() -> MavMessage {
        use crate::communication::mavlink::handlers::command::CommandHandler;
        MavMessage::AUTOPILOT_VERSION(CommandHandler::<V>::create_autopilot_version_message())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::vehicle::GroundRover;
    use mavlink::common::MavAutopilot;

    #[test]
    fn test_stream_config_should_send() {
        let mut config = StreamConfig::new(10); // 10Hz = 100ms interval

        // First call should always return true (last_send_us = 0)
        assert!(config.should_send(0));

        // Mark as sent at t=0
        config.mark_sent(0);

        // Before interval elapsed: should not send
        assert!(!config.should_send(50_000)); // 50ms < 100ms

        // After interval elapsed: should send
        assert!(config.should_send(100_000)); // 100ms >= 100ms
        assert!(config.should_send(150_000)); // 150ms >= 100ms
    }

    #[test]
    fn test_stream_config_disabled() {
        let config = StreamConfig::new(0); // 0Hz = disabled

        // Should never send when disabled
        assert!(!config.should_send(0));
        assert!(!config.should_send(1_000_000));
        assert!(!config.should_send(10_000_000));
    }

    #[test]
    fn test_stream_config_1hz() {
        let mut config = StreamConfig::new(1); // 1Hz = 1s interval
        config.mark_sent(0);

        assert!(!config.should_send(500_000)); // 0.5s < 1s
        assert!(config.should_send(1_000_000)); // 1s >= 1s
    }

    #[test]
    fn test_telemetry_streamer_creation() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let (hb, att, gps, sys, bat) = streamer.get_rates();

        assert_eq!(hb, 1); // HEARTBEAT always 1Hz
        assert_eq!(att, 2); // ATTITUDE default 2Hz
        assert_eq!(gps, 2); // GPS default 2Hz
        assert_eq!(sys, 1); // SYS_STATUS default 1Hz
        assert_eq!(bat, 2); // BATTERY_STATUS default 2Hz
    }

    #[test]
    fn test_update_rates() {
        let mut streamer = TelemetryStreamer::<GroundRover>::new(1, 1);

        streamer.update_rates(20, 10);
        let (hb, att, gps, sys, bat) = streamer.get_rates();

        assert_eq!(hb, 1); // HEARTBEAT always 1Hz (not changed)
        assert_eq!(att, 20); // ATTITUDE updated to 20Hz
        assert_eq!(gps, 10); // GPS updated to 10Hz
        assert_eq!(sys, 1); // SYS_STATUS clamped to 1Hz max
        assert_eq!(bat, 2); // BATTERY_STATUS hardcoded 2Hz (not changed)
    }

    #[test]
    fn test_build_heartbeat() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Test unarmed
        let msg = streamer.build_heartbeat(&state).unwrap();
        if let MavMessage::HEARTBEAT(data) = msg {
            assert_eq!(data.mavtype, GroundRover::mav_type());
            assert_eq!(data.autopilot, MavAutopilot::MAV_AUTOPILOT_GENERIC);
            assert!(!data
                .base_mode
                .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED));
        } else {
            panic!("Expected HEARTBEAT message");
        }

        // Test armed
        state.arm().unwrap();
        let msg = streamer.build_heartbeat(&state).unwrap();
        if let MavMessage::HEARTBEAT(data) = msg {
            assert!(data
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
        state.uptime_us = 5_000_000; // 5 seconds

        // Test with default (zero) values
        let msg = streamer.build_attitude(&state).unwrap();
        if let MavMessage::ATTITUDE(data) = msg {
            assert_eq!(data.time_boot_ms, 5000); // 5000ms
            assert_eq!(data.roll, 0.0);
            assert_eq!(data.pitch, 0.0);
            assert_eq!(data.yaw, 0.0);
        } else {
            panic!("Expected ATTITUDE message");
        }

        // Test with real AHRS values
        state.update_attitude_direct(
            0.1,       // roll (rad)
            0.2,       // pitch (rad)
            1.57,      // yaw (rad) ~90 degrees
            0.01,      // rollspeed (rad/s)
            0.02,      // pitchspeed (rad/s)
            0.03,      // yawspeed (rad/s)
            5_000_000, // timestamp
        );

        let msg = streamer.build_attitude(&state).unwrap();
        if let MavMessage::ATTITUDE(data) = msg {
            assert_eq!(data.time_boot_ms, 5000);
            assert!((data.roll - 0.1).abs() < 0.001);
            assert!((data.pitch - 0.2).abs() < 0.001);
            assert!((data.yaw - 1.57).abs() < 0.001);
            assert!((data.rollspeed - 0.01).abs() < 0.001);
            assert!((data.pitchspeed - 0.02).abs() < 0.001);
            assert!((data.yawspeed - 0.03).abs() < 0.001);
        } else {
            panic!("Expected ATTITUDE message");
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
            assert_eq!(data.voltage_battery, 12500); // 12.5V = 12500mV
            assert_eq!(data.current_battery, 230); // 2.3A = 230 centiamps
            assert_eq!(data.load, 455); // 45.5% = 455 in 0.1% units
        } else {
            panic!("Expected SYS_STATUS message");
        }
    }

    #[test]
    fn test_build_battery_status() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();
        state.battery.voltage = 12.5;
        state.battery.remaining_percent = 97; // 12.5V is ~97% for 3S LiPo

        let msg = streamer.build_battery_status(&state).unwrap();
        if let MavMessage::BATTERY_STATUS(data) = msg {
            assert_eq!(data.id, 0); // First battery
            assert_eq!(
                data.battery_function,
                MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL
            );
            assert_eq!(data.mavtype, MavBatteryType::MAV_BATTERY_TYPE_LIPO);
            assert_eq!(data.voltages[0], 12500); // 12.5V = 12500mV
            assert_eq!(data.voltages[1], u16::MAX); // Unknown
            assert_eq!(data.current_battery, -1); // Unknown
            assert_eq!(data.battery_remaining, 97); // Estimated percentage
        } else {
            panic!("Expected BATTERY_STATUS message");
        }
    }

    #[test]
    fn test_update_single_message() {
        let mut streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new();

        // At t=0, all messages should send (first time for all streams)
        let messages = streamer.update(&state, 0);
        assert_eq!(messages.len(), 6); // HEARTBEAT, ATTITUDE, GPS_RAW_INT, GLOBAL_POSITION_INT, SYS_STATUS, BATTERY_STATUS
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::HEARTBEAT(_))));

        // At t=500ms, ATTITUDE/GPS/GLOBAL_POSITION should send (2Hz = 500ms interval)
        let messages = streamer.update(&state, 500_000);
        assert_eq!(messages.len(), 4); // ATTITUDE, GPS_RAW_INT, GLOBAL_POSITION_INT, BATTERY_STATUS
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::ATTITUDE(_))));
    }

    #[test]
    fn test_update_multiple_messages() {
        let mut streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new();

        // At t=1s, HEARTBEAT, ATTITUDE, GPS, SYS_STATUS should all send
        let messages = streamer.update(&state, 1_000_000);
        assert!(messages.len() >= 2); // At least HEARTBEAT and SYS_STATUS (1Hz)

        // Verify HEARTBEAT is present
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::HEARTBEAT(_))));
    }

    #[test]
    fn test_stream_rate_control() {
        let mut streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        streamer.update_rates(10, 5); // 10Hz attitude, 5Hz GPS
        let state = SystemState::new();

        let mut heartbeat_count = 0;
        let mut attitude_count = 0;
        let mut gps_count = 0;

        // Simulate 1 second of updates at 50Hz (20ms intervals)
        for i in 0..50 {
            let time_us = i * 20_000; // 20ms intervals
            let messages = streamer.update(&state, time_us);

            for msg in messages.iter() {
                match msg {
                    MavMessage::HEARTBEAT(_) => heartbeat_count += 1,
                    MavMessage::ATTITUDE(_) => attitude_count += 1,
                    MavMessage::GPS_RAW_INT(_) => gps_count += 1,
                    _ => {}
                }
            }
        }

        // HEARTBEAT: ~1Hz = 1 message in 1 second
        assert!((1..=2).contains(&heartbeat_count));

        // ATTITUDE: ~10Hz = 10 messages in 1 second
        assert!((9..=11).contains(&attitude_count));

        // GPS: ~5Hz = 5 messages in 1 second
        assert!((4..=6).contains(&gps_count));
    }

    // Unit conversion tests

    #[test]
    fn test_degrees_to_deg_e7_positive() {
        // Standard positive latitude (allow small float precision error)
        let result = degrees_to_deg_e7(48.1173);
        assert!(
            (result - 481173000).abs() < 100,
            "Expected ~481173000, got {}",
            result
        );
        // Maximum latitude - exact values should work
        assert_eq!(degrees_to_deg_e7(90.0), 900000000);
        // Equator
        assert_eq!(degrees_to_deg_e7(0.0), 0);
    }

    #[test]
    fn test_degrees_to_deg_e7_negative() {
        // Southern hemisphere latitude (allow small float precision error)
        let result = degrees_to_deg_e7(-33.8688);
        assert!(
            (result + 338688000).abs() < 100,
            "Expected ~-338688000, got {}",
            result
        );
        // Maximum negative latitude
        assert_eq!(degrees_to_deg_e7(-90.0), -900000000);
    }

    #[test]
    fn test_degrees_to_deg_e7_longitude() {
        // Standard positive longitude (allow small float precision error)
        let result = degrees_to_deg_e7(11.5166);
        assert!(
            (result - 115166000).abs() < 100,
            "Expected ~115166000, got {}",
            result
        );
        // Maximum positive longitude
        assert_eq!(degrees_to_deg_e7(180.0), 1800000000);
        // Maximum negative longitude
        assert_eq!(degrees_to_deg_e7(-180.0), -1800000000);
    }

    #[test]
    fn test_meters_to_mm_typical() {
        // Typical altitude values
        assert_eq!(meters_to_mm(545.4), 545400);
        assert_eq!(meters_to_mm(0.0), 0);
        assert_eq!(meters_to_mm(100.5), 100500);
    }

    #[test]
    fn test_meters_to_mm_negative() {
        // Below sea level (Dead Sea area ~-430m)
        assert_eq!(meters_to_mm(-430.0), -430000);
    }

    #[test]
    fn test_meters_to_mm_high_altitude() {
        // Everest height
        assert_eq!(meters_to_mm(8848.86), 8848860);
        // Typical drone ceiling
        assert_eq!(meters_to_mm(400.0), 400000);
    }

    #[test]
    fn test_mps_to_cms_typical() {
        // Walking speed (~1.4 m/s)
        assert_eq!(mps_to_cms(1.4), 140);
        // Car speed (~30 m/s)
        assert_eq!(mps_to_cms(30.0), 3000);
        // Zero speed
        assert_eq!(mps_to_cms(0.0), 0);
    }

    #[test]
    fn test_mps_to_cms_boundary() {
        // Maximum representable speed (u16::MAX cm/s = 655.35 m/s)
        // Due to float precision, 655.35 * 100 may be 65534.xxx
        let result = mps_to_cms(655.35);
        assert!(result >= 65534, "Expected >= 65534, got {}", result);
        // Above maximum should clamp to u16::MAX
        assert_eq!(mps_to_cms(1000.0), 65535);
        // Negative should clamp to 0
        assert_eq!(mps_to_cms(-10.0), 0);
    }

    #[test]
    fn test_mps_to_cms_i16_typical() {
        // Forward velocity
        assert_eq!(mps_to_cms_i16(10.0), 1000);
        // Backward velocity
        assert_eq!(mps_to_cms_i16(-10.0), -1000);
        // Zero
        assert_eq!(mps_to_cms_i16(0.0), 0);
    }

    #[test]
    fn test_mps_to_cms_i16_boundary() {
        // Maximum positive (i16::MAX = 32767 cm/s = 327.67 m/s)
        assert_eq!(mps_to_cms_i16(327.67), 32767);
        // Above maximum should clamp
        assert_eq!(mps_to_cms_i16(500.0), 32767);
        // Maximum negative
        assert_eq!(mps_to_cms_i16(-327.68), -32768);
        // Below minimum should clamp
        assert_eq!(mps_to_cms_i16(-500.0), -32768);
    }

    #[test]
    fn test_degrees_to_cdeg_typical() {
        // North
        assert_eq!(degrees_to_cdeg(0.0), 0);
        // East
        assert_eq!(degrees_to_cdeg(90.0), 9000);
        // South
        assert_eq!(degrees_to_cdeg(180.0), 18000);
        // West
        assert_eq!(degrees_to_cdeg(270.0), 27000);
    }

    #[test]
    fn test_degrees_to_cdeg_boundary() {
        // Maximum valid COG (just under 360)
        assert_eq!(degrees_to_cdeg(359.99), 35999);
        // Should clamp to max
        assert_eq!(degrees_to_cdeg(360.0), 35999);
        // Negative should clamp to 0
        assert_eq!(degrees_to_cdeg(-10.0), 0);
    }

    #[test]
    fn test_radians_to_cdeg_typical() {
        use core::f32::consts::PI;

        // North (0 rad)
        assert_eq!(radians_to_cdeg(0.0), 0);
        // East (PI/2 rad = 90 deg)
        assert_eq!(radians_to_cdeg(PI / 2.0), 9000);
        // South (PI rad = 180 deg)
        assert_eq!(radians_to_cdeg(PI), 18000);
        // West (3*PI/2 rad = 270 deg)
        assert_eq!(radians_to_cdeg(3.0 * PI / 2.0), 27000);
    }

    #[test]
    fn test_radians_to_cdeg_negative() {
        use core::f32::consts::PI;

        // -90 degrees (-PI/2 rad) should normalize to 270 degrees
        assert_eq!(radians_to_cdeg(-PI / 2.0), 27000);
        // -180 degrees (-PI rad) should normalize to 180 degrees
        assert_eq!(radians_to_cdeg(-PI), 18000);
        // -45 degrees should normalize to 315 degrees
        assert_eq!(radians_to_cdeg(-PI / 4.0), 31500);
    }

    #[test]
    fn test_convert_gps_fix_type() {
        use crate::devices::gps::GpsFixType as DeviceGpsFixType;

        assert_eq!(
            convert_gps_fix_type(DeviceGpsFixType::NoFix),
            GpsFixType::GPS_FIX_TYPE_NO_FIX
        );
        assert_eq!(
            convert_gps_fix_type(DeviceGpsFixType::Fix2D),
            GpsFixType::GPS_FIX_TYPE_2D_FIX
        );
        assert_eq!(
            convert_gps_fix_type(DeviceGpsFixType::Fix3D),
            GpsFixType::GPS_FIX_TYPE_3D_FIX
        );
    }

    #[test]
    fn test_build_gps_with_no_fix() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new(); // No GPS data

        let msg = streamer.build_gps(&state).unwrap();
        if let MavMessage::GPS_RAW_INT(data) = msg {
            assert_eq!(data.fix_type, GpsFixType::GPS_FIX_TYPE_NO_FIX);
            assert_eq!(data.lat, 0);
            assert_eq!(data.lon, 0);
            assert_eq!(data.satellites_visible, 0);
        } else {
            panic!("Expected GPS_RAW_INT message");
        }
    }

    #[test]
    fn test_build_gps_with_valid_position() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 10.5,
            course_over_ground: Some(84.4),
            fix_type: DeviceGpsFixType::Fix3D,
            satellites: 8,
        };
        state.update_gps(position, 1000000);

        let msg = streamer.build_gps(&state).unwrap();
        if let MavMessage::GPS_RAW_INT(data) = msg {
            assert_eq!(data.fix_type, GpsFixType::GPS_FIX_TYPE_3D_FIX);
            // Allow small float precision error in degE7 conversion
            assert!((data.lat - 481173000).abs() < 100, "lat mismatch");
            assert!((data.lon - 115166000).abs() < 100, "lon mismatch");
            assert_eq!(data.alt, 545400); // 545.4m in mm
            assert_eq!(data.vel, 1050); // 10.5 m/s in cm/s
            assert_eq!(data.cog, 8440); // 84.4 degrees in cdeg
            assert_eq!(data.satellites_visible, 8);
        } else {
            panic!("Expected GPS_RAW_INT message");
        }
    }

    #[test]
    fn test_build_gps_with_no_cog() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position without COG (low speed)
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 0.2,               // Low speed, COG unreliable
            course_over_ground: None, // No COG
            fix_type: DeviceGpsFixType::Fix3D,
            satellites: 8,
        };
        state.update_gps(position, 1000000);

        let msg = streamer.build_gps(&state).unwrap();
        if let MavMessage::GPS_RAW_INT(data) = msg {
            // COG should be u16::MAX when unavailable
            assert_eq!(data.cog, u16::MAX);
        } else {
            panic!("Expected GPS_RAW_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_no_gps() {
        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let state = SystemState::new(); // No GPS data, no AHRS

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            assert_eq!(data.lat, 0);
            assert_eq!(data.lon, 0);
            assert_eq!(data.alt, 0);
            assert_eq!(data.vx, 0);
            assert_eq!(data.vy, 0);
            assert_eq!(data.vz, 0);
            assert_eq!(data.hdg, u16::MAX); // Unknown heading (no AHRS, no GPS COG)
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_with_velocity() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position with velocity heading east (90 degrees)
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 10.0,                    // 10 m/s
            course_over_ground: Some(90.0), // East
            fix_type: DeviceGpsFixType::Fix3D,
            satellites: 8,
        };
        state.update_gps(position, 1000000);

        // Set AHRS heading to east (90 degrees = PI/2 radians)
        state.update_attitude_direct(
            0.0,                         // roll
            0.0,                         // pitch
            core::f32::consts::PI / 2.0, // yaw = 90 degrees
            0.0,
            0.0,
            0.0,     // angular rates
            1000000, // timestamp
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // Allow small float precision error in degE7 conversion
            assert!((data.lat - 481173000).abs() < 100, "lat mismatch");
            assert!((data.lon - 115166000).abs() < 100, "lon mismatch");
            assert_eq!(data.alt, 545400);
            // Heading east (90 deg): vx ≈ 0, vy ≈ 1000 cm/s
            // Due to floating point, allow small tolerance
            assert!(data.vx.abs() < 10); // Nearly 0
            assert!((data.vy - 1000).abs() < 10); // Nearly 1000 cm/s
            assert_eq!(data.vz, 0); // No vertical velocity
            assert_eq!(data.hdg, 9000); // 90 degrees from AHRS
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_heading_north() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position with velocity heading north (0 degrees)
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 10.0,                   // 10 m/s
            course_over_ground: Some(0.0), // North
            fix_type: DeviceGpsFixType::Fix3D,
            satellites: 6,
        };
        state.update_gps(position, 1000000);

        // Set AHRS heading to north (0 degrees = 0 radians)
        state.update_attitude_direct(
            0.0, 0.0, 0.0, // roll, pitch, yaw
            0.0, 0.0, 0.0,     // angular rates
            1000000, // timestamp
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // Heading north (0 deg): vx ≈ 1000 cm/s, vy ≈ 0
            assert!((data.vx - 1000).abs() < 10); // Nearly 1000 cm/s
            assert!(data.vy.abs() < 10); // Nearly 0
            assert_eq!(data.hdg, 0); // 0 degrees from AHRS
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_no_cog_no_ahrs() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position without COG, and AHRS not healthy
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 0.3,
            course_over_ground: None, // No COG
            fix_type: DeviceGpsFixType::Fix2D,
            satellites: 4,
        };
        state.update_gps(position, 1000000);
        // AHRS is not healthy by default

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // Without COG, velocity components should be 0
            assert_eq!(data.vx, 0);
            assert_eq!(data.vy, 0);
            // Heading should be unknown (no AHRS, no GPS COG)
            assert_eq!(data.hdg, u16::MAX);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_ahrs_fallback_to_gps_cog() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position with COG, but AHRS not healthy
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 5.0,
            course_over_ground: Some(45.0), // NE direction
            fix_type: DeviceGpsFixType::Fix3D,
            satellites: 6,
        };
        state.update_gps(position, 1000000);
        // AHRS is not healthy by default

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // Heading should fall back to GPS COG (45 degrees)
            assert_eq!(data.hdg, 4500);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }

    #[test]
    fn test_build_global_position_int_ahrs_negative_yaw() {
        use crate::devices::gps::{GpsFixType as DeviceGpsFixType, GpsPosition};

        let streamer = TelemetryStreamer::<GroundRover>::new(1, 1);
        let mut state = SystemState::new();

        // Set GPS position
        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 100.0,
            speed: 5.0,
            course_over_ground: Some(0.0),
            fix_type: DeviceGpsFixType::Fix3D,
            satellites: 6,
        };
        state.update_gps(position, 1000000);

        // Set AHRS heading to west (-90 degrees = -PI/2 radians)
        // Should normalize to 270 degrees
        state.update_attitude_direct(
            0.0,
            0.0,
            -core::f32::consts::PI / 2.0, // yaw = -90 degrees
            0.0,
            0.0,
            0.0,
            1000000,
        );

        let msg = streamer.build_global_position_int(&state).unwrap();
        if let MavMessage::GLOBAL_POSITION_INT(data) = msg {
            // -90 degrees should normalize to 270 degrees = 27000 cdeg
            assert_eq!(data.hdg, 27000);
        } else {
            panic!("Expected GLOBAL_POSITION_INT message");
        }
    }
}
