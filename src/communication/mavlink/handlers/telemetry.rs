//! MAVLink Telemetry Streaming
//!
//! Implements periodic telemetry message streaming to Ground Control Stations.
//!
//! # Supported Messages
//!
//! - **HEARTBEAT**: System status (1Hz)
//! - **ATTITUDE**: Roll, pitch, yaw (10Hz default, configurable via SR_EXTRA1)
//! - **GPS_RAW_INT**: GPS position (5Hz default, configurable via SR_POSITION)
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
use mavlink::common::{
    GpsFixType, MavAutopilot, MavBatteryFunction, MavBatteryType, MavMessage, MavModeFlag,
    MavState, MavSysStatusSensor, MavType, ATTITUDE_DATA, BATTERY_STATUS_DATA, GPS_RAW_INT_DATA,
    HEARTBEAT_DATA, SYS_STATUS_DATA,
};

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

/// Telemetry streamer for periodic message transmission
///
/// Manages stream rates and generates telemetry messages based on system state.
pub struct TelemetryStreamer {
    /// System ID for MAVLink messages
    #[allow(dead_code)]
    system_id: u8,
    /// Component ID for MAVLink messages
    #[allow(dead_code)]
    component_id: u8,
    /// HEARTBEAT stream config (always 1Hz)
    heartbeat: StreamConfig,
    /// ATTITUDE stream config (SR_EXTRA1)
    attitude: StreamConfig,
    /// GPS_RAW_INT stream config (SR_POSITION)
    gps: StreamConfig,
    /// SYS_STATUS stream config (SR_EXTRA1)
    sys_status: StreamConfig,
    /// BATTERY_STATUS stream config (hardcoded 2Hz)
    battery_status: StreamConfig,
}

impl TelemetryStreamer {
    /// Create a new telemetry streamer with default rates
    ///
    /// # Arguments
    ///
    /// * `system_id` - MAVLink system ID (default: 1)
    /// * `component_id` - MAVLink component ID (default: 1)
    ///
    /// # Returns
    ///
    /// Returns a telemetry streamer with default stream rates:
    /// - HEARTBEAT: 1Hz
    /// - ATTITUDE: 10Hz
    /// - GPS: 5Hz
    /// - SYS_STATUS: 1Hz
    /// - BATTERY_STATUS: 2Hz
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            system_id,
            component_id,
            heartbeat: StreamConfig::new(1),      // Always 1Hz
            attitude: StreamConfig::new(10),      // SR_EXTRA1 default
            gps: StreamConfig::new(5),            // SR_POSITION default
            sys_status: StreamConfig::new(1), // SR_EXTRA1 default (lower priority than attitude)
            battery_status: StreamConfig::new(2), // Hardcoded 2Hz
        }
    }

    /// Update stream rates from parameters
    ///
    /// # Arguments
    ///
    /// * `sr_extra1` - Rate for ATTITUDE and SYS_STATUS (Hz)
    /// * `sr_position` - Rate for GPS_RAW_INT (Hz)
    pub fn update_rates(&mut self, sr_extra1: u32, sr_position: u32) {
        self.attitude.rate_hz = sr_extra1;
        self.sys_status.rate_hz = sr_extra1.min(1); // SYS_STATUS max 1Hz
        self.gps.rate_hz = sr_position;
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
    /// Vector of messages to send (max 5: HEARTBEAT, ATTITUDE, GPS, SYS_STATUS, BATTERY_STATUS)
    pub fn update(
        &mut self,
        state: &SystemState,
        current_time_us: u64,
    ) -> heapless::Vec<MavMessage, 5> {
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
        let base_mode = if state.is_armed() {
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
        } else {
            MavModeFlag::empty()
        };

        Some(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GROUND_ROVER, // Rover/boat autopilot
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        }))
    }

    /// Build ATTITUDE message
    ///
    /// Currently returns zeros for roll, pitch, yaw until AHRS is implemented.
    fn build_attitude(&self, state: &SystemState) -> Option<MavMessage> {
        Some(MavMessage::ATTITUDE(ATTITUDE_DATA {
            time_boot_ms: (state.uptime_us / 1000) as u32,
            roll: 0.0,       // Placeholder: will be from AHRS
            pitch: 0.0,      // Placeholder: will be from AHRS
            yaw: 0.0,        // Placeholder: will be from AHRS
            rollspeed: 0.0,  // Placeholder: will be from AHRS
            pitchspeed: 0.0, // Placeholder: will be from AHRS
            yawspeed: 0.0,   // Placeholder: will be from AHRS
        }))
    }

    /// Build GPS_RAW_INT message
    ///
    /// Currently returns zeros until GPS integration is complete.
    fn build_gps(&self, state: &SystemState) -> Option<MavMessage> {
        Some(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: state.uptime_us,
            lat: 0,    // Placeholder: will be from GPS (degrees * 1e7)
            lon: 0,    // Placeholder: will be from GPS (degrees * 1e7)
            alt: 0,    // Placeholder: will be from GPS (mm)
            eph: 9999, // Placeholder: GPS HDOP (cm)
            epv: 9999, // Placeholder: GPS VDOP (cm)
            vel: 0,    // Placeholder: ground speed (cm/s)
            cog: 0,    // Placeholder: course over ground (cdeg)
            fix_type: GpsFixType::GPS_FIX_TYPE_NO_FIX, // Placeholder: no fix
            satellites_visible: 0, // Placeholder: satellites in view
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
        MavMessage::PROTOCOL_VERSION(CommandHandler::create_protocol_version_message())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let streamer = TelemetryStreamer::new(1, 1);
        let (hb, att, gps, sys, bat) = streamer.get_rates();

        assert_eq!(hb, 1); // HEARTBEAT always 1Hz
        assert_eq!(att, 10); // ATTITUDE default 10Hz
        assert_eq!(gps, 5); // GPS default 5Hz
        assert_eq!(sys, 1); // SYS_STATUS default 1Hz
        assert_eq!(bat, 2); // BATTERY_STATUS default 2Hz
    }

    #[test]
    fn test_update_rates() {
        let mut streamer = TelemetryStreamer::new(1, 1);

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
        let streamer = TelemetryStreamer::new(1, 1);
        let mut state = SystemState::new();

        // Test unarmed
        let msg = streamer.build_heartbeat(&state).unwrap();
        if let MavMessage::HEARTBEAT(data) = msg {
            assert_eq!(data.mavtype, MavType::MAV_TYPE_GROUND_ROVER);
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
        let streamer = TelemetryStreamer::new(1, 1);
        let mut state = SystemState::new();
        state.uptime_us = 5_000_000; // 5 seconds

        let msg = streamer.build_attitude(&state).unwrap();
        if let MavMessage::ATTITUDE(data) = msg {
            assert_eq!(data.time_boot_ms, 5000); // 5000ms
                                                 // Placeholder values should be zero
            assert_eq!(data.roll, 0.0);
            assert_eq!(data.pitch, 0.0);
            assert_eq!(data.yaw, 0.0);
        } else {
            panic!("Expected ATTITUDE message");
        }
    }

    #[test]
    fn test_build_sys_status() {
        let streamer = TelemetryStreamer::new(1, 1);
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
        let streamer = TelemetryStreamer::new(1, 1);
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
        let mut streamer = TelemetryStreamer::new(1, 1);
        let state = SystemState::new();

        // At t=0, all messages should send (first time for all streams)
        let messages = streamer.update(&state, 0);
        assert_eq!(messages.len(), 5); // HEARTBEAT, ATTITUDE, GPS, SYS_STATUS, BATTERY_STATUS
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::HEARTBEAT(_))));

        // At t=100ms, ATTITUDE should send (10Hz = 100ms interval)
        let messages = streamer.update(&state, 100_000);
        assert_eq!(messages.len(), 1);
        assert!(matches!(messages[0], MavMessage::ATTITUDE(_)));
    }

    #[test]
    fn test_update_multiple_messages() {
        let mut streamer = TelemetryStreamer::new(1, 1);
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
        let mut streamer = TelemetryStreamer::new(1, 1);
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
}
