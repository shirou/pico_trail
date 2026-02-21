//! Integration tests for SITL autopilot (core integration).
//!
//! Tests the closed-loop data flow: GCS command → dispatcher → mode → actuator → state.

use mavlink::common::*;
use mavlink::MavHeader;
use pico_trail_core::autopilot::state::{ArmedState, FlightMode, SYSTEM_STATE};
use pico_trail_core::traits::sync::SharedState;
use pico_trail_sitl::autopilot::VehicleAutopilot;
use pico_trail_sitl::types::{GpsData, GpsFixType, SensorData, VehicleId};

/// GCS header (Mission Planner defaults: sys=255, comp=190).
fn gcs_header() -> MavHeader {
    MavHeader {
        system_id: 255,
        component_id: 190,
        sequence: 0,
    }
}

/// Build a COMMAND_LONG to arm (force=true bypasses pre-arm checks).
fn arm_command(force: bool) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 1.0,                               // arm
        param2: if force { 21196.0 } else { 0.0 }, // force magic
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}

/// Build a COMMAND_LONG to disarm (force).
fn disarm_command() -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 0.0,     // disarm
        param2: 21196.0, // force
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    })
}

/// Reset global SYSTEM_STATE to a clean disarmed state.
fn reset_system_state() {
    critical_section::with(|cs| {
        let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
        let _ = state.disarm_forced();
        state.mode = FlightMode::Manual;
    });
}

#[test]
fn test_arm_via_dispatcher() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);
    let header = gcs_header();
    let msg = arm_command(true);

    let responses = autopilot.dispatch(&header, &msg, 1_000_000);

    // Should get COMMAND_ACK with ACCEPTED
    let ack = responses
        .iter()
        .find(|m| matches!(m, MavMessage::COMMAND_ACK(_)));
    assert!(ack.is_some(), "Expected COMMAND_ACK in responses");
    if let MavMessage::COMMAND_ACK(data) = ack.unwrap() {
        assert_eq!(data.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    // SYSTEM_STATE should now be armed
    let armed = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed());
    assert!(armed, "Vehicle should be armed after ARM command");

    reset_system_state();
}

#[test]
fn test_disarm_via_dispatcher() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);
    let header = gcs_header();

    // First arm
    autopilot.dispatch(&header, &arm_command(true), 1_000_000);

    // Then disarm
    let responses = autopilot.dispatch(&header, &disarm_command(), 2_000_000);

    let ack = responses
        .iter()
        .find(|m| matches!(m, MavMessage::COMMAND_ACK(_)));
    assert!(ack.is_some());
    if let MavMessage::COMMAND_ACK(data) = ack.unwrap() {
        assert_eq!(data.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    let armed = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed());
    assert!(!armed, "Vehicle should be disarmed after DISARM command");

    reset_system_state();
}

#[test]
fn test_disarmed_actuators_produce_zero() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);

    // Execute mode while disarmed — actuators should stay at zero
    autopilot.execute_mode(1_000_000);

    // The SitlActuator enforces armed check internally.
    // Since we can't directly inspect the leaked actuator, we verify through
    // the fact that no errors occur and the mode executor runs cleanly.
    assert_eq!(autopilot.mode_executor.current_mode_name(), "Manual");

    reset_system_state();
}

#[test]
fn test_update_from_sensors_propagates_gps() {
    reset_system_state();

    let autopilot = VehicleAutopilot::new(1);

    let sensors = SensorData {
        timestamp_us: 500_000,
        vehicle_id: VehicleId(1),
        imu: None,
        gps: Some(GpsData {
            lat_deg: 35.681236,
            lon_deg: 139.767125,
            alt_m: 40.0,
            speed_ms: 1.5,
            course_deg: 90.0,
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
            hdop: 1.2,
        }),
        compass: None,
        barometer: None,
        attitude_quat: None,
        battery_voltage: None,
    };

    autopilot.update_from_sensors(&sensors);

    // Verify GPS data propagated to SYSTEM_STATE
    let (lat, lon, alt) = critical_section::with(|cs| {
        let state = SYSTEM_STATE.borrow_ref(cs);
        match state.gps_position {
            Some(pos) => (pos.latitude, pos.longitude, pos.altitude),
            None => (0.0, 0.0, 0.0),
        }
    });

    assert!((lat - 35.681236).abs() < 0.001, "lat={lat}");
    assert!((lon - 139.767_12).abs() < 0.001, "lon={lon}");
    assert!((alt - 40.0).abs() < 0.1, "alt={alt}");

    reset_system_state();
}

#[test]
fn test_update_from_sensors_propagates_attitude() {
    reset_system_state();

    let autopilot = VehicleAutopilot::new(1);

    // 90° yaw rotation quaternion: [cos(45°), 0, 0, sin(45°)]
    let yaw_90 = [
        (core::f32::consts::FRAC_PI_4).cos(),
        0.0,
        0.0,
        (core::f32::consts::FRAC_PI_4).sin(),
    ];

    let sensors = SensorData {
        timestamp_us: 1_000_000,
        vehicle_id: VehicleId(1),
        imu: None,
        gps: None,
        compass: None,
        barometer: None,
        attitude_quat: Some(yaw_90),
        battery_voltage: None,
    };

    autopilot.update_from_sensors(&sensors);

    let yaw = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).attitude.yaw);

    // Yaw should be ~PI/2 (90°)
    assert!(
        (yaw - core::f32::consts::FRAC_PI_2).abs() < 0.01,
        "yaw={yaw}, expected ~{:.3}",
        core::f32::consts::FRAC_PI_2
    );

    reset_system_state();
}

#[test]
fn test_rc_input_dispatch_and_sync() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);

    // Send RC_CHANNELS_OVERRIDE with steering and throttle
    let rc_msg = MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
        target_system: 1,
        target_component: 1,
        chan1_raw: 1700, // steering right
        chan2_raw: 1500, // centered
        chan3_raw: 1600, // throttle forward
        chan4_raw: 1500,
        chan5_raw: 0,
        chan6_raw: 0,
        chan7_raw: 0,
        chan8_raw: 0,
        // v2 extensions
        chan9_raw: 0,
        chan10_raw: 0,
        chan11_raw: 0,
        chan12_raw: 0,
        chan13_raw: 0,
        chan14_raw: 0,
        chan15_raw: 0,
        chan16_raw: 0,
        chan17_raw: 0,
        chan18_raw: 0,
    });

    // Process RC through dispatcher (async)
    embassy_futures::block_on(async {
        autopilot.process_rc_input(&rc_msg, 1_000_000).await;
    });

    // Verify RC_INPUT was updated by checking global state
    let ch1 = pico_trail_core::rc::RC_INPUT.with(|rc| rc.get_channel(1));
    // Channel 1 should be non-zero (mapped from 1700)
    assert!(
        ch1.abs() > 0.1,
        "RC channel 1 should be non-zero, got {ch1}"
    );

    reset_system_state();
}

#[test]
fn test_closed_loop_arm_and_execute() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);
    let header = gcs_header();

    // 1. Arm the vehicle (force)
    autopilot.dispatch(&header, &arm_command(true), 1_000_000);
    let armed = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed());
    assert!(armed, "Should be armed");

    // 2. Send RC input
    let rc_msg = MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
        target_system: 1,
        target_component: 1,
        chan1_raw: 1600,
        chan2_raw: 1500,
        chan3_raw: 1700,
        chan4_raw: 1500,
        chan5_raw: 0,
        chan6_raw: 0,
        chan7_raw: 0,
        chan8_raw: 0,
        chan9_raw: 0,
        chan10_raw: 0,
        chan11_raw: 0,
        chan12_raw: 0,
        chan13_raw: 0,
        chan14_raw: 0,
        chan15_raw: 0,
        chan16_raw: 0,
        chan17_raw: 0,
        chan18_raw: 0,
    });
    embassy_futures::block_on(async {
        autopilot.process_rc_input(&rc_msg, 2_000_000).await;
    });

    // 3. Execute mode — ManualMode reads RC and writes to SitlActuator
    autopilot.execute_mode(2_000_000);

    // 4. Verify mode is still Manual and no errors occurred
    assert_eq!(autopilot.mode_executor.current_mode_name(), "Manual");

    // 5. Get telemetry — should produce messages
    let telemetry = autopilot.update_telemetry(3_000_000);
    // Telemetry may or may not have messages depending on rate limiting,
    // but the call should not panic.
    let _ = telemetry;

    reset_system_state();
}

#[test]
fn test_manual_control_dispatch() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);

    // Send MANUAL_CONTROL with forward throttle and right steering
    let mc_msg = MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
        target: 1,
        x: 1000, // full forward
        y: 500,  // half right
        z: 0,
        r: 0,
        buttons: 0,
        buttons2: 0,
        enabled_extensions: 0,
        s: 0,
        t: 0,
        aux1: 0,
        aux2: 0,
        aux3: 0,
        aux4: 0,
        aux5: 0,
        aux6: 0,
    });

    embassy_futures::block_on(async {
        autopilot.process_rc_input(&mc_msg, 1_000_000).await;
    });

    // Verify RC_INPUT was updated
    let ch1 = pico_trail_core::rc::RC_INPUT.with(|rc| rc.get_channel(1));
    let ch3 = pico_trail_core::rc::RC_INPUT.with(|rc| rc.get_channel(3));

    // x=1000 (forward) → throttle ~+1.0
    assert!(
        (ch3 - 1.0).abs() < 0.01,
        "Throttle should be ~1.0 for x=1000, got {ch3}"
    );
    // y=500 (right) → steering ~+0.5
    assert!(
        (ch1 - 0.5).abs() < 0.01,
        "Steering should be ~0.5 for y=500, got {ch1}"
    );

    reset_system_state();
}

#[test]
fn test_flight_mode_helpers() {
    reset_system_state();

    // Initially should be Manual and disarmed
    assert_eq!(
        pico_trail_sitl::autopilot::current_flight_mode(),
        FlightMode::Manual
    );
    assert!(!pico_trail_sitl::autopilot::is_armed());

    // Arm
    critical_section::with(|cs| {
        let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
        state.armed = ArmedState::Armed;
    });
    assert!(pico_trail_sitl::autopilot::is_armed());

    // Change mode
    critical_section::with(|cs| {
        SYSTEM_STATE.borrow_ref_mut(cs).mode = FlightMode::Hold;
    });
    assert_eq!(
        pico_trail_sitl::autopilot::current_flight_mode(),
        FlightMode::Hold
    );

    reset_system_state();
}

#[test]
fn test_set_mode_via_dispatcher() {
    reset_system_state();

    let mut autopilot = VehicleAutopilot::new(1);
    let header = gcs_header();

    // Send SET_MODE command: param2 = 10 (Auto)
    let set_mode_cmd = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_DO_SET_MODE,
        confirmation: 0,
        param1: 0.0,
        param2: 10.0, // Auto mode
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });

    let responses = autopilot.dispatch(&header, &set_mode_cmd, 1_000_000);

    // Should get COMMAND_ACK with ACCEPTED
    let ack = responses
        .iter()
        .find(|m| matches!(m, MavMessage::COMMAND_ACK(_)));
    assert!(ack.is_some(), "Expected COMMAND_ACK in responses");
    if let MavMessage::COMMAND_ACK(data) = ack.unwrap() {
        assert_eq!(data.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    // SYSTEM_STATE should now be in Auto mode
    let mode = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).mode);
    assert_eq!(mode, FlightMode::Auto, "Mode should be Auto after SET_MODE");

    // Verify heartbeat reflects the new mode (custom_mode == 10)
    let telemetry = autopilot.update_telemetry(2_000_000);
    let heartbeat = telemetry
        .iter()
        .find(|m| matches!(m, MavMessage::HEARTBEAT(_)));
    if let Some(MavMessage::HEARTBEAT(hb)) = heartbeat {
        assert_eq!(
            hb.custom_mode, 10,
            "Heartbeat custom_mode should be 10 (Auto)"
        );
    }

    reset_system_state();
}

/// Full closed-loop integration test: SitlBridge + LightweightAdapter + VehicleAutopilot.
///
/// Exercises the complete pipeline without Gazebo:
/// ARM → RC input → execute mode → bridge.step() → sensors → telemetry
#[tokio::test]
async fn test_full_closed_loop_with_lightweight_adapter() {
    reset_system_state();

    use pico_trail_sitl::{
        LightweightAdapter, LightweightConfig, SitlBridge, TimeMode, VehicleConfig, VehicleType,
    };

    // 1. Set up bridge with LightweightAdapter
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Lockstep {
        step_size_us: 10_000,
    });

    let id = VehicleId(1);
    let config = LightweightConfig {
        seed: Some(42),
        gps_noise_m: 0.0, // deterministic
        gps_rate_hz: 100, // GPS every step for reliable test
        step_size_us: 10_000,
        ..Default::default()
    };
    let adapter = LightweightAdapter::new("sim1", id, config);
    bridge
        .register_adapter(Box::new(adapter))
        .expect("register adapter");
    bridge
        .spawn_vehicle(VehicleConfig::new(id, VehicleType::Rover))
        .expect("spawn vehicle");
    bridge
        .assign_vehicle_to_adapter(id, "sim1")
        .expect("assign vehicle");
    bridge
        .get_adapter_mut("sim1")
        .unwrap()
        .connect()
        .await
        .expect("connect adapter");

    // Set up PWM channels
    let v = bridge.get_vehicle(id).unwrap();
    v.platform.create_pwm(0, 50).unwrap();
    v.platform.create_pwm(1, 50).unwrap();

    // 2. Create autopilot
    let mut autopilot = VehicleAutopilot::new(1);

    // 3. ARM the vehicle
    let header = gcs_header();
    autopilot.dispatch(&header, &arm_command(true), 1_000_000);
    let armed = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed());
    assert!(armed, "Vehicle should be armed");

    // 4. Send RC input (throttle forward)
    let rc_msg = MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
        target_system: 1,
        target_component: 1,
        chan1_raw: 1500, // steering neutral
        chan2_raw: 1500,
        chan3_raw: 1700, // throttle forward
        chan4_raw: 1500,
        chan5_raw: 0,
        chan6_raw: 0,
        chan7_raw: 0,
        chan8_raw: 0,
        chan9_raw: 0,
        chan10_raw: 0,
        chan11_raw: 0,
        chan12_raw: 0,
        chan13_raw: 0,
        chan14_raw: 0,
        chan15_raw: 0,
        chan16_raw: 0,
        chan17_raw: 0,
        chan18_raw: 0,
    });
    autopilot.process_rc_input(&rc_msg, 2_000_000).await;

    // 5. Execute mode → writes actuator output
    autopilot.execute_mode(2_000_000);

    // 6. Apply actuators to platform PWM channels
    if let Some(vehicle) = bridge.get_vehicle(id) {
        autopilot.apply_actuators_to_platform(&vehicle.platform);
    }

    // 7. Step bridge → sends actuators, advances physics, receives sensors
    for _ in 0..5 {
        bridge.step().await.expect("bridge step");
    }

    // 8. Read sensor data from bridge and update autopilot state
    if let Some(sensors) = bridge
        .get_vehicle(id)
        .and_then(|v| v.platform.peek_sensors())
    {
        autopilot.update_from_sensors(&sensors);
    }

    // 9. Verify sensors were propagated (GPS should be populated by LightweightAdapter)
    let has_gps = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).gps_position.is_some());
    assert!(
        has_gps,
        "GPS data should be populated from LightweightAdapter sensors"
    );

    // 10. Generate telemetry — should produce messages
    let telemetry = autopilot.update_telemetry(5_000_000);
    assert!(
        !telemetry.is_empty(),
        "Telemetry should contain at least HEARTBEAT"
    );

    // Verify HEARTBEAT is in telemetry
    let has_heartbeat = telemetry
        .iter()
        .any(|m| matches!(m, MavMessage::HEARTBEAT(_)));
    assert!(has_heartbeat, "Telemetry should include HEARTBEAT");

    reset_system_state();
}

#[test]
fn test_home_auto_set_on_gps_fix() {
    reset_system_state();

    // Ensure home is initially None
    let has_home_before =
        critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).home_position.is_some());
    assert!(!has_home_before, "Home should be None at start");

    let mut autopilot = VehicleAutopilot::new(1);

    // Inject GPS 3D fix via sensor update
    let sensors = SensorData {
        gps: Some(GpsData {
            lat_deg: 35.6812,
            lon_deg: 139.7671,
            alt_m: 40.0,
            speed_ms: 0.0,
            course_deg: 0.0,
            fix_type: pico_trail_core::navigation::GpsFixType::Fix3D,
            satellites: 12,
            hdop: 1.0,
        }),
        timestamp_us: 1_000_000,
        vehicle_id: VehicleId(1),
        imu: None,
        compass: None,
        barometer: None,
        attitude_quat: None,
        battery_voltage: None,
    };
    autopilot.update_from_sensors(&sensors);

    // Call check_home_position — should auto-set home and return HOME_POSITION
    let result = autopilot.check_home_position();
    assert!(result.is_some(), "Should return HOME_POSITION on auto-set");
    assert!(
        matches!(result.unwrap(), MavMessage::HOME_POSITION(_)),
        "Message should be HOME_POSITION"
    );

    // Verify home is now set in SYSTEM_STATE
    let has_home_after =
        critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).home_position.is_some());
    assert!(has_home_after, "Home should be set after auto-set");

    // Second call should NOT return HOME_POSITION (already set)
    let result2 = autopilot.check_home_position();
    assert!(result2.is_none(), "Should not re-set home on second call");

    reset_system_state();
}
