//! SITL Autopilot Integration
//!
//! Per-vehicle autopilot context that wires core's `MessageDispatcher` and `ModeExecutor`
//! into the SITL simulation loop. Each vehicle gets its own dispatcher, mode executor,
//! and actuator interface.
//!
//! ## Data Flow
//!
//! 1. GCS sends MAVLink command (e.g., ARM, RC_CHANNELS_OVERRIDE)
//! 2. `VehicleAutopilot::dispatch()` routes to core's MessageDispatcher
//! 3. `sync_rc_input()` copies RC state for ManualMode
//! 4. `VehicleAutopilot::execute_mode()` runs the active mode
//! 5. ManualMode reads RC input and writes to SitlActuator
//! 6. Actuator outputs are applied to SitlPlatform's PWM channels

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex as EmbassyMutex;

use pico_trail_core::autopilot::battery::BatteryFailsafeConfig;
use pico_trail_core::autopilot::mavlink_runner::{BatteryAction, MavlinkLoopRunner};
use pico_trail_core::autopilot::state::{FlightMode, SystemState, SYSTEM_STATE};
use pico_trail_core::autopilot::vehicle::GroundRover;
use pico_trail_core::communication::dispatcher::MessageDispatcher;
use pico_trail_core::communication::handlers::{
    CommandHandler, MissionHandler, ParamHandler, RcInputHandler, TelemetryStreamer,
};
use pico_trail_core::mode::{ManualMode, Mode, ModeExecutor};
use pico_trail_core::navigation::NavigationRunner;
use pico_trail_core::parameters::ParameterStore;
use pico_trail_core::rc::RcInput;
use pico_trail_core::servo::ActuatorInterface;
use pico_trail_core::traits::sync::SharedState;

use crate::platform::SitlPlatform;
use crate::types::SensorData;

/// Static RC input for ManualMode.
///
/// ManualMode expects `&'static Mutex<CriticalSectionRawMutex, RcInput>`.
/// After the dispatcher processes RC messages (which write to core's `RC_INPUT`
/// EmbassyState), call `sync_rc_input()` to copy the latest values here.
static MANUAL_RC: EmbassyMutex<CriticalSectionRawMutex, RcInput> =
    EmbassyMutex::new(RcInput::new());

/// Global actuator output: (steering, throttle).
///
/// Written by `SitlActuator` (via ManualMode), read by `apply_actuators_to_platform()`.
/// Uses the same `critical_section::Mutex<RefCell<T>>` pattern as `SYSTEM_STATE`.
static ACTUATOR_OUTPUT: critical_section::Mutex<core::cell::RefCell<(f32, f32)>> =
    critical_section::Mutex::new(core::cell::RefCell::new((0.0, 0.0)));

/// Sync RC input from core's global RC_INPUT (EmbassyState) to ManualMode's mutex.
///
/// Must be called after `dispatcher.process_rc_input()` to propagate RC updates
/// to the active ManualMode.
pub fn sync_rc_input() {
    let rc_copy = pico_trail_core::rc::RC_INPUT.with(|rc| rc.clone());
    embassy_futures::block_on(async {
        let mut guard = MANUAL_RC.lock().await;
        *guard = rc_copy;
    });
}

/// SITL actuator implementation.
///
/// Stores normalized steering/throttle values (-1.0 to +1.0) that are read
/// by the simulation loop and applied to the platform's PWM channels.
/// Enforces armed state check: outputs are forced to neutral when disarmed.
pub struct SitlActuator {
    steering: f32,
    throttle: f32,
}

impl Default for SitlActuator {
    fn default() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
        }
    }
}

impl SitlActuator {
    pub fn new() -> Self {
        Self::default()
    }
}

impl ActuatorInterface for SitlActuator {
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        critical_section::with(|cs| {
            let armed = SYSTEM_STATE.borrow_ref(cs).is_armed();
            self.steering = if armed {
                normalized.clamp(-1.0, 1.0)
            } else {
                0.0
            };
            ACTUATOR_OUTPUT.borrow_ref_mut(cs).0 = self.steering;
        });
        Ok(())
    }

    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
        critical_section::with(|cs| {
            let armed = SYSTEM_STATE.borrow_ref(cs).is_armed();
            self.throttle = if armed {
                normalized.clamp(-1.0, 1.0)
            } else {
                0.0
            };
            ACTUATOR_OUTPUT.borrow_ref_mut(cs).1 = self.throttle;
        });
        Ok(())
    }

    fn get_steering(&self) -> f32 {
        self.steering
    }

    fn get_throttle(&self) -> f32 {
        self.throttle
    }
}

/// Per-vehicle autopilot context.
///
/// Owns a core `MessageDispatcher` and `ModeExecutor`, connecting GCS commands
/// to mode execution and actuator output.
pub struct VehicleAutopilot {
    pub dispatcher: MessageDispatcher<GroundRover>,
    pub mode_executor: ModeExecutor,
    pub system_id: u8,
    /// Navigation runner for Guided/Auto modes (shared core logic)
    nav_runner: NavigationRunner,
    /// Battery failsafe runner (reused from core)
    mavlink_runner: MavlinkLoopRunner,
}

impl VehicleAutopilot {
    /// Create a new vehicle autopilot with ManualMode as initial mode.
    ///
    /// # Arguments
    ///
    /// * `system_id` - MAVLink system ID for this vehicle
    pub fn new(system_id: u8) -> Self {
        // Create parameter store and register core defaults
        let mut store = ParameterStore::default();
        let _ = pico_trail_core::parameters::ArmingParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::BatteryParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::FailsafeParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::FenceParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::CompassParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::NavigationParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::CircleParams::register_defaults(&mut store);
        let _ = pico_trail_core::parameters::LoiterParams::register_defaults(&mut store);

        // Register MAVLink stream rate and system ID parameters
        use pico_trail_core::parameters::{ParamFlags, ParamValue};
        let _ = store.register("SR_EXTRA1", ParamValue::Int(10), ParamFlags::empty());
        let _ = store.register("SR_POSITION", ParamValue::Int(5), ParamFlags::empty());
        let _ = store.register("SR_RC_CHAN", ParamValue::Int(5), ParamFlags::empty());
        let _ = store.register("SR_RAW_SENS", ParamValue::Int(5), ParamFlags::empty());
        let _ = store.register(
            "SYSID_THISMAV",
            ParamValue::Int(system_id as i32),
            ParamFlags::empty(),
        );

        // Extract battery config before store is consumed by ParamHandler
        let battery_params = pico_trail_core::parameters::BatteryParams::from_store(&store);
        let battery_failsafe_config = BatteryFailsafeConfig::from_params(&battery_params);

        let param_handler = ParamHandler::from_store(store);
        let command_handler = CommandHandler::new();
        let telemetry_streamer = TelemetryStreamer::new(system_id, 1);
        let mission_handler = MissionHandler::new(system_id, 1);
        let rc_input_handler = RcInputHandler::with_system_id(system_id);

        let dispatcher = MessageDispatcher::new(
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        );

        // Create ManualMode with owned actuator and global RC input
        let manual_mode: Box<dyn Mode> =
            Box::new(ManualMode::new(&MANUAL_RC, Box::new(SitlActuator::new())));

        // Create ModeExecutor with owned SystemState
        let mode_executor = ModeExecutor::new(manual_mode, SystemState::new());

        Self {
            dispatcher,
            mode_executor,
            system_id,
            nav_runner: NavigationRunner::new(),
            mavlink_runner: MavlinkLoopRunner::new(battery_failsafe_config),
        }
    }

    /// Dispatch an incoming MAVLink message through core's dispatcher.
    ///
    /// Returns response messages to be sent back to the GCS.
    pub fn dispatch(
        &mut self,
        header: &mavlink::MavHeader,
        message: &mavlink::common::MavMessage,
        timestamp_us: u64,
    ) -> heapless::Vec<mavlink::common::MavMessage, 64> {
        self.dispatcher.dispatch(header, message, timestamp_us)
    }

    /// Process RC input messages (async).
    pub async fn process_rc_input(
        &mut self,
        message: &mavlink::common::MavMessage,
        timestamp_us: u64,
    ) -> bool {
        let result = self
            .dispatcher
            .process_rc_input(message, timestamp_us)
            .await;
        if result {
            sync_rc_input();
        }
        result
    }

    /// Process navigation input messages (async).
    ///
    /// Handles SET_POSITION_TARGET_GLOBAL_INT for GUIDED mode targets.
    pub async fn process_navigation_input(
        &mut self,
        message: &mavlink::common::MavMessage,
    ) -> bool {
        self.dispatcher.process_navigation_input(message).await
    }

    /// Execute the active mode.
    ///
    /// For Manual mode, runs the ModeExecutor (RC → actuators).
    /// For Guided/Auto modes, runs the NavigationRunner (core shared logic).
    pub fn execute_mode(&mut self, current_time_us: u64) {
        let mode = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).mode);

        match mode {
            FlightMode::Guided | FlightMode::Auto => {
                // Use shared NavigationRunner for autonomous modes
                if let Some(output) = self.nav_runner.step(
                    sitl_gps_provider,
                    sitl_heading_provider,
                    0.02, // 50 Hz
                    None, // No gyro yaw rate in SITL
                ) {
                    // Write navigation output to actuators
                    critical_section::with(|cs| {
                        let mut out = ACTUATOR_OUTPUT.borrow_ref_mut(cs);
                        out.0 = output.steering;
                        out.1 = output.throttle;
                    });
                }
            }
            _ => {
                // Manual and other modes use ModeExecutor
                if let Err(e) = self.mode_executor.execute(current_time_us) {
                    eprintln!("  [Autopilot] Mode execute error: {e}");
                }
            }
        }
    }

    /// Get telemetry messages from the dispatcher.
    pub fn update_telemetry(
        &mut self,
        timestamp_us: u64,
    ) -> heapless::Vec<mavlink::common::MavMessage, 16> {
        let state = critical_section::with(|cs| *SYSTEM_STATE.borrow_ref(cs));
        self.dispatcher.update_telemetry(&state, timestamp_us)
    }

    /// Update the global SYSTEM_STATE with sensor data from the simulator.
    pub fn update_from_sensors(&self, sensors: &SensorData) {
        critical_section::with(|cs| {
            let mut state = SYSTEM_STATE.borrow_ref_mut(cs);

            // Update attitude from simulator quaternion
            if let Some(quat) = sensors.attitude_quat {
                // Convert quaternion [w, x, y, z] to Euler angles
                let (roll, pitch, yaw) = quat_to_euler(quat);
                state.update_attitude_direct(
                    roll,
                    pitch,
                    yaw,
                    0.0, // rollspeed
                    0.0, // pitchspeed
                    0.0, // yawspeed
                    sensors.timestamp_us,
                );
            }

            // Update GPS
            if let Some(ref gps) = sensors.gps {
                use pico_trail_core::navigation::GpsPosition;
                let pos = GpsPosition {
                    latitude: gps.lat_deg as f32,
                    longitude: gps.lon_deg as f32,
                    altitude: gps.alt_m,
                    speed: gps.speed_ms,
                    course_over_ground: Some(gps.course_deg),
                    fix_type: gps.fix_type,
                    satellites: gps.satellites,
                };
                state.update_gps(pos, sensors.timestamp_us);
            }

            // Update battery voltage
            if let Some(voltage) = sensors.battery_voltage {
                state.update_battery_voltage(voltage);
            }

            state.update_uptime(sensors.timestamp_us);
        });
    }

    /// Check battery voltage and trigger failsafe actions if needed.
    ///
    /// Should be called at ~10 Hz from the main loop.
    pub fn check_battery_failsafe(&mut self) {
        let (voltage, is_armed) = critical_section::with(|cs| {
            let state = SYSTEM_STATE.borrow_ref(cs);
            (state.battery.voltage, state.is_armed())
        });
        let action = self.mavlink_runner.check_battery(
            voltage,
            is_armed,
            self.dispatcher.param_handler().store(),
        );
        match action {
            BatteryAction::None => {}
            BatteryAction::SetMode(mode) => {
                eprintln!("  [Autopilot] Battery failsafe: voltage={voltage}, mode={mode:?}");
                critical_section::with(|cs| {
                    let _ = SYSTEM_STATE.borrow_ref_mut(cs).set_mode(mode);
                });
            }
            BatteryAction::Disarm => {
                eprintln!("  [Autopilot] Battery CRITICAL: voltage={voltage}, disarming");
                critical_section::with(|cs| {
                    let _ = SYSTEM_STATE.borrow_ref_mut(cs).disarm_forced();
                });
            }
        }
    }

    /// Check for mission protocol timeouts and return an ACK if one expired.
    pub fn check_mission_timeout(
        &mut self,
        timestamp_us: u64,
    ) -> Option<mavlink::common::MavMessage> {
        self.dispatcher.check_mission_timeout(timestamp_us)
    }

    /// Apply actuator outputs to the platform's PWM channels.
    ///
    /// Reads steering/throttle from the global `ACTUATOR_OUTPUT` (written by `SitlActuator`
    /// during mode execution) and applies `DifferentialDrive::mix()` from core for
    /// consistent mixing with firmware.
    ///
    /// Output mapping matches Gazebo ArduPilotPlugin servo channels:
    /// - PWM index 0 → servo slot 0 → left motors (motor_0, motor_1)
    /// - PWM index 2 → servo slot 2 → right motors (motor_2, motor_3)
    pub fn apply_actuators_to_platform(&self, platform: &SitlPlatform) {
        let (steering, throttle) = critical_section::with(|cs| {
            if !SYSTEM_STATE.borrow_ref(cs).is_armed() {
                return (0.0_f32, 0.0_f32);
            }
            let out = ACTUATOR_OUTPUT.borrow_ref(cs);
            (out.0, out.1)
        });

        // Use core's DifferentialDrive for consistent mixing with firmware
        let (left, right) = pico_trail_core::kinematics::DifferentialDrive::mix(steering, throttle);

        // Convert from [-1, 1] to [0, 1] duty cycle for PWM
        let left_duty = (left + 1.0) / 2.0;
        let right_duty = (right + 1.0) / 2.0;

        platform.set_pwm_duty(0, left_duty); // servo slot 0: left motors
        platform.set_pwm_duty(2, right_duty); // servo slot 2: right motors
    }
}

/// GPS provider for SITL modes (reads from SYSTEM_STATE).
fn sitl_gps_provider() -> Option<pico_trail_core::navigation::GpsPosition> {
    critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).gps_position)
}

/// Heading provider for SITL modes (reads yaw from SYSTEM_STATE attitude).
fn sitl_heading_provider() -> Option<f32> {
    critical_section::with(|cs| {
        let state = SYSTEM_STATE.borrow_ref(cs);
        // Use yaw from attitude (converted to 0-360 degrees)
        let yaw_deg = state.attitude.yaw.to_degrees();
        let heading = if yaw_deg < 0.0 {
            yaw_deg + 360.0
        } else {
            yaw_deg
        };
        Some(heading)
    })
}

/// Read current FlightMode from the global SYSTEM_STATE.
pub fn current_flight_mode() -> FlightMode {
    critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).mode)
}

/// Check if the vehicle is armed from the global SYSTEM_STATE.
pub fn is_armed() -> bool {
    critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed())
}

/// Convert quaternion [w, x, y, z] (NED frame) to Euler angles (roll, pitch, yaw) in radians.
pub(crate) fn quat_to_euler(q: [f32; 4]) -> (f32, f32, f32) {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        core::f32::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    (roll, pitch, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pico_trail_core::autopilot::state::ArmedState;

    #[test]
    fn test_sitl_actuator_neutral_when_disarmed() {
        // Ensure disarmed
        critical_section::with(|cs| {
            let _ = SYSTEM_STATE.borrow_ref_mut(cs).disarm_forced();
        });

        let mut actuator = SitlActuator::new();
        actuator.set_steering(0.5).unwrap();
        actuator.set_throttle(0.8).unwrap();

        assert!(actuator.get_steering().abs() < 0.001);
        assert!(actuator.get_throttle().abs() < 0.001);
    }

    #[test]
    fn test_sitl_actuator_passes_through_when_armed() {
        // Arm the vehicle
        critical_section::with(|cs| {
            let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
            state.armed = ArmedState::Armed;
        });

        let mut actuator = SitlActuator::new();
        actuator.set_steering(0.5).unwrap();
        actuator.set_throttle(0.8).unwrap();

        assert!((actuator.get_steering() - 0.5).abs() < 0.001);
        assert!((actuator.get_throttle() - 0.8).abs() < 0.001);

        // Clean up
        critical_section::with(|cs| {
            let _ = SYSTEM_STATE.borrow_ref_mut(cs).disarm_forced();
        });
    }

    #[test]
    fn test_actuator_output_global_updated() {
        // Arm
        critical_section::with(|cs| {
            SYSTEM_STATE.borrow_ref_mut(cs).armed = ArmedState::Armed;
        });

        let mut actuator = SitlActuator::new();
        actuator.set_steering(0.3).unwrap();
        actuator.set_throttle(0.7).unwrap();

        // Verify ACTUATOR_OUTPUT global was updated
        let (steer, throttle) = critical_section::with(|cs| *ACTUATOR_OUTPUT.borrow_ref(cs));
        assert!((steer - 0.3).abs() < 0.001, "steer={steer}");
        assert!((throttle - 0.7).abs() < 0.001, "throttle={throttle}");

        // Clean up
        critical_section::with(|cs| {
            let _ = SYSTEM_STATE.borrow_ref_mut(cs).disarm_forced();
        });
    }

    #[test]
    fn test_quat_to_euler_identity() {
        let (roll, pitch, yaw) = quat_to_euler([1.0, 0.0, 0.0, 0.0]);
        assert!(roll.abs() < 0.001);
        assert!(pitch.abs() < 0.001);
        assert!(yaw.abs() < 0.001);
    }

    #[test]
    fn test_vehicle_autopilot_creation() {
        let autopilot = VehicleAutopilot::new(1);
        assert_eq!(autopilot.system_id, 1);
        assert_eq!(autopilot.mode_executor.current_mode_name(), "Manual");
    }

    #[test]
    fn test_sync_rc_input() {
        // Write to core's RC_INPUT
        pico_trail_core::rc::RC_INPUT.with_mut(|rc| {
            rc.channels[0] = 0.5;
            rc.channels[2] = 0.8;
        });

        // Sync to MANUAL_RC
        sync_rc_input();

        // Verify MANUAL_RC has the values
        let (ch1, ch3) = embassy_futures::block_on(async {
            let rc = MANUAL_RC.lock().await;
            (rc.channels[0], rc.channels[2])
        });

        assert!((ch1 - 0.5).abs() < 0.001);
        assert!((ch3 - 0.8).abs() < 0.001);
    }
}
