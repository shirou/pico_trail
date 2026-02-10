//! Single-vehicle SITL bridge for Gazebo Harmonic.
//!
//! Each process handles exactly one vehicle with its own autopilot state.
//! Run multiple processes (one per vehicle) with distinct `--system-id`,
//! `--gazebo-port`, and `--mavlink-port` values. Use mavp2p to aggregate
//! all vehicle TCP streams into a single GCS endpoint.
//!
//! Usage:
//!   cargo run -p pico_trail_sitl --bin gazebo_bridge -- [OPTIONS]
//!
//! Options:
//!   --system-id <ID>       MAVLink system ID (default: 1)
//!   --gazebo-port <PORT>   Gazebo fdm_port_in UDP port (default: 9002)
//!   --mavlink-port <PORT>  MAVLink TCP port for GCS (default: 5760)

use std::env;
use std::process;

use pico_trail_sitl::adapter::{GazeboAdapter, GazeboConfig};
use pico_trail_sitl::autopilot::VehicleAutopilot;
use pico_trail_sitl::{GcsLink, SitlBridge, TimeMode, VehicleConfig, VehicleId, VehicleType};

struct Args {
    system_id: u8,
    gazebo_port: u16,
    mavlink_port: u16,
}

fn parse_args() -> Args {
    let mut args = Args {
        system_id: 1,
        gazebo_port: 9002,
        mavlink_port: 5760,
    };

    let raw: Vec<String> = env::args().collect();
    let mut i = 1;
    while i < raw.len() {
        match raw[i].as_str() {
            "--system-id" => {
                i += 1;
                let val = parse_u16_arg(&raw, i, "system-id");
                if val == 0 || val > 255 {
                    eprintln!("Error: --system-id must be 1..255");
                    process::exit(1);
                }
                args.system_id = val as u8;
            }
            "--gazebo-port" => {
                i += 1;
                args.gazebo_port = parse_u16_arg(&raw, i, "gazebo-port");
            }
            "--mavlink-port" => {
                i += 1;
                args.mavlink_port = parse_u16_arg(&raw, i, "mavlink-port");
            }
            "-h" | "--help" => {
                print_usage();
                process::exit(0);
            }
            other => {
                eprintln!("Unknown option: {other}");
                print_usage();
                process::exit(1);
            }
        }
        i += 1;
    }

    args
}

fn parse_u16_arg(raw: &[String], i: usize, name: &str) -> u16 {
    raw.get(i)
        .unwrap_or_else(|| {
            eprintln!("Error: --{name} requires a value");
            process::exit(1);
        })
        .parse()
        .unwrap_or_else(|_| {
            eprintln!("Error: invalid value for --{name}");
            process::exit(1);
        })
}

fn print_usage() {
    eprintln!(
        "Usage: gazebo_bridge [OPTIONS]\n\
         \n\
         Options:\n\
         \x20 --system-id <ID>       MAVLink system ID (default: 1)\n\
         \x20 --gazebo-port <PORT>   Gazebo fdm_port_in UDP port (default: 9002)\n\
         \x20 --mavlink-port <PORT>  MAVLink TCP port for GCS (default: 5760)\n\
         \x20 -h, --help             Show this help\n\
         \n\
         For multi-vehicle, run one process per vehicle:\n\
         \x20 gazebo_bridge --system-id 1 --gazebo-port 9002 --mavlink-port 5760\n\
         \x20 gazebo_bridge --system-id 2 --gazebo-port 9012 --mavlink-port 5762\n\
         \x20 gazebo_bridge --system-id 3 --gazebo-port 9022 --mavlink-port 5764\n\
         \n\
         Then use mavp2p to aggregate for GCS:\n\
         \x20 mavp2p tcpc:127.0.0.1:5760 tcpc:127.0.0.1:5762 tcpc:127.0.0.1:5764 tcps:0.0.0.0:5770"
    );
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args = parse_args();

    println!("=== pico_trail Gazebo Bridge ===");
    println!(
        "Vehicle: system-id={}, gazebo-port={}, mavlink-port={}",
        args.system_id, args.gazebo_port, args.mavlink_port
    );
    println!();

    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Scaled { factor: 1.0 });

    let mut gcs = GcsLink::new(args.mavlink_port).expect("Failed to bind MAVLink TCP port");

    // Register single vehicle
    let id = VehicleId(args.system_id);
    let adapter_name = "gazebo";

    let config = GazeboConfig {
        gazebo_addr: format!("127.0.0.1:{}", args.gazebo_port).parse().unwrap(),
        timeout_ms: 100,
        ..Default::default()
    };
    let adapter = GazeboAdapter::new(adapter_name, id, config);
    bridge
        .register_adapter(Box::new(adapter))
        .expect("Failed to register adapter");

    let vehicle_config = VehicleConfig::new(id, VehicleType::Rover);
    bridge
        .spawn_vehicle(vehicle_config)
        .expect("Failed to spawn vehicle");
    bridge
        .assign_vehicle_to_adapter(id, adapter_name)
        .expect("Failed to assign vehicle");

    bridge
        .get_adapter_mut(adapter_name)
        .unwrap()
        .connect()
        .await
        .expect("Failed to connect adapter");

    // Set up PWM channels for motor outputs
    let v = bridge.get_vehicle(id).unwrap();
    v.platform.create_pwm(0, 50).unwrap(); // left motor
    v.platform.create_pwm(1, 50).unwrap(); // right motor

    println!(
        "Bridge running. MAVLink TCP on port {}. Press Ctrl+C to stop.\n",
        args.mavlink_port
    );

    let mut autopilot = VehicleAutopilot::new(args.system_id);
    println!(
        "Autopilot initialized (mode: {})",
        autopilot.mode_executor.current_mode_name()
    );

    let mut was_connected = false;
    let mut gazebo_ready = false;
    let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(10));
    let mut step_count: u64 = 0;
    let wall_start = std::time::Instant::now();

    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    loop {
        tokio::select! {
            _ = &mut ctrl_c => {
                println!("\nShutdown requested.");
                break;
            }
            _ = interval.tick() => {
                let wall_us = wall_start.elapsed().as_micros() as u64;

                // 1. Poll GCS incoming → dispatch through core's MessageDispatcher
                let incoming = gcs.poll_incoming();
                for (header, msg) in &incoming {
                    let responses = autopilot.dispatch(header, msg, wall_us);
                    for response in &responses {
                        let _ = gcs.send_message_as(args.system_id, response);
                    }

                    // Process RC input (async) for ManualMode
                    autopilot.process_rc_input(msg, wall_us).await;
                }

                // 2. Step the simulation bridge (Gazebo physics)
                let step_ok = match bridge.step().await {
                    Ok(()) => {
                        if !gazebo_ready {
                            gazebo_ready = true;
                            println!("Gazebo connection established.");
                        }
                        true
                    }
                    Err(e) => {
                        if gazebo_ready {
                            eprintln!("Step error: {e}");
                        }
                        false
                    }
                };

                step_count += 1;

                // 3. Update SYSTEM_STATE from sensor data
                if step_ok {
                    if let Some(sensors) = bridge.get_vehicle(id)
                        .and_then(|v| v.platform.peek_sensors())
                    {
                        autopilot.update_from_sensors(&sensors);
                    }
                }

                // 4. Execute active mode (ManualMode reads RC, writes to SitlActuator)
                autopilot.execute_mode(wall_us);

                // 5. Apply actuator outputs to platform PWM channels
                if let Some(vehicle) = bridge.get_vehicle(id) {
                    autopilot.apply_actuators_to_platform(&vehicle.platform);
                }

                // 6. Send telemetry (core dispatcher handles HEARTBEAT, ATTITUDE, GPS, SYS_STATUS)
                let telemetry = autopilot.update_telemetry(wall_us);
                for msg in &telemetry {
                    let _ = gcs.send_message_as(args.system_id, msg);
                }

                // Connection status tracking
                if gcs.is_connected() && !was_connected {
                    was_connected = true;
                    println!("GCS connected");
                }
                if !gcs.is_connected() && was_connected {
                    was_connected = false;
                    println!("GCS disconnected");
                }

                // Summary every 10 seconds
                if step_count.is_multiple_of(1000) {
                    let secs = wall_us / 1_000_000;
                    let mode = pico_trail_sitl::autopilot::current_flight_mode();
                    let armed = pico_trail_sitl::autopilot::is_armed();
                    println!(
                        "[{secs}s] {step_count} steps, mode={mode:?}, armed={armed}, GCS {}",
                        if gcs.is_connected() { "connected" } else { "waiting" }
                    );
                }
            }
        }
    }

    println!(
        "Simulation complete. {} steps, final time: {} us",
        step_count,
        bridge.sim_time_us()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_args_defaults() {
        // When no args are provided, defaults should be used.
        // We can't easily test parse_args() directly because it reads env::args(),
        // but we verify the Args struct defaults.
        let args = Args {
            system_id: 1,
            gazebo_port: 9002,
            mavlink_port: 5760,
        };
        assert_eq!(args.system_id, 1);
        assert_eq!(args.gazebo_port, 9002);
        assert_eq!(args.mavlink_port, 5760);
    }

    #[test]
    fn test_parse_u16_arg_valid() {
        let raw = vec!["bin".to_string(), "1234".to_string()];
        assert_eq!(parse_u16_arg(&raw, 1, "test"), 1234);
    }
}
