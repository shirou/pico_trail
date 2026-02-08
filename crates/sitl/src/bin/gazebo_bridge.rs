//! Multi-vehicle SITL bridge for Gazebo Harmonic.
//!
//! Spawns N rovers, each connected to Gazebo via a GazeboAdapter on
//! separate UDP port pairs, and exposes MAVLink telemetry for GCS.
//!
//! All vehicles share a single TCP port — Mission Planner auto-detects
//! them via distinct MAVLink system_id values.
//!
//! Usage:
//!   cargo run -p pico_trail_sitl --bin gazebo_bridge -- [OPTIONS]
//!
//! Options:
//!   -n, --count <N>            Number of vehicles (default: 3)
//!   --gazebo-port-base <PORT>  Gazebo fdm_port_in for vehicle 1 (default: 9002)
//!   --port-stride <N>          Port offset between vehicles (default: 10)
//!   --mavlink-port <PORT>      MAVLink TCP port for GCS (default: 14550)

use std::env;
use std::process;

use pico_trail_sitl::adapter::{GazeboAdapter, GazeboConfig};
use pico_trail_sitl::{GcsLink, SitlBridge, TimeMode, VehicleConfig, VehicleId, VehicleType};

struct Args {
    count: u8,
    gazebo_port_base: u16,
    port_stride: u16,
    mavlink_port: u16,
}

fn parse_args() -> Args {
    let mut args = Args {
        count: 3,
        gazebo_port_base: 9002,
        port_stride: 10,
        mavlink_port: 14550,
    };

    let raw: Vec<String> = env::args().collect();
    let mut i = 1;
    while i < raw.len() {
        match raw[i].as_str() {
            "-n" | "--count" => {
                i += 1;
                args.count = parse_u16_arg(&raw, i, "count") as u8;
            }
            "--gazebo-port-base" => {
                i += 1;
                args.gazebo_port_base = parse_u16_arg(&raw, i, "gazebo-port-base");
            }
            "--port-stride" => {
                i += 1;
                args.port_stride = parse_u16_arg(&raw, i, "port-stride");
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

    if args.count == 0 {
        eprintln!("Error: count must be at least 1");
        process::exit(1);
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
         \x20 -n, --count <N>            Number of vehicles (default: 3)\n\
         \x20 --gazebo-port-base <PORT>  Gazebo fdm_port_in for vehicle 1 (default: 9002)\n\
         \x20 --port-stride <N>          Port offset between vehicles (default: 10)\n\
         \x20 --mavlink-port <PORT>      MAVLink TCP port for GCS (default: 14550)\n\
         \x20 -h, --help                 Show this help"
    );
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args = parse_args();

    println!("=== pico_trail Gazebo Bridge ===");
    println!(
        "Vehicles: {}, Gazebo port base: {}, stride: {}, MAVLink TCP: {}",
        args.count, args.gazebo_port_base, args.port_stride, args.mavlink_port
    );
    println!();

    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Scaled { factor: 1.0 });

    // Single GCS link for all vehicles
    let mut gcs = GcsLink::new(args.mavlink_port).expect("Failed to bind MAVLink TCP port");

    for i in 1..=args.count {
        let id = VehicleId(i);
        let adapter_name = format!("gazebo{i}");

        let gazebo_port = args.gazebo_port_base + (i as u16 - 1) * args.port_stride;

        let config = GazeboConfig {
            gazebo_addr: format!("127.0.0.1:{gazebo_port}").parse().unwrap(),
            timeout_ms: 100,
            ..Default::default()
        };
        let adapter = GazeboAdapter::new(&adapter_name, id, config);
        bridge
            .register_adapter(Box::new(adapter))
            .expect("Failed to register adapter");

        let vehicle_config = VehicleConfig::new(id, VehicleType::Rover);
        bridge
            .spawn_vehicle(vehicle_config)
            .expect("Failed to spawn vehicle");
        bridge
            .assign_vehicle_to_adapter(id, &adapter_name)
            .expect("Failed to assign vehicle");

        bridge
            .get_adapter_mut(&adapter_name)
            .unwrap()
            .connect()
            .await
            .expect("Failed to connect adapter");

        // Set up PWM channels for motor outputs
        let v = bridge.get_vehicle(id).unwrap();
        v.platform.create_pwm(0, 50).unwrap(); // left motor
        v.platform.create_pwm(1, 50).unwrap(); // right motor

        // Register vehicle with GCS link
        gcs.register_vehicle(i);

        println!("Vehicle {i}: gazebo_port={gazebo_port}");
    }

    println!(
        "\nBridge running. MAVLink TCP on port {}. Press Ctrl+C to stop.\n",
        args.mavlink_port
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

                // Use wall-clock for GCS so heartbeats stay at real 1 Hz
                let wall_us = wall_start.elapsed().as_micros() as u64;

                // Poll once, sends heartbeats for all vehicles
                gcs.poll_and_heartbeat(wall_us);

                if step_ok {
                    for i in 1..=args.count {
                        let vehicle = bridge.get_vehicle(VehicleId(i)).unwrap();
                        if let Some(sensors) = vehicle.platform.peek_sensors() {
                            gcs.send_telemetry(i, &sensors, wall_us);
                        }
                    }
                }

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
                    println!(
                        "[{secs}s] {step_count} steps, GCS {}",
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
