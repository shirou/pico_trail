//! Multi-vehicle SITL example demonstrating 3 rovers with MAVLink GCS telemetry.
//!
//! Creates a bridge with 3 lightweight adapters and vehicles, assigns each
//! vehicle to its adapter, and runs a continuous loop sending MAVLink telemetry
//! on UDP ports 14551–14553 for Mission Planner integration.
//!
//! Run with: `cargo run -p pico_trail_sitl --example multi_vehicle`

use pico_trail_sitl::{
    GcsLink, LightweightAdapter, LightweightConfig, SitlBridge, TimeMode, VehicleConfig, VehicleId,
    VehicleType,
};

const VEHICLE_COUNT: u8 = 3;
const BASE_MAVLINK_PORT: u16 = 14551;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    println!("=== pico_trail SITL Multi-Vehicle Example ===\n");

    // 1. Create bridge with realtime timing (Mission Planner expects wall-clock time)
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Scaled { factor: 1.0 });

    // 2. Register adapters, spawn vehicles, and assign them
    let mut gcs_links = Vec::new();
    for i in 1..=VEHICLE_COUNT {
        let adapter_name = format!("sim{i}");
        let config = LightweightConfig {
            seed: Some(40 + i as u64),
            gps_noise_m: 0.5,
            step_size_us: 10_000,
            ..Default::default()
        };
        let adapter = LightweightAdapter::new(&adapter_name, VehicleId(i), config);
        bridge
            .register_adapter(Box::new(adapter))
            .expect("Failed to register adapter");

        let vehicle_config = VehicleConfig::new(VehicleId(i), VehicleType::Rover);
        bridge
            .spawn_vehicle(vehicle_config)
            .expect("Failed to spawn vehicle");
        bridge
            .assign_vehicle_to_adapter(VehicleId(i), &adapter_name)
            .expect("Failed to assign vehicle");

        bridge
            .get_adapter_mut(&adapter_name)
            .unwrap()
            .connect()
            .await
            .expect("Failed to connect adapter");

        // Set up PWM channels for motor outputs
        let vehicle = bridge.get_vehicle(VehicleId(i)).unwrap();
        vehicle.platform.create_pwm(0, 50).unwrap(); // left motor
        vehicle.platform.create_pwm(1, 50).unwrap(); // right motor
        vehicle.platform.set_pwm_duty(0, 0.75);
        vehicle.platform.set_pwm_duty(1, 0.75);

        // Create GCS link for MAVLink telemetry
        let port = BASE_MAVLINK_PORT + (i - 1) as u16;
        let gcs = GcsLink::new(i, port).expect("Failed to bind MAVLink UDP port");
        gcs_links.push(gcs);

        println!("Vehicle {i}: spawned, adapter '{adapter_name}', MAVLink port {port}");
    }
    println!(
        "\nWaiting for Mission Planner connections on UDP ports {BASE_MAVLINK_PORT}–{}...",
        BASE_MAVLINK_PORT + (VEHICLE_COUNT - 1) as u16
    );
    println!("Press Ctrl+C to stop.\n");

    // Track per-vehicle connection status for logging
    let mut was_connected = vec![false; VEHICLE_COUNT as usize];

    // 3. Run continuous simulation loop at 100 Hz
    let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(10));
    let mut step_count: u64 = 0;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nShutdown requested.");
                break;
            }
            _ = interval.tick() => {
                bridge.step().await.expect("Step failed");
                step_count += 1;

                let sim_time = bridge.sim_time_us();

                for i in 1..=VEHICLE_COUNT {
                    let idx = (i - 1) as usize;
                    let vehicle = bridge.get_vehicle(VehicleId(i)).unwrap();

                    if let Some(sensors) = vehicle.platform.peek_sensors() {
                        gcs_links[idx].update(&sensors, sim_time);
                    }

                    // Log when a GCS connects
                    if gcs_links[idx].is_connected() && !was_connected[idx] {
                        was_connected[idx] = true;
                        println!("Vehicle {i}: GCS connected");
                    }
                }

                // Print summary every 10 seconds (1000 steps at 100 Hz)
                if step_count.is_multiple_of(1000) {
                    let secs = sim_time / 1_000_000;
                    let connected: usize = was_connected.iter().filter(|c| **c).count();
                    println!(
                        "[{secs}s] {step_count} steps, {connected}/{VEHICLE_COUNT} GCS connected"
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
