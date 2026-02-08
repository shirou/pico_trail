//! Multi-vehicle SITL example demonstrating 3 rovers with MAVLink GCS telemetry.
//!
//! Creates a bridge with 3 lightweight adapters and vehicles, assigns each
//! vehicle to its adapter, and runs a continuous loop sending MAVLink telemetry
//! on a single TCP port for Mission Planner integration.
//!
//! Mission Planner auto-detects all 3 vehicles via their distinct system_id.
//!
//! Run with: `cargo run -p pico_trail_sitl --example multi_vehicle`

use pico_trail_sitl::{
    GcsLink, LightweightAdapter, LightweightConfig, SitlBridge, TimeMode, VehicleConfig, VehicleId,
    VehicleType,
};

const VEHICLE_COUNT: u8 = 3;
const MAVLINK_PORT: u16 = 14550;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    println!("=== pico_trail SITL Multi-Vehicle Example ===\n");

    // 1. Create bridge with realtime timing (Mission Planner expects wall-clock time)
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Scaled { factor: 1.0 });

    // Single GCS link for all vehicles
    let mut gcs = GcsLink::new(MAVLINK_PORT).expect("Failed to bind MAVLink TCP port");

    // 2. Register adapters, spawn vehicles, and assign them
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

        gcs.register_vehicle(i);

        println!("Vehicle {i}: spawned, adapter '{adapter_name}'");
    }
    println!("\nWaiting for Mission Planner TCP connection on port {MAVLINK_PORT}...");
    println!("Press Ctrl+C to stop.\n");

    let mut was_connected = false;

    // 3. Run continuous simulation loop at 100 Hz
    let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(10));
    let mut step_count: u64 = 0;

    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    loop {
        tokio::select! {
            _ = &mut ctrl_c => {
                println!("\nShutdown requested.");
                break;
            }
            _ = interval.tick() => {
                bridge.step().await.expect("Step failed");
                step_count += 1;

                let sim_time = bridge.sim_time_us();

                gcs.poll_and_heartbeat(sim_time);

                for i in 1..=VEHICLE_COUNT {
                    let vehicle = bridge.get_vehicle(VehicleId(i)).unwrap();
                    if let Some(sensors) = vehicle.platform.peek_sensors() {
                        gcs.send_telemetry(i, &sensors, sim_time);
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

                // Print summary every 10 seconds (1000 steps at 100 Hz)
                if step_count.is_multiple_of(1000) {
                    let secs = sim_time / 1_000_000;
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
