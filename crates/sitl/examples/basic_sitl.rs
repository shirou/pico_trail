//! Basic SITL example demonstrating bridge setup and simulation loop.
//!
//! Creates a bridge, registers a LightweightAdapter, spawns a vehicle,
//! assigns it to the adapter, and runs a step loop printing sensor data.
//!
//! Run with: `cargo run -p pico_trail_sitl --example basic_sitl`

use pico_trail_sitl::{
    LightweightAdapter, LightweightConfig, SitlBridge, TimeMode, VehicleConfig, VehicleId,
    VehicleType,
};

#[tokio::main(flavor = "current_thread")]
async fn main() {
    println!("=== pico_trail SITL Basic Example ===\n");

    // 1. Create bridge with lockstep timing at 100 Hz
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Lockstep {
        step_size_us: 10_000,
    });

    // 2. Register a deterministic lightweight adapter
    let config = LightweightConfig {
        seed: Some(42),
        gps_noise_m: 0.5,
        step_size_us: 10_000,
        ..Default::default()
    };
    let adapter = LightweightAdapter::new("sim1", VehicleId(1), config);
    bridge
        .register_adapter(Box::new(adapter))
        .expect("Failed to register adapter");

    // 3. Spawn a rover vehicle
    let vehicle_config = VehicleConfig::new(VehicleId(1), VehicleType::Rover);
    let vehicle_id = bridge
        .spawn_vehicle(vehicle_config)
        .expect("Failed to spawn vehicle");
    println!("Spawned {vehicle_id}");

    // 4. Assign vehicle to adapter
    bridge
        .assign_vehicle_to_adapter(vehicle_id, "sim1")
        .expect("Failed to assign vehicle");

    // 5. Connect the adapter
    bridge
        .get_adapter_mut("sim1")
        .unwrap()
        .connect()
        .await
        .expect("Failed to connect adapter");
    println!("Adapter 'sim1' connected\n");

    // 6. Set up PWM channels for motor outputs
    {
        let vehicle = bridge.get_vehicle(vehicle_id).unwrap();
        vehicle.platform.create_pwm(0, 50).unwrap(); // left motor
        vehicle.platform.create_pwm(1, 50).unwrap(); // right motor
                                                     // Drive forward: both motors at 75% duty (maps to 0.5 normalized)
        vehicle.platform.set_pwm_duty(0, 0.75);
        vehicle.platform.set_pwm_duty(1, 0.75);
    }

    // 7. Run simulation loop
    println!("Running 100 steps (1 second at 100 Hz)...\n");
    for i in 0..100 {
        bridge.step().await.expect("Step failed");

        // Print sensor data every 20 steps
        if (i + 1) % 20 == 0 {
            let vehicle = bridge.get_vehicle(vehicle_id).unwrap();
            if let Some(sensors) = vehicle.platform.take_sensors() {
                println!("Step {:>3} | time: {} us", i + 1, sensors.timestamp_us);
                if let Some(imu) = &sensors.imu {
                    println!(
                        "  IMU: accel=[{:.2}, {:.2}, {:.2}] m/s^2",
                        imu.accel_mss[0], imu.accel_mss[1], imu.accel_mss[2]
                    );
                }
                if let Some(gps) = &sensors.gps {
                    println!(
                        "  GPS: lat={:.6}, lon={:.6}, speed={:.2} m/s",
                        gps.lat_deg, gps.lon_deg, gps.speed_ms
                    );
                }
                println!();
            }
        }
    }

    println!(
        "Simulation complete. Final time: {} us",
        bridge.sim_time_us()
    );
}
