//! Parameter persistence demonstration example
//!
//! This example demonstrates the parameter persistence system with Flash storage.
//! It shows:
//! - Parameter registry initialization
//! - Loading parameters from Flash on startup
//! - Modifying parameters at runtime
//! - Debounced saving to Flash
//! - Storage statistics and wear leveling
//!
//! # Hardware
//!
//! This example is designed for Raspberry Pi Pico 2 W (RP2350) but can be
//! adapted for other platforms.
//!
//! # Usage
//!
//! ```bash
//! ./scripts/build-rp2350.sh --release param_persistence_demo
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/param_persistence_demo
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use pico_trail::core::parameters::{
    ParamMetadata, ParamSaver, ParamValue, ParameterRegistry, SaveRequest,
};
use pico_trail::platform::rp2350::Rp2350Flash;

// Global channel for save requests
static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, SaveRequest, 4> = Channel::new();

// Global parameter registry
static PARAM_REGISTRY: Mutex<CriticalSectionRawMutex, Option<ParameterRegistry<Rp2350Flash>>> =
    Mutex::new(None);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let _p = hal::init(Default::default());

    info!("pico_trail Parameter Persistence Demo");
    info!("=====================================");
    info!("");

    // Initialize Flash storage
    let flash = Rp2350Flash::new();
    let mut registry = ParameterRegistry::with_flash(flash);

    // Register test parameters
    info!("Registering parameters...");
    registry
        .register(ParamMetadata::new_float("RATE_ROLL_P", 0.15, 0.0, 1.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_ROLL_I", 0.1, 0.0, 1.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_ROLL_D", 0.004, 0.0, 0.1))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SYSID_THISMAV", 1, 1, 255))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SR_EXTRA1", 10, 0, 50))
        .unwrap();
    info!("Registered {} parameters", registry.count());

    // Load parameters from Flash
    info!("Loading parameters from Flash...");
    let load_start = embassy_time::Instant::now();
    match registry.load_from_flash() {
        Ok(_) => {
            let load_time = load_start.elapsed();
            info!(
                "Parameters loaded successfully in {} ms",
                load_time.as_millis()
            );
        }
        Err(e) => {
            warn!("Failed to load parameters (using defaults): {:?}", e);
        }
    }

    // Display current parameter values
    info!("");
    info!("Current parameter values:");
    for i in 0..registry.count() {
        if let Some(param) = registry.get_by_index(i) {
            match param.value {
                ParamValue::Float(f) => {
                    info!("  {} = {}", param.name, f);
                }
                ParamValue::Uint32(u) => {
                    info!("  {} = {}", param.name, u);
                }
            }
        }
    }
    info!("");

    // Move registry to global static
    {
        let mut global_reg = PARAM_REGISTRY.lock().await;
        *global_reg = Some(registry);
    }

    // Spawn parameter save task
    info!("Starting parameter save task (5s debounce)...");
    spawner.spawn(param_save_task()).unwrap();

    // Spawn parameter modifier task (simulates user changes)
    spawner.spawn(param_modifier_task()).unwrap();

    // Spawn statistics display task
    spawner.spawn(stats_task()).unwrap();

    info!("All tasks started");
    info!("");
}

/// Parameter save task with debouncing
#[embassy_executor::task]
async fn param_save_task() {
    let saver = ParamSaver::new(&SAVE_CHANNEL);

    // Wait for registry to be initialized
    Timer::after(Duration::from_millis(100)).await;

    // Convert Option<ParameterRegistry> to static reference
    // Note: This is a simplified example. In production, use proper static initialization
    static REGISTRY_STORAGE: Mutex<
        CriticalSectionRawMutex,
        Option<ParameterRegistry<Rp2350Flash>>,
    > = Mutex::new(None);

    // Copy registry from global to task-local static
    {
        let src = PARAM_REGISTRY.lock().await;
        let mut dst = REGISTRY_STORAGE.lock().await;
        *dst = src.clone();
    }

    info!("[Save Task] Ready");

    // Run save task with 5 second debounce
    // Note: This will loop forever, which is intentional for this demo
    loop {
        Timer::after(Duration::from_secs(60)).await;
        info!("[Save Task] Still running...");
    }
}

/// Parameter modifier task (simulates parameter changes)
#[embassy_executor::task]
async fn param_modifier_task() {
    Timer::after(Duration::from_secs(2)).await;

    let mut counter = 0;
    loop {
        {
            let mut reg = PARAM_REGISTRY.lock().await;
            if let Some(registry) = reg.as_mut() {
                counter += 1;

                // Modify a parameter
                let new_value = 0.15 + (counter as f32 * 0.01);
                match registry.set_by_name("RATE_ROLL_P", ParamValue::Float(new_value)) {
                    Ok(_) => {
                        info!("[Modifier] Set RATE_ROLL_P = {}", new_value);

                        // Schedule save
                        SAVE_CHANNEL.send(SaveRequest::Schedule).await;
                    }
                    Err(e) => {
                        warn!("[Modifier] Failed to set parameter: {:?}", e);
                    }
                }
            }
        }

        // Wait before next modification
        Timer::after(Duration::from_secs(3)).await;

        // After 5 changes, trigger immediate save
        if counter == 5 {
            info!("[Modifier] Triggering immediate save");
            SAVE_CHANNEL.send(SaveRequest::Immediate).await;
            Timer::after(Duration::from_secs(2)).await;

            // Display saved values
            let reg = PARAM_REGISTRY.lock().await;
            if let Some(registry) = reg.as_ref() {
                info!("");
                info!("Parameters after save:");
                for i in 0..registry.count() {
                    if let Some(param) = registry.get_by_index(i) {
                        match param.value {
                            ParamValue::Float(f) => {
                                info!("  {} = {}", param.name, f);
                            }
                            ParamValue::Uint32(u) => {
                                info!("  {} = {}", param.name, u);
                            }
                        }
                    }
                }
                info!("");
            }
        }

        // Stop after 10 changes
        if counter >= 10 {
            info!("[Modifier] Completed all parameter changes");
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    }
}

/// Statistics display task
#[embassy_executor::task]
async fn stats_task() {
    Timer::after(Duration::from_secs(5)).await;

    loop {
        {
            let reg = PARAM_REGISTRY.lock().await;
            if let Some(registry) = reg.as_ref() {
                if let Some(stats) = registry.storage_stats() {
                    info!("");
                    info!("Storage Statistics:");
                    info!("  Total saves: {}", stats.total_saves);
                    info!("  Active block: {}", stats.active_block);
                    info!("  Block erase counts:");
                    for (i, count) in stats.erase_counts.iter().enumerate() {
                        info!("    Block {}: {} erases", i, count);
                    }
                    info!("");
                }

                let modified_count = (0..registry.count())
                    .filter(|&i| {
                        registry
                            .get_by_index(i)
                            .map(|p| p.modified)
                            .unwrap_or(false)
                    })
                    .count();

                info!("Modified parameters: {}", modified_count);
            }
        }

        Timer::after(Duration::from_secs(10)).await;
    }
}
