/// Time synchronization mode for the SITL simulation.
#[derive(Debug, Clone)]
pub enum TimeMode {
    /// Simulation runs as fast as possible, no synchronization.
    FreeRunning,
    /// Simulation advances in discrete steps of `step_size_us` microseconds.
    Lockstep { step_size_us: u64 },
    /// Simulation runs at a scaled rate relative to wall-clock time.
    Scaled { factor: f32 },
}

impl Default for TimeMode {
    fn default() -> Self {
        Self::FreeRunning
    }
}
