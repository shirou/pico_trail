# SITL Examples

## multi_vehicle — Multi-Vehicle SITL with Mission Planner

Spawns 3 simulated rovers in a single `SitlBridge` and runs them continuously at 100 Hz with realtime pacing. Each vehicle gets its own `LightweightAdapter` (built-in 2D differential-drive physics) and a dedicated `GcsLink` that sends MAVLink telemetry over UDP so that Mission Planner can see all vehicles on the map.

### Vehicle Port Map

| Vehicle | MAVLink System ID | UDP Port | Adapter |
| ------- | ----------------- | -------- | ------- |
| 1       | 1                 | 14551    | sim1    |
| 2       | 2                 | 14552    | sim2    |
| 3       | 3                 | 14553    | sim3    |

Ports follow the formula `14550 + vehicle_id`.

### Run

```bash
cargo run -p pico_trail_sitl --example multi_vehicle
```

Press Ctrl+C to stop the simulation.

### Connecting Mission Planner

1. Start the `multi_vehicle` example.
2. In Mission Planner, open **CTRL+L** (Connection Options) or use the top-right connection drop-down.
3. Add each vehicle as a separate UDP connection:

   | Connection | Type | Host      | Port  |
   | ---------- | ---- | --------- | ----- |
   | Vehicle 1  | UDP  | 127.0.0.1 | 14551 |
   | Vehicle 2  | UDP  | 127.0.0.1 | 14552 |
   | Vehicle 3  | UDP  | 127.0.0.1 | 14553 |

4. After connecting, switch between vehicles using the **Vehicle** selector in Mission Planner's toolbar.

### Telemetry Messages

Each vehicle sends the following MAVLink v2 messages:

| Message              | Rate | Purpose                       |
| -------------------- | ---- | ----------------------------- |
| HEARTBEAT            | 1 Hz | Vehicle presence and type     |
| ATTITUDE             | 4 Hz | Roll, pitch, yaw, gyro rates  |
| GPS_RAW_INT          | 2 Hz | Raw GPS position and fix info |
| GLOBAL_POSITION_INT  | 2 Hz | Filtered position (map view)  |
| SYS_STATUS           | 1 Hz | Battery voltage and health    |

### Architecture

```text
Mission Planner
  |  UDP :14551          UDP :14552          UDP :14553
  v                      v                   v
+-----------+      +-----------+      +-----------+
| GcsLink 1 |      | GcsLink 2 |      | GcsLink 3 |
+-----+-----+      +-----+-----+      +-----+-----+
      |                   |                   |
| Vehicle 1 |      | Vehicle 2 |      | Vehicle 3 |
| SitlPlatform     | SitlPlatform     | SitlPlatform
+-----+-----+      +-----+-----+      +-----+-----+
      |                   |                   |
      v                   v                   v
  LightweightAdapter  LightweightAdapter  LightweightAdapter
  (2D physics)        (2D physics)        (2D physics)
      |                   |                   |
      +-------------------+-------------------+
                          |
                      SitlBridge
                   (TimeCoordinator: Scaled 1x)
```

Each `bridge.step()` call:

1. Steps all adapters (advance physics for each vehicle independently)
2. Collects sensor data (IMU, GPS, compass) from each adapter
3. Routes sensor data to the matching `SitlPlatform`
4. Reads actuator commands (PWM motor outputs) from each platform
5. Sends actuator commands back to the assigned adapter
6. Advances simulation time

After each step, `gcs_link.update()` sends rate-limited telemetry to any connected GCS.

### Future Work

The following features are not yet implemented:

- **Command input**: Parse incoming `RC_CHANNELS_OVERRIDE`, `SET_MODE`, `COMMAND_LONG` from Mission Planner.
- **Parameter protocol**: Respond to `PARAM_REQUEST_LIST` / `PARAM_SET`.
- **Mission protocol**: Handle `MISSION_*` messages for waypoint upload/download.

## basic_sitl — Single Vehicle

Minimal single-vehicle example. One rover, one adapter, 100 steps.

```bash
cargo run -p pico_trail_sitl --example basic_sitl
```
