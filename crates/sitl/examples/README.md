# SITL Examples

## basic_sitl — Single Vehicle

Minimal single-vehicle example. One rover, one adapter, 100 steps.

```bash
cargo run -p pico_trail_sitl --example basic_sitl
```

## Multi-Vehicle

For multi-vehicle operation, use the launch script which starts one `gazebo_bridge` process per vehicle:

```bash
./scripts/sitl-multi-vehicle.sh
```

See the [SITL README](../README.md) for full details.
