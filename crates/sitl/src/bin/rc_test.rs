//! RC override test sender for verifying SITL manual control.
//!
//! Connects to gazebo_bridge via TCP and sends:
//! 1. HEARTBEAT (GCS)
//! 2. ARM command
//! 3. RC_CHANNELS_OVERRIDE with ramping throttle
//!
//! Usage:
//!   cargo run -p pico_trail_sitl --bin rc_test [PORT]
//!
//! Default port: 5760 (gazebo_bridge default)

use std::io::{Cursor, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use mavlink::common::*;
use mavlink::peek_reader::PeekReader;
use mavlink::MavHeader;

fn main() {
    let port: u16 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(5760);

    println!("=== RC Override Test ===");
    println!("Connecting to 127.0.0.1:{port}...");

    let mut stream = TcpStream::connect(format!("127.0.0.1:{port}"))
        .expect("Failed to connect to gazebo_bridge");
    stream.set_nonblocking(true).unwrap();
    stream.set_nodelay(true).unwrap();
    println!("Connected!\n");

    let mut sequence: u8 = 0;
    let mut read_buf = Vec::with_capacity(4096);
    let mut tmp = [0u8; 1024];

    // Phase 1: Send heartbeats for 2 seconds to establish connection
    println!("Phase 1: Sending heartbeats...");
    let phase_start = Instant::now();
    while phase_start.elapsed() < Duration::from_secs(2) {
        send_heartbeat(&mut stream, &mut sequence);
        drain_incoming(&mut stream, &mut read_buf, &mut tmp);
        std::thread::sleep(Duration::from_millis(100));
    }

    // Phase 2: Send ARM command
    println!("Phase 2: Sending ARM command...");
    send_arm(&mut stream, &mut sequence);
    std::thread::sleep(Duration::from_millis(500));
    drain_incoming(&mut stream, &mut read_buf, &mut tmp);

    // Phase 3: Send RC overrides with varying throttle
    println!("Phase 3: Sending RC overrides (10 seconds)...");
    println!("  Throttle ramps: neutral(1500) -> forward(1200) -> full(1000) -> neutral(1500)\n");

    let phase_start = Instant::now();
    let mut last_hb = Instant::now();
    let mut step = 0u32;

    loop {
        let elapsed = phase_start.elapsed();
        if elapsed > Duration::from_secs(10) {
            break;
        }

        // Vary throttle over time
        // 0-2s: neutral (1500), 2-5s: forward (1200), 5-8s: full forward (1000), 8-10s: neutral
        let secs = elapsed.as_secs_f32();
        let (steering_pwm, throttle_pwm) = if secs < 2.0 {
            (1500u16, 1500u16) // neutral
        } else if secs < 5.0 {
            (1500, 1200) // half forward
        } else if secs < 8.0 {
            (1600, 1000) // full forward + slight right
        } else {
            (1500, 1500) // neutral
        };

        send_rc_override(&mut stream, &mut sequence, steering_pwm, throttle_pwm);

        if step.is_multiple_of(50) {
            println!(
                "  [{:.1}s] steering={steering_pwm} throttle={throttle_pwm}",
                secs
            );
        }

        // Heartbeat at 1Hz
        if last_hb.elapsed() >= Duration::from_secs(1) {
            send_heartbeat(&mut stream, &mut sequence);
            last_hb = Instant::now();
        }

        drain_incoming(&mut stream, &mut read_buf, &mut tmp);
        step += 1;
        std::thread::sleep(Duration::from_millis(20)); // 50 Hz
    }

    // Phase 4: Disarm
    println!("\nPhase 4: Sending DISARM command...");
    send_disarm(&mut stream, &mut sequence);
    std::thread::sleep(Duration::from_millis(500));

    println!("Done. Check gazebo_bridge logs for [RC_OVERRIDE] and [PWM] output.");
}

fn send_v2(stream: &mut TcpStream, sequence: &mut u8, msg: &MavMessage) {
    let header = MavHeader {
        system_id: 255, // GCS
        component_id: 190,
        sequence: *sequence,
    };
    *sequence = sequence.wrapping_add(1);

    let mut buf = Cursor::new(Vec::with_capacity(280));
    mavlink::write_v2_msg(&mut buf, header, msg).expect("Failed to serialize");
    let bytes = buf.into_inner();
    let _ = stream.write_all(&bytes);
}

fn send_heartbeat(stream: &mut TcpStream, sequence: &mut u8) {
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });
    send_v2(stream, sequence, &msg);
}

fn send_arm(stream: &mut TcpStream, sequence: &mut u8) {
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 1.0, // arm
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    send_v2(stream, sequence, &msg);
    println!("  ARM command sent");
}

fn send_disarm(stream: &mut TcpStream, sequence: &mut u8) {
    let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 0.0,     // disarm
        param2: 21196.0, // force
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    send_v2(stream, sequence, &msg);
    println!("  DISARM command sent");
}

fn send_rc_override(
    stream: &mut TcpStream,
    sequence: &mut u8,
    steering_pwm: u16,
    throttle_pwm: u16,
) {
    let msg = MavMessage::RC_CHANNELS_OVERRIDE(RC_CHANNELS_OVERRIDE_DATA {
        target_system: 1,
        target_component: 1,
        chan1_raw: steering_pwm,
        chan2_raw: 65535, // ignore
        chan3_raw: throttle_pwm,
        chan4_raw: 65535,
        chan5_raw: 65535,
        chan6_raw: 65535,
        chan7_raw: 65535,
        chan8_raw: 65535,
        chan9_raw: 65535,
        chan10_raw: 65535,
        chan11_raw: 65535,
        chan12_raw: 65535,
        chan13_raw: 65535,
        chan14_raw: 65535,
        chan15_raw: 65535,
        chan16_raw: 65535,
        chan17_raw: 65535,
        chan18_raw: 65535,
    });
    send_v2(stream, sequence, &msg);
}

fn drain_incoming(stream: &mut TcpStream, read_buf: &mut Vec<u8>, tmp: &mut [u8]) {
    loop {
        match stream.read(tmp) {
            Ok(0) => {
                eprintln!("Connection closed by server");
                std::process::exit(1);
            }
            Ok(n) => read_buf.extend_from_slice(&tmp[..n]),
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
            Err(e) => {
                eprintln!("Read error: {e}");
                std::process::exit(1);
            }
        }
    }

    // Parse and log response messages
    loop {
        let magic_pos = read_buf.iter().position(|&b| b == 0xFE || b == 0xFD);
        let Some(pos) = magic_pos else {
            read_buf.clear();
            break;
        };
        if pos > 0 {
            read_buf.drain(..pos);
        }
        if read_buf.len() < 2 {
            break;
        }

        let magic = read_buf[0];
        let payload_len = read_buf[1] as usize;
        let frame_size = if magic == 0xFD {
            let base = 12 + payload_len;
            if read_buf.len() >= 3 && (read_buf[2] & 0x01) != 0 {
                base + 13
            } else {
                base
            }
        } else {
            8 + payload_len
        };

        if read_buf.len() < frame_size {
            break;
        }

        let frame = read_buf[..frame_size].to_vec();
        let cursor = Cursor::new(&frame[..]);
        let mut reader = PeekReader::new(cursor);
        let result = if magic == 0xFD {
            mavlink::read_v2_msg::<MavMessage, _>(&mut reader)
        } else {
            mavlink::read_v1_msg::<MavMessage, _>(&mut reader)
        };

        if let Ok((_hdr, MavMessage::COMMAND_ACK(ack))) = &result {
            println!("  ACK: cmd={:?} result={:?}", ack.command, ack.result);
        }
        read_buf.drain(..frame_size);
    }
}
