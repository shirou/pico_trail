//! Minimal MAVLink heartbeat sender over TCP for diagnosing Mission Planner connectivity.
//!
//! Usage:
//!   cargo run -p pico_trail_sitl --bin mavlink_test [PORT]
//!
//! Then connect Mission Planner via "TCP" mode to 127.0.0.1:PORT (default: 14550).

use std::io::{Cursor, Read, Write};
use std::net::TcpListener;
use std::time::{Duration, Instant};

use mavlink::common::*;
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, Message};

fn main() {
    let port: u16 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(14550);

    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).expect("Failed to bind");
    let local_addr = listener.local_addr().unwrap();

    println!("=== MAVLink TCP Test Sender ===");
    println!("Listening on {local_addr}");
    println!("Connect Mission Planner via TCP to this address.\n");

    let (mut stream, addr) = listener.accept().expect("Failed to accept");
    stream.set_nonblocking(true).unwrap();
    stream.set_nodelay(true).unwrap();
    println!("Client connected from {addr}\n");

    let mut sequence: u8 = 0;
    let mut hb_count: u32 = 0;
    let mut read_buf = Vec::with_capacity(2048);
    let mut tmp = [0u8; 1024];

    // Send initial burst
    for _ in 0..3 {
        send_heartbeat(&mut stream, &mut sequence, &mut hb_count);
    }
    let mut last_heartbeat = Instant::now();

    loop {
        // Read available data
        loop {
            match stream.read(&mut tmp) {
                Ok(0) => {
                    println!("Connection closed.");
                    return;
                }
                Ok(n) => {
                    read_buf.extend_from_slice(&tmp[..n]);
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => {
                    eprintln!("Read error: {e}");
                    return;
                }
            }
        }

        // Parse buffered MAVLink frames
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
                // v2 signed: incompat_flags bit 0 adds 13-byte signature
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

            match result {
                Ok((hdr, msg)) => {
                    let ver = if magic == 0xFD { "v2" } else { "v1" };
                    println!(
                        "RX {ver} msg_id={} sys={} comp={}",
                        msg.message_id(),
                        hdr.system_id,
                        hdr.component_id,
                    );
                    read_buf.drain(..frame_size);
                }
                Err(_) => {
                    let hex: Vec<String> =
                        frame.iter().take(16).map(|b| format!("{b:02x}")).collect();
                    println!("RX PARSE FAILED: {}", hex.join(" "));
                    read_buf.drain(..frame_size);
                }
            }
        }

        // Heartbeat at 1Hz
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            send_heartbeat(&mut stream, &mut sequence, &mut hb_count);
            last_heartbeat = Instant::now();
        }

        std::thread::sleep(Duration::from_millis(10));
    }
}

fn send_heartbeat(stream: &mut std::net::TcpStream, sequence: &mut u8, count: &mut u32) {
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: *sequence,
    };
    *sequence = sequence.wrapping_add(1);

    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GROUND_ROVER,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });

    let mut buf = Cursor::new(Vec::with_capacity(280));
    mavlink::write_v1_msg(&mut buf, header, &msg).expect("Failed to serialize");
    let bytes = buf.into_inner();

    *count += 1;
    println!("TX HEARTBEAT v1 #{count}");

    if let Err(e) = stream.write_all(&bytes) {
        eprintln!("Write error: {e}");
    }
}
