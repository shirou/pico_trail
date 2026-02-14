//! GCS (Ground Control Station) communication over MAVLink TCP.
//!
//! Provides a TCP transport for sending and receiving MAVLink messages.
//! All telemetry (HEARTBEAT, ATTITUDE, GPS, SYS_STATUS) is handled by
//! core's `TelemetryStreamer` via `VehicleAutopilot::update_telemetry()`.
//!
//! Mission Planner connects via its "TCP" connection mode as a TCP client.

use std::io::{self, Cursor, Read, Write};
use std::net::{TcpListener, TcpStream};

use mavlink::common::*;
use mavlink::peek_reader::PeekReader;
use mavlink::MavHeader;
#[cfg(test)]
use mavlink::Message;
use num_traits::FromPrimitive;

/// MAVLink TCP transport for GCS communication.
///
/// Acts as a TCP server: listens for a single incoming connection from
/// Mission Planner, then sends/receives MAVLink v1 messages.
pub struct GcsLink {
    listener: TcpListener,
    stream: Option<TcpStream>,
    component_id: u8,
    sequence: u8,
    /// Accumulated bytes from TCP stream, parsed into MAVLink frames.
    read_buf: Vec<u8>,
}

impl GcsLink {
    /// Create a new GCS link listening on `0.0.0.0:{port}` for TCP connections.
    pub fn new(port: u16) -> io::Result<Self> {
        let listener = TcpListener::bind(format!("0.0.0.0:{port}"))?;
        listener.set_nonblocking(true)?;

        Ok(Self {
            listener,
            stream: None,
            component_id: 1, // MAV_COMP_ID_AUTOPILOT1
            sequence: 0,
            read_buf: Vec::with_capacity(2048),
        })
    }

    /// Return the local socket address (useful for tests with OS-assigned ports).
    pub fn local_addr(&self) -> io::Result<std::net::SocketAddr> {
        self.listener.local_addr()
    }

    /// Accept a pending TCP connection if none is active.
    fn try_accept(&mut self) {
        if self.stream.is_some() {
            return;
        }
        match self.listener.accept() {
            Ok((stream, addr)) => {
                stream.set_nonblocking(true).ok();
                stream.set_nodelay(true).ok();
                self.stream = Some(stream);
                println!("  [GCS] TCP client connected from {addr}");
            }
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {}
            Err(_) => {}
        }
    }

    /// Read available bytes from the TCP stream into the read buffer.
    fn read_from_stream(&mut self) {
        if self.stream.is_none() {
            return;
        }

        let mut tmp = [0u8; 1024];
        loop {
            let stream = self.stream.as_mut().unwrap();
            match stream.read(&mut tmp) {
                Ok(0) => {
                    println!("  [GCS] TCP connection closed");
                    self.stream = None;
                    self.read_buf.clear();
                    return;
                }
                Ok(n) => {
                    self.read_buf.extend_from_slice(&tmp[..n]);
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => return,
                Err(e) => {
                    eprintln!("  [GCS] TCP read error: {e}");
                    self.stream = None;
                    self.read_buf.clear();
                    return;
                }
            }
        }
    }

    /// Extract complete MAVLink frames from the read buffer.
    fn parse_buffered_messages(&mut self) -> Vec<(MavHeader, MavMessage)> {
        let mut messages = Vec::new();

        loop {
            // Find next magic byte (0xFE = v1, 0xFD = v2)
            let magic_pos = self.read_buf.iter().position(|&b| b == 0xFE || b == 0xFD);
            let Some(pos) = magic_pos else {
                self.read_buf.clear();
                break;
            };

            // Discard bytes before magic
            if pos > 0 {
                self.read_buf.drain(..pos);
            }

            // Need at least magic + length
            if self.read_buf.len() < 2 {
                break;
            }

            let magic = self.read_buf[0];
            let payload_len = self.read_buf[1] as usize;

            // Calculate total frame size
            let frame_size = if magic == 0xFD {
                // v2: 10 header + payload + 2 CRC
                let base = 12 + payload_len;
                // If incompat_flags bit 0 is set, a 13-byte signature is appended
                if self.read_buf.len() >= 3 && (self.read_buf[2] & 0x01) != 0 {
                    base + 13
                } else {
                    base
                }
            } else {
                // v1: 6 header + payload + 2 CRC
                8 + payload_len
            };

            // Wait for complete frame
            if self.read_buf.len() < frame_size {
                break;
            }

            // Try to parse the frame
            let frame_bytes = self.read_buf[..frame_size].to_vec();
            if let Some(msg) = parse_mavlink_frame(&frame_bytes) {
                messages.push(msg);
                self.read_buf.drain(..frame_size);
            } else {
                self.read_buf.drain(..frame_size);
            }
        }

        messages
    }

    /// Try to receive and parse incoming MAVLink messages.
    pub fn poll_incoming(&mut self) -> Vec<(MavHeader, MavMessage)> {
        self.try_accept();
        self.read_from_stream();
        self.parse_buffered_messages()
    }

    /// Send a MAVLink v2 message on behalf of the given `system_id`.
    ///
    /// Does nothing if no GCS is connected.
    pub fn send_message_as(&mut self, system_id: u8, msg: &MavMessage) -> io::Result<()> {
        if self.stream.is_none() {
            return Ok(());
        }

        let header = MavHeader {
            system_id,
            component_id: self.component_id,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);

        let mut buf = Cursor::new(Vec::with_capacity(280));
        mavlink::write_v2_msg(&mut buf, header, msg)
            .map_err(|e| io::Error::other(format!("{e:?}")))?;

        let bytes = buf.into_inner();
        let stream = self.stream.as_mut().unwrap();
        match stream.write_all(&bytes) {
            Ok(()) => Ok(()),
            Err(e) => {
                eprintln!("  [GCS] TCP write error: {e}");
                self.stream = None;
                self.read_buf.clear();
                Err(e)
            }
        }
    }

    /// Whether a GCS client is connected.
    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }
}

// --- Helpers ---

/// Parse a single MAVLink frame (v1 or v2) from a byte slice.
fn parse_mavlink_frame(data: &[u8]) -> Option<(MavHeader, MavMessage)> {
    if data.is_empty() {
        return None;
    }
    let cursor = Cursor::new(data);
    let mut reader = PeekReader::new(cursor);
    let result = if data[0] == 0xFD {
        mavlink::read_v2_msg::<MavMessage, _>(&mut reader)
    } else {
        mavlink::read_v1_msg::<MavMessage, _>(&mut reader)
    };
    match result {
        Ok(msg) => Some(msg),
        Err(e) => {
            // Extract msg_id for debug
            let msg_id = if data[0] == 0xFD && data.len() >= 10 {
                data[7] as u32 | (data[8] as u32) << 8 | (data[9] as u32) << 16
            } else if data.len() >= 6 {
                data[5] as u32
            } else {
                9999
            };

            // Fallback for SET_POSITION_TARGET_GLOBAL_INT (msg_id=86).
            // Mission Planner sends type_mask with bits 12-15 set, which
            // `from_bits()` rejects. Parse raw bytes with `from_bits_truncate()`.
            if msg_id == 86 && data[0] == 0xFD && data.len() >= 63 {
                if let Some(parsed) = fallback_parse_set_position_target_global_int(data) {
                    eprintln!("  [GCS] Fallback parsed SET_POSITION_TARGET_GLOBAL_INT");
                    return Some(parsed);
                }
            }

            eprintln!("  [GCS] Parse error msg_id={}: {:?}", msg_id, e);
            None
        }
    }
}

/// Fallback parser for SET_POSITION_TARGET_GLOBAL_INT (msg_id=86).
///
/// The mavlink crate's `from_bits()` rejects `type_mask` values with
/// undefined bits (12-15) that Mission Planner sets. This function
/// manually extracts fields from the MAVLink v2 wire format and uses
/// `from_bits_truncate()` to ignore unknown bits.
fn fallback_parse_set_position_target_global_int(data: &[u8]) -> Option<(MavHeader, MavMessage)> {
    // MAVLink v2 header: [0]=magic, [1]=len, [2]=incompat, [3]=compat,
    // [4]=seq, [5]=sysid, [6]=compid, [7..10]=msgid
    let header = MavHeader {
        system_id: data[5],
        component_id: data[6],
        sequence: data[4],
    };

    // Payload starts at byte 10. Wire order (largest types first):
    let p = &data[10..];

    let time_boot_ms = u32::from_le_bytes(p[0..4].try_into().ok()?);
    let lat_int = i32::from_le_bytes(p[4..8].try_into().ok()?);
    let lon_int = i32::from_le_bytes(p[8..12].try_into().ok()?);
    let alt = f32::from_le_bytes(p[12..16].try_into().ok()?);
    let vx = f32::from_le_bytes(p[16..20].try_into().ok()?);
    let vy = f32::from_le_bytes(p[20..24].try_into().ok()?);
    let vz = f32::from_le_bytes(p[24..28].try_into().ok()?);
    let afx = f32::from_le_bytes(p[28..32].try_into().ok()?);
    let afy = f32::from_le_bytes(p[32..36].try_into().ok()?);
    let afz = f32::from_le_bytes(p[36..40].try_into().ok()?);
    let yaw = f32::from_le_bytes(p[40..44].try_into().ok()?);
    let yaw_rate = f32::from_le_bytes(p[44..48].try_into().ok()?);
    let type_mask_raw = u16::from_le_bytes(p[48..50].try_into().ok()?);
    let target_system = p[50];
    let target_component = p[51];
    let coordinate_frame_raw = p[52];

    let type_mask = PositionTargetTypemask::from_bits_truncate(type_mask_raw);
    let coordinate_frame =
        MavFrame::from_u32(coordinate_frame_raw as u32).unwrap_or(MavFrame::MAV_FRAME_GLOBAL);

    #[allow(deprecated)]
    let msg = MavMessage::SET_POSITION_TARGET_GLOBAL_INT(SET_POSITION_TARGET_GLOBAL_INT_DATA {
        time_boot_ms,
        lat_int,
        lon_int,
        alt,
        vx,
        vy,
        vz,
        afx,
        afy,
        afz,
        yaw,
        yaw_rate,
        type_mask,
        target_system,
        target_component,
        coordinate_frame,
    });

    Some((header, msg))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a HEARTBEAT message for a ground rover (basic, unarmed).
    fn build_heartbeat() -> MavMessage {
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GROUND_ROVER,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }

    #[test]
    fn test_gcs_link_creation() {
        let link = GcsLink::new(0);
        assert!(link.is_ok());
        let link = link.unwrap();
        assert!(!link.is_connected());
    }

    #[test]
    fn test_send_without_client_is_noop() {
        let mut link = GcsLink::new(0).unwrap();
        let msg = build_heartbeat();
        assert!(link.send_message_as(1, &msg).is_ok());
    }

    #[test]
    fn test_poll_incoming_no_client() {
        let mut link = GcsLink::new(0).unwrap();
        let messages = link.poll_incoming();
        assert!(messages.is_empty());
    }

    #[test]
    fn test_parse_skips_signed_v2_frame() {
        let payload_len: u8 = 4;
        let frame_size = 12 + payload_len as usize + 13;
        let mut frame = vec![0u8; frame_size];
        frame[0] = 0xFD;
        frame[1] = payload_len;
        frame[2] = 0x01; // MAVLINK_IFLAG_SIGNED

        // Append a valid v1 heartbeat after the signed frame
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        let msg = build_heartbeat();
        let mut buf = Cursor::new(Vec::with_capacity(64));
        mavlink::write_v1_msg(&mut buf, header, &msg).unwrap();
        let hb_bytes = buf.into_inner();

        let mut link = GcsLink::new(0).unwrap();
        link.read_buf.extend_from_slice(&frame);
        link.read_buf.extend_from_slice(&hb_bytes);

        let messages = link.parse_buffered_messages();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].1.message_id(), 0); // HEARTBEAT
    }

    #[test]
    fn test_tcp_loopback() {
        let mut link = GcsLink::new(0).unwrap();
        let link_addr = link.listener.local_addr().unwrap();

        // Simulate a GCS TCP client
        let mut gcs = TcpStream::connect(link_addr).unwrap();
        gcs.set_nonblocking(true).unwrap();

        std::thread::sleep(std::time::Duration::from_millis(50));

        link.try_accept();
        assert!(link.is_connected());

        // GCS sends a MAVLink v1 heartbeat
        let header = MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        };
        let msg = build_heartbeat();
        let mut buf = Cursor::new(Vec::with_capacity(280));
        mavlink::write_v1_msg(&mut buf, header, &msg).unwrap();
        gcs.write_all(&buf.into_inner()).unwrap();

        std::thread::sleep(std::time::Duration::from_millis(50));

        let received = link.poll_incoming();
        assert_eq!(received.len(), 1);
        assert_eq!(received[0].1.message_id(), 0);

        // Link sends a heartbeat as vehicle 1
        let result = link.send_message_as(1, &build_heartbeat());
        assert!(result.is_ok());

        // GCS should receive it
        std::thread::sleep(std::time::Duration::from_millis(50));
        let mut recv_buf = [0u8; 280];
        gcs.set_nonblocking(false).unwrap();
        gcs.set_read_timeout(Some(std::time::Duration::from_millis(100)))
            .unwrap();
        let n = gcs.read(&mut recv_buf).unwrap();
        assert!(n > 0);
        let parsed = parse_mavlink_frame(&recv_buf[..n]);
        assert!(parsed.is_some());
        let (hdr, _) = parsed.unwrap();
        assert_eq!(hdr.system_id, 1);
    }
}
