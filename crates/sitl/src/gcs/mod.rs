//! GCS (Ground Control Station) communication over MAVLink TCP.
//!
//! Provides a single TCP link that multiplexes telemetry for multiple vehicles
//! (distinguished by MAVLink `system_id`) to Mission Planner or other GCS software.
//!
//! Mission Planner connects via its "TCP" connection mode as a TCP client.
//! Multiple vehicles appear automatically — no need for Ctrl-L or extra ports.

mod telemetry;

use std::io::{self, Cursor, Read, Write};
use std::net::{TcpListener, TcpStream};

use mavlink::common::*;
use mavlink::peek_reader::PeekReader;
use mavlink::MavHeader;
#[cfg(test)]
use mavlink::Message;

use crate::types::SensorData;
use telemetry::{build_heartbeat, build_sys_status, build_telemetry};

/// Rate intervals in microseconds for each telemetry message type.
const HEARTBEAT_INTERVAL_US: u64 = 1_000_000; // 1 Hz
const ATTITUDE_INTERVAL_US: u64 = 250_000; // 4 Hz
const POSITION_INTERVAL_US: u64 = 500_000; // 2 Hz
const SYS_STATUS_INTERVAL_US: u64 = 1_000_000; // 1 Hz

/// Per-vehicle rate-limiting state.
struct VehicleState {
    system_id: u8,
    last_heartbeat_us: u64,
    last_attitude_us: u64,
    last_position_us: u64,
    last_sys_status_us: u64,
}

/// MAVLink TCP link multiplexing telemetry for multiple vehicles.
///
/// Acts as a TCP server: listens for a single incoming connection from
/// Mission Planner, then sends/receives MAVLink v1 messages for all
/// registered vehicles (each with a unique `system_id`).
pub struct GcsLink {
    listener: TcpListener,
    stream: Option<TcpStream>,
    component_id: u8,
    sequence: u8,
    vehicles: Vec<VehicleState>,
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
            vehicles: Vec::new(),
            read_buf: Vec::with_capacity(2048),
        })
    }

    /// Register a vehicle so it gets heartbeats and telemetry.
    pub fn register_vehicle(&mut self, system_id: u8) {
        if self.vehicles.iter().any(|v| v.system_id == system_id) {
            return;
        }
        self.vehicles.push(VehicleState {
            system_id,
            last_heartbeat_us: 0,
            last_attitude_us: 0,
            last_position_us: 0,
            last_sys_status_us: 0,
        });
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
                // Parse failed (CRC error, unknown msg_id, etc.) — skip entire frame
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

    /// Send a MAVLink v1 message on behalf of the given `system_id`.
    ///
    /// Does nothing if no GCS is connected.
    fn send_message_as(&mut self, system_id: u8, msg: &MavMessage) -> io::Result<()> {
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
        mavlink::write_v1_msg(&mut buf, header, msg)
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

    /// Poll incoming messages, respond to GCS requests, and send heartbeats
    /// for all registered vehicles.
    pub fn poll_and_heartbeat(&mut self, sim_time_us: u64) {
        let incoming = self.poll_incoming();
        for (header, msg) in &incoming {
            self.handle_incoming(header, msg);
        }

        // Send heartbeat + sys_status for every registered vehicle
        let vehicle_ids: Vec<(u8, bool)> = self
            .vehicles
            .iter()
            .map(|v| {
                (
                    v.system_id,
                    sim_time_us - v.last_heartbeat_us >= HEARTBEAT_INTERVAL_US,
                )
            })
            .collect();

        for (sys_id, due) in vehicle_ids {
            if due {
                if let Some(v) = self.vehicles.iter_mut().find(|v| v.system_id == sys_id) {
                    v.last_heartbeat_us = sim_time_us;
                }
                let _ = self.send_message_as(sys_id, &build_heartbeat());
                let _ = self.send_message_as(sys_id, &build_sys_status(12600));
            }
        }
    }

    /// Handle an incoming MAVLink message from the GCS.
    fn handle_incoming(&mut self, header: &MavHeader, msg: &MavMessage) {
        // Determine which vehicle is being targeted
        let target_sys = match msg {
            MavMessage::COMMAND_LONG(cmd) => cmd.target_system,
            MavMessage::PARAM_REQUEST_LIST(data) => data.target_system,
            MavMessage::PARAM_REQUEST_READ(data) => data.target_system,
            _ => 0,
        };

        // Resolve system_id: 0 = broadcast → use first vehicle
        let sys_id = if target_sys == 0 {
            self.vehicles.first().map(|v| v.system_id).unwrap_or(1)
        } else {
            target_sys
        };

        match msg {
            MavMessage::COMMAND_LONG(cmd) => {
                self.handle_command_long(sys_id, cmd, header.system_id, header.component_id);
            }
            MavMessage::PARAM_REQUEST_LIST(_) => {
                self.send_all_params(sys_id);
            }
            MavMessage::PARAM_REQUEST_READ(data) => {
                let name = char_array_to_string(data.param_id.as_ref());
                if let Some(pv) = self.find_param(sys_id, &name, data.param_index) {
                    let _ = self.send_message_as(sys_id, &MavMessage::PARAM_VALUE(pv));
                }
            }
            #[allow(deprecated)]
            MavMessage::REQUEST_DATA_STREAM(data) => {
                #[allow(deprecated)]
                let _ = self.send_message_as(
                    sys_id,
                    &MavMessage::DATA_STREAM(DATA_STREAM_DATA {
                        stream_id: data.req_stream_id,
                        message_rate: data.req_message_rate,
                        on_off: data.start_stop,
                    }),
                );
            }
            _ => {}
        }
    }

    /// Handle COMMAND_LONG following firmware's dispatcher pattern.
    fn handle_command_long(
        &mut self,
        sys_id: u8,
        cmd: &COMMAND_LONG_DATA,
        sender_sys: u8,
        sender_comp: u8,
    ) {
        let result = match cmd.command {
            MavCmd::MAV_CMD_REQUEST_MESSAGE => {
                self.handle_request_message(sys_id, cmd);
                MavResult::MAV_RESULT_ACCEPTED
            }
            #[allow(deprecated)]
            MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => {
                let _ = self.send_message_as(sys_id, &build_autopilot_version());
                MavResult::MAV_RESULT_ACCEPTED
            }
            _ => MavResult::MAV_RESULT_UNSUPPORTED,
        };

        let ack = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command: cmd.command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: sender_sys,
            target_component: sender_comp,
        });
        let _ = self.send_message_as(sys_id, &ack);
    }

    /// Handle MAV_CMD_REQUEST_MESSAGE (cmd 512).
    fn handle_request_message(&mut self, sys_id: u8, cmd: &COMMAND_LONG_DATA) {
        const MSG_ID_AUTOPILOT_VERSION: u32 = 148;
        const MSG_ID_PROTOCOL_VERSION: u32 = 300;

        let message_id = cmd.param1 as u32;
        match message_id {
            MSG_ID_AUTOPILOT_VERSION => {
                let _ = self.send_message_as(sys_id, &build_autopilot_version());
            }
            MSG_ID_PROTOCOL_VERSION => {
                let _ = self.send_message_as(sys_id, &build_protocol_version());
            }
            _ => {}
        }
    }

    // --- Parameter handling ---

    fn send_all_params(&mut self, sys_id: u8) {
        let params = build_params(sys_id);
        let count = params.len() as u16;
        for (i, (name, value)) in params.iter().enumerate() {
            let _ = self.send_message_as(
                sys_id,
                &MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                    param_id: string_to_param_id(name).into(),
                    param_value: *value,
                    param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
                    param_count: count,
                    param_index: i as u16,
                }),
            );
        }
    }

    fn find_param(&self, sys_id: u8, name: &str, index: i16) -> Option<PARAM_VALUE_DATA> {
        let params = build_params(sys_id);
        let count = params.len() as u16;
        let entry = if index >= 0 {
            params.get(index as usize)
        } else {
            params.iter().find(|(n, _)| *n == name)
        };
        entry.map(|(n, v)| PARAM_VALUE_DATA {
            param_id: string_to_param_id(n).into(),
            param_value: *v,
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            param_count: count,
            param_index: params.iter().position(|(pn, _)| *pn == *n).unwrap_or(0) as u16,
        })
    }

    // --- Telemetry ---

    /// Send rate-limited sensor telemetry for a specific vehicle.
    pub fn send_telemetry(&mut self, system_id: u8, sensors: &SensorData, sim_time_us: u64) {
        let telem = build_telemetry(sensors);

        let Some(v) = self.vehicles.iter_mut().find(|v| v.system_id == system_id) else {
            return;
        };

        let send_attitude = sim_time_us - v.last_attitude_us >= ATTITUDE_INTERVAL_US;
        let send_position = sim_time_us - v.last_position_us >= POSITION_INTERVAL_US;
        let send_sys_status = sim_time_us - v.last_sys_status_us >= SYS_STATUS_INTERVAL_US;

        if send_attitude {
            v.last_attitude_us = sim_time_us;
        }
        if send_position {
            v.last_position_us = sim_time_us;
        }
        if send_sys_status {
            v.last_sys_status_us = sim_time_us;
        }

        // Send after releasing mutable borrow on self.vehicles
        if send_attitude {
            if let Some(msg) = &telem.attitude {
                let _ = self.send_message_as(system_id, msg);
            }
        }
        if send_position {
            if let Some(msg) = &telem.gps_raw {
                let _ = self.send_message_as(system_id, msg);
            }
            if let Some(msg) = &telem.global_pos {
                let _ = self.send_message_as(system_id, msg);
            }
        }
        if send_sys_status {
            let _ = self.send_message_as(system_id, &build_sys_status(12600));
        }
    }

    /// Whether a GCS client is connected.
    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }
}

// --- Message builders ---

/// Minimal parameter set for Mission Planner compatibility.
fn build_params(system_id: u8) -> Vec<(&'static str, f32)> {
    vec![("SYSID_THISMAV", system_id as f32)]
}

fn build_autopilot_version() -> MavMessage {
    let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
        | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
        | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
        | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT;

    #[allow(clippy::identity_op)]
    let flight_sw_version: u32 = (0 << 24) | (1 << 16) | (0 << 8) | 0;

    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities,
        uid: 0,
        flight_sw_version,
        middleware_sw_version: 0,
        os_sw_version: 0,
        board_version: 0,
        vendor_id: 0,
        product_id: 0,
        flight_custom_version: [0u8; 8],
        middleware_custom_version: [0u8; 8],
        os_custom_version: [0u8; 8],
        uid2: [0u8; 18],
    })
}

fn build_protocol_version() -> MavMessage {
    MavMessage::PROTOCOL_VERSION(PROTOCOL_VERSION_DATA {
        version: 200,
        min_version: 100,
        max_version: 200,
        spec_version_hash: [0u8; 8],
        library_version_hash: [0u8; 8],
    })
}

// --- Helpers ---

/// Parse a single MAVLink frame (v1 or v2) from a byte slice.
fn parse_mavlink_frame(data: &[u8]) -> Option<(MavHeader, MavMessage)> {
    if data.is_empty() {
        return None;
    }
    let cursor = Cursor::new(data);
    let mut reader = PeekReader::new(cursor);
    if data[0] == 0xFD {
        mavlink::read_v2_msg::<MavMessage, _>(&mut reader).ok()
    } else {
        mavlink::read_v1_msg::<MavMessage, _>(&mut reader).ok()
    }
}

fn string_to_param_id(name: &str) -> [u8; 16] {
    let mut id = [0u8; 16];
    let bytes = name.as_bytes();
    let len = bytes.len().min(16);
    id[..len].copy_from_slice(&bytes[..len]);
    id
}

fn char_array_to_string(id: &[u8]) -> String {
    let end = id.iter().position(|&b| b == 0).unwrap_or(id.len());
    String::from_utf8_lossy(&id[..end]).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcs_link_creation() {
        let link = GcsLink::new(0);
        assert!(link.is_ok());
        let link = link.unwrap();
        assert!(!link.is_connected());
    }

    #[test]
    fn test_register_vehicle() {
        let mut link = GcsLink::new(0).unwrap();
        link.register_vehicle(1);
        link.register_vehicle(2);
        link.register_vehicle(1); // duplicate, ignored
        assert_eq!(link.vehicles.len(), 2);
    }

    #[test]
    fn test_send_without_client_is_noop() {
        let mut link = GcsLink::new(0).unwrap();
        link.register_vehicle(1);
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
        link.register_vehicle(1);
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

    #[test]
    fn test_multi_vehicle_heartbeats() {
        let mut link = GcsLink::new(0).unwrap();
        link.register_vehicle(1);
        link.register_vehicle(2);
        link.register_vehicle(3);
        let link_addr = link.listener.local_addr().unwrap();

        let mut gcs = TcpStream::connect(link_addr).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));

        link.try_accept();
        assert!(link.is_connected());

        // Trigger heartbeats for all vehicles (time >= HEARTBEAT_INTERVAL_US)
        link.poll_and_heartbeat(HEARTBEAT_INTERVAL_US);

        // Give TCP stack time to deliver data
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Read with blocking + timeout
        gcs.set_nonblocking(false).unwrap();
        gcs.set_read_timeout(Some(std::time::Duration::from_secs(1)))
            .unwrap();

        let mut recv_buf = [0u8; 4096];
        let n = gcs.read(&mut recv_buf).expect("first read should succeed");
        assert!(n > 0);

        // Parse all received frames and collect system_ids
        let mut sys_ids = Vec::new();
        let mut offset = 0;
        while offset < n {
            if recv_buf[offset] != 0xFE {
                offset += 1;
                continue;
            }
            if offset + 2 > n {
                break;
            }
            let payload_len = recv_buf[offset + 1] as usize;
            let frame_size = 8 + payload_len;
            if offset + frame_size > n {
                break;
            }
            if let Some((hdr, _)) = parse_mavlink_frame(&recv_buf[offset..offset + frame_size]) {
                sys_ids.push(hdr.system_id);
            }
            offset += frame_size;
        }

        // All 3 system_ids should appear (heartbeat + sys_status each)
        assert!(sys_ids.contains(&1), "missing sys_id 1 in {sys_ids:?}");
        assert!(sys_ids.contains(&2), "missing sys_id 2 in {sys_ids:?}");
        assert!(sys_ids.contains(&3), "missing sys_id 3 in {sys_ids:?}");
    }
}
