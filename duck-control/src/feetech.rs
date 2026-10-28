//! Feetech bus (HD-1910M, SCS/STS protocol) — the replica HAT's servo bus.
//!
//! Kept beside [`crate::bus`], which speaks Dynamixel protocol 2.0 for the original
//! Microduck. The control loop, the kinematics and the policies are unchanged: this module
//! only replaces the wire and the unit conversions, and it deliberately keeps the original
//! joint conventions — radians, `2π·count/4096 − π`, positive as on an XL330 — so a policy
//! trained on the original robot sees the same numbers. Per-joint `sign`/`offset` in
//! [`JointCalibration`] absorb the differences of a replica build (mirrored joints, a
//! different zero) without touching the observation vector.
//!
//! Protocol facts come from `fanhao375/microduck-replica`
//! (`tools/FeeTech_HD1910M_Servo/src/registers.rs`, `software/studio/studio/bus/feetech.py`).

use std::collections::HashMap;
use std::f64::consts::PI;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serialport::SerialPort;

use crate::imu::{ImuData, SflpDecoder, IMU_BLOCK_LEN};
use crate::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{IMU_DXL_ID, JOINT_IDS, NUM_JOINTS};

/// Feetech SCS/STS runs at 1 Mbps by default (EEPROM `baud` = 0).
pub const BAUD_RATE: u32 = 1_000_000;

const HEADER: [u8; 2] = [0xFF, 0xFF];
const BROADCAST: u8 = 0xFE;

const PING: u8 = 0x01;
const READ: u8 = 0x02;
const WRITE: u8 = 0x03;
const RESET: u8 = 0x06;
const SYNC_READ: u8 = 0x82;
const SYNC_WRITE: u8 = 0x83;

/// HD-1910M register map (HLS_2 memory table).
const REG_ID: u8 = 5;
const REG_BAUD: u8 = 6;
const REG_TORQUE_SWITCH: u8 = 40;
const REG_GOAL_POS: u8 = 42;
const REG_TORQUE_LIMIT: u8 = 48;
const REG_KP: u8 = 50;
const REG_KD: u8 = 51;
const REG_KI: u8 = 52;
/// `present position` (u16) · `present speed` (i16) · `present load` (i16) · voltage · temperature.
/// The IMU board, when it is on this bus, serves its 12-byte block at the same address.
const REG_FEEDBACK: u8 = 56;
const FEEDBACK_LEN: u8 = 12;
/// `present voltage` (0.1 V) — reading two bytes here also covers `present temperature` (63).
const REG_VOLTAGE: u8 = 62;

/// Counts per revolution, same as an XL330, which is what keeps the position convention.
const COUNTS_PER_TURN: f64 = 4096.0;

/// `present speed` counts 0.732 rev/min each (HD-1910M), in rad/s.
const RAD_PER_SEC_PER_COUNT: f64 = 0.732 * (2.0 * PI / 60.0);

/// One feedback byte pair is 0.1 V (`present voltage`).
const VOLTS_PER_COUNT: f64 = 0.1;

/// A 16-device read at 1 Mbps finishes in a few milliseconds; 25 ms is the same bound the
/// Dynamixel backend uses and keeps a missing servo from stalling the 50 Hz loop.
const READ_TIMEOUT: Duration = Duration::from_millis(25);

/// Per-joint mapping between the bus counts and the original robot's joint convention.
///
/// `sign = 1`, `offset = 0` is exactly the original: `rad = 2π·count/4096 − π`. A replica
/// whose joint is mounted mirrored sets `sign = -1`; a joint whose mechanical zero differs
/// sets `offset` (radians). Both are applied in software, so the observation vector stays
/// in the same frame as a policy trained on the original.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointCalibration {
    pub sign: f64,
    pub offset: f64,
}

impl Default for JointCalibration {
    fn default() -> Self {
        Self {
            sign: 1.0,
            offset: 0.0,
        }
    }
}

impl JointCalibration {
    /// The original convention, for every joint.
    pub const IDENTITY: [JointCalibration; NUM_JOINTS] = [JointCalibration {
        sign: 1.0,
        offset: 0.0,
    }; NUM_JOINTS];

    fn to_rad(self, counts: u16) -> f64 {
        self.sign * (2.0 * PI * counts as f64 / COUNTS_PER_TURN - PI) + self.offset
    }

    fn to_counts(self, rad: f64) -> u16 {
        let raw = (rad - self.offset) / self.sign;
        let counts = (raw + PI) * COUNTS_PER_TURN / (2.0 * PI);
        counts.round().clamp(0.0, COUNTS_PER_TURN - 1.0) as u16
    }
}

/// Parse a comma-separated list of numbers, skipping blanks and malformed entries.
fn parse_list(raw: &str) -> Vec<f64> {
    raw.split(',')
        .filter_map(|item| item.trim().parse().ok())
        .collect()
}

/// Per-joint calibration from the environment, so a replica build can mirror a joint or shift
/// a zero without recompiling. Both lists are in [`JOINT_IDS`] order.
///
/// `MICRODUCK_FEETECH_SIGNS` is 15 values, each coerced to ±1; `MICRODUCK_FEETECH_OFFSETS` is
/// 15 values in radians. Leaving either unset is the original convention: `sign = 1`,
/// `offset = 0`, which is what makes a policy trained on the original robot see the same
/// joint vector.
fn calibration_from_env() -> [JointCalibration; NUM_JOINTS] {
    let mut calibration = JointCalibration::IDENTITY;
    if let Ok(raw) = std::env::var("MICRODUCK_FEETECH_SIGNS") {
        let signs = parse_list(&raw);
        if signs.len() == NUM_JOINTS {
            for (joint, sign) in calibration.iter_mut().zip(signs) {
                joint.sign = if sign < 0.0 { -1.0 } else { 1.0 };
            }
        } else {
            tracing::warn!(
                got = signs.len(),
                "MICRODUCK_FEETECH_SIGNS needs 15 values; using +1"
            );
        }
    }
    if let Ok(raw) = std::env::var("MICRODUCK_FEETECH_OFFSETS") {
        let offsets = parse_list(&raw);
        if offsets.len() == NUM_JOINTS {
            for (joint, offset) in calibration.iter_mut().zip(offsets) {
                joint.offset = offset;
            }
        } else {
            tracing::warn!(
                got = offsets.len(),
                "MICRODUCK_FEETECH_OFFSETS needs 15 values; using 0"
            );
        }
    }
    calibration
}

/// Encode one instruction frame: `FF FF id len instr params chk`.
///
/// `len` counts the instruction and the parameters *plus* the two bytes that follow itself,
/// which is the SCS convention (`len = params + 2`). The checksum is the bitwise inverse of
/// the sum of every byte after the header.
fn frame(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(6 + params.len());
    out.extend_from_slice(&HEADER);
    out.push(id);
    out.push((params.len() + 2) as u8);
    out.push(instruction);
    out.extend_from_slice(params);
    let sum = out[2..].iter().fold(0u32, |s, b| s + u32::from(*b)) as u8;
    out.push(!sum);
    out
}

/// Parse one status packet out of `buf`.
///
/// Returns `(consumed, (id, error, data))`. `None` means there is not yet a complete packet,
/// which is ordinary while the rest of the burst is still arriving.
fn parse_status(buf: &[u8]) -> Option<(usize, (u8, u8, Vec<u8>))> {
    let start = buf.windows(2).position(|w| w == HEADER)?;
    if buf.len() < start + 5 {
        return None;
    }
    let id = buf[start + 2];
    let len = usize::from(buf[start + 3]);
    let total = start + 4 + len;
    if buf.len() < total {
        return None;
    }
    let error = buf[start + 4];
    let data_len = len.saturating_sub(2);
    let data = buf[start + 5..start + 5 + data_len].to_vec();
    let sum = buf[start + 2..start + 4 + len - 1]
        .iter()
        .fold(0u32, |s, b| s + u32::from(*b)) as u8;
    let checksum_ok = !sum == buf[total - 1];
    if !checksum_ok {
        // Drop the header and let the caller resynchronise on the next FF FF.
        return Some((start + 2, (0, 0xFF, Vec::new())));
    }
    Some((total, (id, error, data)))
}

/// The replica HAT's servo bus.
pub struct FeetechIo {
    port: String,
    serial: Box<dyn SerialPort>,
    /// The IDs read every tick, in the order the loop expects. The IMU board, when present on
    /// the bus, is first — exactly as on the original.
    ids: Vec<u8>,
    /// Whether the IMU board answers at [`IMU_DXL_ID`] on this bus. The replica HAT puts the
    /// IMU on I2C, so this is off there and the sample is a neutral orientation until the I2C
    /// reader lands.
    imu_on_bus: bool,
    imu: SflpDecoder,
    last_imu_block: Option<[u8; IMU_BLOCK_LEN]>,
    stale: ImuStale,
    calibration: [JointCalibration; NUM_JOINTS],
}

impl FeetechIo {
    /// Open the bus. `fast_sync_read` is accepted for signature parity with the Dynamixel
    /// backend and ignored: SCS has one sync-read instruction and no fast variant.
    pub fn open(port: &str, _fast_sync_read: bool) -> Result<Self> {
        let serial = serialport::new(port, BAUD_RATE)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: std::io::Error::other(e),
            })?;

        // The replica HAT keeps the IMU on I2C; a build that puts the board on the servo bus
        // (the original wiring) sets this.
        let imu_on_bus = std::env::var("MICRODUCK_IMU_ON_BUS").ok().as_deref() == Some("1");
        let mut ids = Vec::with_capacity(NUM_JOINTS + 1);
        if imu_on_bus {
            ids.push(IMU_DXL_ID);
        }
        ids.extend_from_slice(&JOINT_IDS);

        Ok(Self {
            port: port.to_owned(),
            serial,
            ids,
            imu_on_bus,
            imu: SflpDecoder::default(),
            last_imu_block: None,
            stale: ImuStale::default(),
            calibration: calibration_from_env(),
        })
    }

    /// Override the per-joint mapping (sign / zero). Call before the first read.
    pub fn set_calibration(&mut self, calibration: [JointCalibration; NUM_JOINTS]) {
        self.calibration = calibration;
    }

    /// The body IMU's mounting rotation, as on the Dynamixel backend.
    pub fn set_imu_mount(&mut self, mount: [f64; 4]) {
        self.imu = SflpDecoder::new(mount);
    }

    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        self.serial
            .write_all(bytes)
            .and_then(|()| self.serial.flush())
            .map_err(|e| IoError::Bus(format!("write {} bytes: {e}", bytes.len())))
    }

    /// Read up to `expected` status packets, waiting at most [`READ_TIMEOUT`].
    fn read_statuses(&mut self, expected: usize) -> Result<Vec<(u8, u8, Vec<u8>)>> {
        let mut buf = Vec::new();
        let mut out = Vec::new();
        let deadline = Instant::now() + READ_TIMEOUT;
        let mut chunk = [0u8; 256];

        while out.len() < expected {
            if Instant::now() >= deadline {
                break;
            }
            match self.serial.read(&mut chunk) {
                Ok(0) => break,
                Ok(n) => buf.extend_from_slice(&chunk[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
                Err(e) => return Err(IoError::Bus(format!("read: {e}"))),
            }
            while let Some((consumed, packet)) = parse_status(&buf) {
                buf.drain(..consumed.min(buf.len()));
                let (id, error, data) = packet;
                if error == 0xFF && data.is_empty() {
                    continue; // checksum mismatch, already resynchronised
                }
                if error != 0 {
                    tracing::warn!(id, error, "feetech status packet reports an error");
                }
                out.push((id, error, data));
            }
        }
        Ok(out)
    }

    /// One `SYNC_READ`: each device answers its own status packet.
    fn sync_read(&mut self, ids: &[u8], addr: u8, len: u8) -> Result<Vec<Vec<u8>>> {
        let mut params = vec![addr, len];
        params.extend_from_slice(ids);
        let bytes = frame(BROADCAST, SYNC_READ, &params);
        self.send(&bytes)?;

        let statuses = self.read_statuses(ids.len())?;
        let mut by_id: HashMap<u8, Vec<u8>> = HashMap::new();
        for (id, error, data) in statuses {
            if error == 0 {
                by_id.insert(id, data);
            }
        }
        let mut out = Vec::with_capacity(ids.len());
        for &id in ids {
            let data = by_id.remove(&id).ok_or(IoError::ShortRead {
                what: "feetech sync_read packet",
                expected: ids.len(),
                got: out.len(),
            })?;
            out.push(data);
        }
        Ok(out)
    }

    /// `SYNC_WRITE` of one register, the same `data` to every id.
    fn sync_write(&mut self, ids: &[u8], addr: u8, data: &[u8]) -> Result<()> {
        // The second parameter is the payload length *per servo*, not the total.
        let mut params = vec![addr, data.len() as u8];
        for &id in ids {
            params.push(id);
            params.extend_from_slice(data);
        }
        let bytes = frame(BROADCAST, SYNC_WRITE, &params);
        self.send(&bytes)
    }

    /// Read one register from one device.
    fn read_register(&mut self, id: u8, addr: u8, len: u8) -> Result<Vec<u8>> {
        let bytes = frame(id, READ, &[addr, len]);
        self.send(&bytes)?;
        let mut statuses = self.read_statuses(1)?;
        statuses
            .pop()
            .map(|(_, _, data)| data)
            .ok_or(IoError::Bus(format!("no answer reading {addr} from {id}")))
    }

    /// Write one register on one device, and wait for its acknowledgement.
    fn write_register(&mut self, id: u8, addr: u8, data: &[u8]) -> Result<()> {
        let mut params = vec![addr];
        params.extend_from_slice(data);
        let bytes = frame(id, WRITE, &params);
        self.send(&bytes)?;
        let _ = self.read_statuses(1)?;
        Ok(())
    }

    fn ping(&mut self, id: u8) -> Result<bool> {
        let bytes = frame(id, PING, &[]);
        self.send(&bytes)?;
        Ok(!self.read_statuses(1)?.is_empty())
    }

    /// Assert the registers the loop depends on. Returns how many were corrected.
    ///
    /// The replica servos are configured from the calibration file rather than flashed here;
    /// this checks that every joint answers and that the ID/baud pair is the one the bus
    /// expects, which is what a motor swap breaks.
    pub fn check_registers(&mut self) -> Result<usize> {
        let mut fixed = 0;
        for &id in &JOINT_IDS {
            if !self.ping(id)? {
                tracing::warn!(id, "feetech servo does not answer at startup");
                continue;
            }
            if let Ok(data) = self.read_register(id, REG_ID, 1) {
                if data.first().copied() != Some(id) {
                    tracing::warn!(id, got = ?data.first(), "feetech servo answers with a different ID");
                }
            }
            if let Ok(data) = self.read_register(id, REG_BAUD, 1) {
                if data.first().copied().unwrap_or(0) != 0 {
                    tracing::warn!(id, got = ?data.first(), "feetech baud register is not 1 Mbps");
                    self.write_register(id, REG_BAUD, &[0])?;
                    fixed += 1;
                }
            }
        }
        Ok(fixed)
    }

    /// Cap the torque of every joint, in 0.1 % of the servo's maximum (1000 = no cap).
    ///
    /// The replica runs a lower ceiling than the factory value; the control loop's own
    /// limits are separate and unchanged.
    pub fn set_torque_limit(&mut self, permille: u16) -> Result<()> {
        let value = permille.min(1000).to_le_bytes();
        for &id in &JOINT_IDS {
            self.write_register(id, REG_TORQUE_LIMIT, &value)?;
        }
        Ok(())
    }

    pub fn missing_servos(&mut self) -> Result<Vec<u8>> {
        let mut missing = Vec::new();
        for &id in &JOINT_IDS {
            if !self.ping(id)? {
                missing.push(id);
            }
        }
        Ok(missing)
    }

    /// Not implemented for Feetech yet: a fresh servo is identified by its factory ID and
    /// flashed ID-first, which needs the calibration file's joint order. Returning `false`
    /// leaves the caller's wait loop intact instead of pretending a swap happened.
    pub fn adopt_replacement(&mut self, id: u8) -> Result<bool> {
        tracing::warn!(
            id,
            "feetech bus: automatic replacement is not implemented yet"
        );
        Ok(false)
    }

    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let blocks = self.sync_read(&JOINT_IDS, REG_FEEDBACK, FEEDBACK_LEN)?;
        let mut out = [0.0; NUM_JOINTS];
        for (j, block) in blocks.iter().enumerate() {
            out[j] = self.calibration[j].to_rad(pos_of(block)?);
        }
        Ok(out)
    }

    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        self.sync_write(&JOINT_IDS, REG_TORQUE_SWITCH, &[u8::from(on)])
    }

    pub fn set_gain(&mut self, kp: u16) -> Result<()> {
        let kp = kp.min(254) as u8;
        for &id in &JOINT_IDS {
            // The replica's default is kP on, kI and kD at zero — the same shape as the
            // prototype's `--ki/--kd 0` defaults, in HD-1910 register units.
            self.write_register(id, REG_KP, &[kp])?;
            self.write_register(id, REG_KI, &[0])?;
            self.write_register(id, REG_KD, &[0])?;
        }
        Ok(())
    }

    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            self.write(&JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }

    pub fn port(&self) -> &str {
        &self.port
    }
}

fn pos_of(block: &[u8]) -> Result<u16> {
    if block.len() < FEEDBACK_LEN as usize {
        return Err(IoError::ShortRead {
            what: "feetech feedback block",
            expected: FEEDBACK_LEN as usize,
            got: block.len(),
        });
    }
    Ok(u16::from_le_bytes([block[0], block[1]]))
}

/// The `present speed` word, sign bit 15.
fn speed_of(block: &[u8]) -> Result<i16> {
    if block.len() < 4 {
        return Err(IoError::ShortRead {
            what: "feetech speed word",
            expected: 4,
            got: block.len(),
        });
    }
    let raw = u16::from_le_bytes([block[2], block[3]]);
    let magnitude = (raw & 0x7FFF) as i16;
    Ok(if raw & 0x8000 != 0 {
        -magnitude
    } else {
        magnitude
    })
}

/// The `present load` word, sign bit 15. The unit is 0.1 % of the torque limit, not mA;
/// [`Sensors::currents_ma`] is the only load channel the loop has, so this is scaled to a
/// comparable magnitude for the thresholds that read it. It is a torque proxy, not a current.
fn load_of(block: &[u8]) -> Result<f64> {
    if block.len() < 6 {
        return Err(IoError::ShortRead {
            what: "feetech load word",
            expected: 6,
            got: block.len(),
        });
    }
    let raw = u16::from_le_bytes([block[4], block[5]]);
    Ok(f64::from(raw & 0x7FFF) * 10.0)
}

impl RobotIo for FeetechIo {
    fn read(&mut self) -> Result<Sensors> {
        let ids = self.ids.clone();
        let blocks = self.sync_read(&ids, REG_FEEDBACK, FEEDBACK_LEN)?;
        let mut sensors = Sensors::default();
        let first_joint = if self.imu_on_bus {
            let mut raw = [0u8; IMU_BLOCK_LEN];
            raw.copy_from_slice(&blocks[0][..IMU_BLOCK_LEN]);
            if self.last_imu_block == Some(raw) {
                self.stale.total = self.stale.total.saturating_add(1);
                self.stale.run = self.stale.run.saturating_add(1);
            } else {
                self.stale.run = 0;
            }
            self.last_imu_block = Some(raw);
            sensors.imu = self.imu.decode(&raw);
            1
        } else {
            // No bus IMU: a neutral sample, so `[policy] enabled=false` can run the loop and
            // hold a pose before the I2C reader exists.
            sensors.imu = ImuData::default();
            0
        };

        if blocks.len() != self.ids.len() {
            return Err(IoError::ShortRead {
                what: "feetech sync_read blocks",
                expected: self.ids.len(),
                got: blocks.len(),
            });
        }

        for j in 0..NUM_JOINTS {
            let block = &blocks[first_joint + j];
            let calibration = self.calibration[j];
            sensors.positions[j] = calibration.to_rad(pos_of(block)?);
            sensors.velocities[j] =
                f64::from(speed_of(block)?) * RAD_PER_SEC_PER_COUNT * calibration.sign;
            sensors.currents_ma[j] = load_of(block)?;
        }
        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        let mut data = Vec::with_capacity(JOINT_IDS.len() * 2);
        for (j, &id) in JOINT_IDS.iter().enumerate() {
            let counts = self.calibration[j].to_counts(targets.positions[j]);
            data.extend_from_slice(&counts.to_le_bytes());
            let _ = id;
        }
        // One sync_write carrying 15 × (id, u16). Built here rather than in `sync_write`
        // because the payload differs per servo, unlike torque on/off.
        let mut params = vec![REG_GOAL_POS, 2u8];
        for (j, &id) in JOINT_IDS.iter().enumerate() {
            params.push(id);
            let counts = u16::from_le_bytes([data[j * 2], data[j * 2 + 1]]);
            params.extend_from_slice(&counts.to_le_bytes());
        }
        self.send(&frame(BROADCAST, SYNC_WRITE, &params))
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        FeetechIo::set_torque(self, on)
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        self.send(&frame(id, RESET, &[]))?;
        let _ = self.read_statuses(1)?;
        Ok(())
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        FeetechIo::set_gain(self, kp)
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let blocks = self.sync_read(&JOINT_IDS, REG_VOLTAGE, 2)?;
        let mut temps_c = [0.0; NUM_JOINTS];
        let mut volts = Vec::with_capacity(NUM_JOINTS);
        for (j, block) in blocks.iter().enumerate() {
            if block.len() < 2 {
                return Err(IoError::ShortRead {
                    what: "feetech voltage+temperature",
                    expected: 2,
                    got: block.len(),
                });
            }
            let v = f64::from(block[0]) * VOLTS_PER_COUNT;
            if v > 0.0 {
                volts.push(v);
            }
            temps_c[j] = f64::from(block[1]);
        }
        if volts.is_empty() {
            return Err(IoError::ShortRead {
                what: "feetech input voltage",
                expected: NUM_JOINTS,
                got: 0,
            });
        }
        Ok(SlowSensors {
            volts: volts.iter().sum::<f64>() / volts.len() as f64,
            temps_c,
        })
    }

    fn imu_stale(&self) -> ImuStale {
        self.stale
    }

    fn imu_ready(&self) -> bool {
        if self.imu_on_bus {
            self.imu.ready()
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ping ID 1: `FF FF 01 02 01 FB`.
    #[test]
    fn ping_frame_matches_the_replica_wire_format() {
        assert_eq!(
            frame(1, PING, &[]),
            vec![0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]
        );
    }

    /// A `SYNC_READ` for two servos at the feedback block, the frame the replica's own tool
    /// prints for the same request (`tools/imu200/imu200.py`).
    #[test]
    fn sync_read_frame_is_the_expected_sequence() {
        let bytes = frame(BROADCAST, SYNC_READ, &[REG_FEEDBACK, FEEDBACK_LEN, 1, 2]);
        assert_eq!(
            bytes,
            vec![0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38, 0x0C, 0x01, 0x02, 0x32]
        );
    }

    /// A status packet round-trips through the parser with its data intact.
    #[test]
    fn status_packet_parses_data_and_checksum() {
        let data = [
            0x00u8, 0x08, 0x10, 0x00, 0x00, 0x00, 0x50, 0x21, 0x00, 0x00, 0x00, 0x00,
        ];
        let mut body = vec![0x01, (data.len() + 2) as u8, 0x00];
        body.extend_from_slice(&data);
        let sum = body.iter().fold(0u32, |s, b| s + u32::from(*b)) as u8;
        let mut packet = vec![0xFF, 0xFF];
        packet.extend_from_slice(&body);
        packet.push(!sum);
        let (consumed, (id, error, parsed)) = parse_status(&packet).expect("complete packet");
        assert_eq!(consumed, packet.len());
        assert_eq!((id, error), (1, 0));
        assert_eq!(parsed, data.to_vec());
    }

    /// Position is the original convention at the identity calibration: 2048 → 0 rad,
    /// 4095 → just under +π, 0 → −π.
    #[test]
    fn position_matches_the_original_convention() {
        let c = JointCalibration::default();
        assert!((c.to_rad(2048) - 0.0).abs() < 1e-12);
        assert!((c.to_rad(4095) - (2.0 * PI * 4095.0 / 4096.0 - PI)).abs() < 1e-12);
        assert!((c.to_rad(0) + PI).abs() < 1e-12);
    }

    /// And the write path is its inverse, so a commanded angle comes back as the same angle.
    #[test]
    fn position_round_trips_through_counts() {
        for counts in [0u16, 1, 1024, 2048, 3072, 4095] {
            let radians = JointCalibration::default().to_rad(counts);
            assert_eq!(JointCalibration::default().to_counts(radians), counts);
        }
    }

    /// A mirrored joint keeps its magnitudes and flips its sign, which is what a `sign = -1`
    /// calibration is for; the zero stays where the original put it.
    #[test]
    fn mirrored_calibration_flips_sign_but_not_zero() {
        let c = JointCalibration {
            sign: -1.0,
            offset: 0.0,
        };
        assert!((c.to_rad(2048) - 0.0).abs() < 1e-12);
        assert!((c.to_rad(3072) + PI / 2.0).abs() < 1e-12);
        assert_eq!(c.to_counts(-PI / 2.0), 3072);
    }

    /// 0.732 rev/min per count, the HD-1910M datasheet figure.
    #[test]
    fn velocity_scale_matches_the_datasheet() {
        let rpm = RAD_PER_SEC_PER_COUNT * 60.0 / (2.0 * PI);
        assert!((rpm - 0.732).abs() < 1e-12);
    }

    #[test]
    fn speed_word_uses_bit_15_as_direction() {
        let mut block = [0u8; FEEDBACK_LEN as usize];
        block[2..4].copy_from_slice(&0x0064u16.to_le_bytes());
        assert_eq!(speed_of(&block).unwrap(), 100);
        block[2..4].copy_from_slice(&0x8064u16.to_le_bytes());
        assert_eq!(speed_of(&block).unwrap(), -100);
    }

    /// The calibration lists are hand-written in `robotd.toml`-adjacent docs; blanks and a
    /// trailing comma are ordinary and must not shift the joints.
    #[test]
    fn calibration_list_tolerates_blanks() {
        assert_eq!(parse_list("1, -1, ,0.5,"), vec![1.0, -1.0, 0.5]);
        assert_eq!(parse_list(""), Vec::<f64>::new());
    }
}
