//! Print the official robot's pose constants as a table.
//!
//! `DEFAULT_POSITION` is the training environment's `HOME_FRAME`: every policy observes joint
//! positions *relative* to it, so it is the pose a replica has to reproduce at its mechanical
//! zero. `REST_POSITION` is where a seated robot settles with torque off, and is printed beside
//! it because a calibration pass usually wants both.
//!
//! ```text
//! cargo run -p duck-control --example official-zero-pose
//! cargo run -p duck-control --example official-zero-pose -- --csv
//! cargo run -p duck-control --example official-zero-pose -- --json
//! ```

use std::f64::consts::PI;

use duck_control::model::{
    DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, MOUTH_CLOSED, MOUTH_OPEN, NUM_JOINTS, REST_POSITION,
    homing_offset,
};

/// Servo counts for an angle, with the original convention (2048 counts = 0 rad).
fn counts(rad: f64) -> f64 {
    2048.0 + rad * 4096.0 / (2.0 * PI)
}

fn degrees(rad: f64) -> f64 {
    rad * 180.0 / PI
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let csv = args.iter().any(|a| a == "--csv");
    let json = args.iter().any(|a| a == "--json");

    if json {
        println!("{{");
        println!("  \"source\": \"duck_control::model::DEFAULT_POSITION (HOME_FRAME)\",");
        println!("  \"joints\": [");
        for j in 0..NUM_JOINTS {
            let comma = if j + 1 == NUM_JOINTS { "" } else { "," };
            println!(
                "    {{\"name\": \"{}\", \"id\": {}, \"home_rad\": {:.6}, \"home_deg\": {:.3}, \"home_counts\": {:.1}, \"rest_rad\": {:.6}, \"homing_offset_counts\": {}}}{}",
                JOINT_NAMES[j],
                JOINT_IDS[j],
                DEFAULT_POSITION[j],
                degrees(DEFAULT_POSITION[j]),
                counts(DEFAULT_POSITION[j]),
                REST_POSITION[j],
                homing_offset(JOINT_IDS[j]),
                comma
            );
        }
        println!("  ],");
        println!("  \"mouth_closed_rad\": {MOUTH_CLOSED:.6}, \"mouth_open_rad\": {MOUTH_OPEN:.6}");
        println!("}}");
        return;
    }

    if csv {
        println!("joint,id,home_rad,home_deg,home_counts,rest_rad,homing_offset_counts");
        for j in 0..NUM_JOINTS {
            println!(
                "{},{},{:.6},{:.3},{:.1},{:.6},{}",
                JOINT_NAMES[j],
                JOINT_IDS[j],
                DEFAULT_POSITION[j],
                degrees(DEFAULT_POSITION[j]),
                counts(DEFAULT_POSITION[j]),
                REST_POSITION[j],
                homing_offset(JOINT_IDS[j])
            );
        }
        return;
    }

    println!(
        "| joint | ID | home (rad) | home (°) | home counts (2048=0) | rest (rad) | EEPROM homing |"
    );
    println!("|---|---:|---:|---:|---:|---:|---:|");
    for j in 0..NUM_JOINTS {
        println!(
            "| {} | {} | {:.4} | {:.2} | {:.0} | {:.4} | {} |",
            JOINT_NAMES[j],
            JOINT_IDS[j],
            DEFAULT_POSITION[j],
            degrees(DEFAULT_POSITION[j]),
            counts(DEFAULT_POSITION[j]),
            REST_POSITION[j],
            homing_offset(JOINT_IDS[j])
        );
    }
    println!();
    println!(
        "mouth: closed {:.2}°, open {:.2}° (not part of any policy's 14 actions)",
        degrees(MOUTH_CLOSED),
        degrees(MOUTH_OPEN)
    );
}
