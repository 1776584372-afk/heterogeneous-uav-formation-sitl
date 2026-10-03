import csv
import io
import json

from pymavlink.dialects.v20 import ardupilotmega as mavlink
import pytest

from scripts.audit_calibration_alignment import audit


def packet(index, wall, boot, east, north):
    stream = io.BytesIO()
    mav = mavlink.MAVLink(stream)
    mav.seq = (index - 1) % 256
    mav.set_position_target_local_ned_send(
        0, 1, 1, 1, 0, north, east, -60, 10, 0, 0, 0, 0, 0, 0, 0)
    payload = stream.getvalue()
    return {
        "schema_version": 1,
        "event": "socket_send",
        "send_index": index,
        "wall_before_s": wall - 0.001,
        "wall_after_s": wall,
        "requested_bytes": len(payload),
        "accepted_bytes": len(payload),
        "status": "SOCKET_ACCEPTED_ALL",
        "error": None,
        "packet_hex": payload.hex(),
        "context": {
            "phase": "turn_coupled_final_14_5_-1",
            "clocks": {
                "ATTITUDE": {"boot_s": boot},
                "LOCAL_POSITION_NED": {"boot_s": boot},
            },
            "target": {"ref_east": east, "ref_north": north},
        },
    }


def write_run(root, records, rows):
    root.mkdir()
    (root / "outgoing_packets.jsonl").write_text(
        "".join(json.dumps(record) + "\n" for record in records))
    fields = ["wall_s", "sim_s", "velocity_sim_s", "phase", "east", "north",
              "ref_east", "ref_north"]
    with (root / "telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_stale_convenience_copy_does_not_become_tracking_failure(tmp_path):
    records = [packet(1, 1.0, 1.0, 10.0, 0.0),
               packet(2, 1.4, 1.4, 14.0, 0.0)]
    rows = [
        {"wall_s": 1.01, "sim_s": 1.0, "velocity_sim_s": 1.0,
         "phase": "turn_coupled_final_14_5_-1", "east": 10.2, "north": 0,
         "ref_east": 10.0, "ref_north": 0},
        # New state arrived before the next command was generated; the copied
        # target is stale and creates a false 4.2 m peak.
        {"wall_s": 1.39, "sim_s": 1.4, "velocity_sim_s": 1.4,
         "phase": "turn_coupled_final_14_5_-1", "east": 14.2, "north": 0,
         "ref_east": 10.0, "ref_north": 0},
    ]
    root = tmp_path / "run"
    write_run(root, records, rows)
    result = audit(root, maximum_gap_s=0.1, position_gate_m=4.0)
    assert result["legacy_copied_reference"]["maximum"] == pytest.approx(4.2)
    assert result["legacy_copied_reference"]["peak_uses_stale_reference"] is True
    assert result["time_aligned_reference"]["all"]["maximum"] < 0.21
    assert result["decision"]["classification"] == "INVALID_EVIDENCE"


def test_synchronous_failure_remains_a_tracking_failure(tmp_path):
    records = [packet(1, 1.0, 1.0, 10.0, 0.0),
               packet(2, 1.05, 1.05, 10.5, 0.0),
               packet(3, 1.10, 1.10, 11.0, 0.0)]
    rows = [
        {"wall_s": 1.01, "sim_s": 1.0, "velocity_sim_s": 1.0,
         "phase": "turn_coupled_final_14_5_-1", "east": 15.0, "north": 0,
         "ref_east": 10.0, "ref_north": 0},
        {"wall_s": 1.06, "sim_s": 1.05, "velocity_sim_s": 1.05,
         "phase": "turn_coupled_final_14_5_-1", "east": 15.5, "north": 0,
         "ref_east": 10.5, "ref_north": 0},
        {"wall_s": 1.11, "sim_s": 1.10, "velocity_sim_s": 1.10,
         "phase": "turn_coupled_final_14_5_-1", "east": 16.0, "north": 0,
         "ref_east": 11.0, "ref_north": 0},
    ]
    root = tmp_path / "run"
    write_run(root, records, rows)
    result = audit(root, maximum_gap_s=0.1, position_gate_m=4.0)
    assert result["legacy_copied_reference"]["peak_uses_stale_reference"] is False
    assert result["time_aligned_reference"]["all"]["maximum"] == 5.0
    assert result["decision"]["classification"] == "TRACKING_FAILED"
