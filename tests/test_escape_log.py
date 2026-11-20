import csv
from dataclasses import replace

import pytest

from flydrones.brain import Brain, build_minifly
from flydrones.cli import main
from flydrones.config import load_config
from flydrones.drones import SimDrone
from flydrones.motor import MotorDecoder
from flydrones.runtime import Pilot
from flydrones.safety import Telemetry


@pytest.mark.parametrize(("flow", "pick", "source"), [
    ((2, 1, 3), "right", "optic_flow"),
    ((1, 2, 3), "left", "optic_flow"),
    ((1.01, 1, 2), "turn180", "optic_flow"),
    ((1, 1, 2), "turn180", "random_tie"),
])
def test_visual_decision_records_the_actual_trigger_values(flow, pick, source):
    cfg = load_config()
    cfg["decoder"]["settle_s"] = 0
    dec = MotorDecoder(cfg)
    # At twice reference speed, 40 Hz is needed even though the base threshold is 20 Hz.
    cmd = dec.update({"DNp01_L": 120}, 0.05, tel=Telemetry(speed=1.6), flow=flow)
    event = dec.escape_event
    assert event.cause == "loom"
    assert event.cue == event.maneuver == pick
    assert event.direction_source == source
    assert (event.gf_hz, event.threshold_hz, event.retrigger) == (48, 40, False)
    if pick == "turn180":
        assert event.turn180_sign == (1 if cmd.yaw > 0 else -1)
    else:
        assert event.turn180_sign is None
    assert dec.update({}, 0.05).escape
    assert dec.escape_event is None


def test_retrigger_during_climb_is_a_new_decision():
    cfg = load_config()
    cfg["decoder"]["settle_s"] = 0
    cfg["decoder"]["escape"]["pick_p"] = {"climb": 1.0, "left": 0.0}
    dec = MotorDecoder(cfg)
    dec.update({"DNp01_L": 120}, 0.05)
    assert dec.escape_event.maneuver == "climb"
    for _ in range(8):
        dec.update({}, 0.05)
        assert dec.escape_event is None
    dec.update({"DNp01_L": 120}, 0.05, flow=(2, 1, 3))
    assert dec.escape_event.retrigger
    assert dec.escape_event.maneuver == "right"


def test_contact_event_survives_simulator_push_off(monkeypatch):
    cfg = load_config()
    cfg["decoder"]["settle_s"] = 0
    drone = SimDrone()
    pilot = Pilot(Brain(build_minifly(), cfg), drone, cfg)
    pilot.safety.kill = True
    tel = replace(drone.telemetry(), touching=True)
    monkeypatch.setattr(drone, "telemetry", lambda: tel)
    pilot.tick(0, 0.05)
    row = pilot.history[-1]
    assert pilot.decoder.esc_cause is None  # runtime consumed the contact for push-off
    assert row["escape_cause"] == "contact"
    assert row["escape_cue"] == "none"
    assert row["escape_direction_source"] == "random_fallback"
    assert row["escape_gf_hz"] < row["escape_threshold_hz"]
    assert not row["escape"]  # The governor suppressed the selected maneuver.


def test_contact_takes_priority_when_both_triggers_are_ready():
    cfg = load_config()
    cfg["decoder"]["settle_s"] = 0
    dec = MotorDecoder(cfg)
    dec.update({"DNp01_L": 120}, 0.05, tel=Telemetry(touching=True), flow=(1, 2, 3))
    event = dec.escape_event
    assert event.gf_hz >= event.threshold_hz
    assert (event.cause, event.maneuver, event.direction_source) == ("contact", "left", "optic_flow")


def test_fly_log_without_triggers_has_empty_event_columns(tmp_path):
    path = tmp_path / "quiet.csv"
    assert main(["fly", "--drone", "sim", "--input", "camera", "--seconds", "1", "--log", str(path)]) == 0
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows and "escape_cause" in rows[0]
    assert all(value == "" for row in rows for key, value in row.items() if key.startswith("escape_"))


def test_demo_csv_separates_trigger_and_hold(tmp_path):
    path = tmp_path / "flight.csv"
    assert main(["demo", "--seconds", "14", "--log", str(path)]) == 0
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    triggers = [r for r in rows if r["escape_cause"]]
    assert triggers
    first = triggers[0]
    assert float(first["t"]) == 13.5
    assert first["escape_cause"] == "loom"
    assert first["escape_cue"] == "none"
    assert first["escape_maneuver"] in {"left", "right", "climb"}
    assert first["escape_direction_source"] == "random_fallback"
    assert float(first["escape_gf_hz"]) >= float(first["escape_threshold_hz"])
    assert first["escape_retrigger"] == "False"
    held = rows[rows.index(first) + 1]
    assert held["escape"] == "True"
    assert all(value == "" for key, value in held.items() if key.startswith("escape_"))
