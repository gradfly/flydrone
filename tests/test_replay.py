import csv
import gzip
import json
import subprocess
import sys

import pytest

from flydrones.brain import build_minifly
from flydrones.config import load_config
from flydrones.replay import record_senses, replay_rates


def test_demo_trace_replays_in_a_new_process(tmp_path):
    trace = tmp_path / "demo.jsonl.gz"
    flight = tmp_path / "flight.csv"
    replay = tmp_path / "replay.csv"
    config = tmp_path / "demo.yaml"
    config.write_text("brain:\n  seed: 17\ncontrol:\n  hz: 16\n", encoding="utf-8")
    cli = [sys.executable, "-m", "flydrones"]
    subprocess.run([*cli, "demo", "--config", config, "--seconds", "5", "--sensory-trace", trace,
                    "--log", flight], check=True)
    subprocess.run([*cli, "replay", trace, "--log", replay], check=True)

    with flight.open(newline="") as src, replay.open(newline="") as dst:
        expected = list(csv.DictReader(src))
        actual = list(csv.DictReader(dst))
    assert len(expected) == len(actual) == 80
    fields = ["t", *(name for name in expected[0] if name.startswith("hz_"))]
    assert len(fields) == 7
    assert actual == [{key: row[key] for key in fields} for row in expected]

    other = build_minifly(seed=8).save(tmp_path / "other.npz")
    alt = list(replay_rates(trace, other))
    assert [r["t"] for r in alt] == [float(r["t"]) for r in actual]
    assert any(a["hz_DNg02_L"] != float(b["hz_DNg02_L"]) for a, b in zip(alt, actual, strict=True))


def test_rejects_unknown_trace_version(tmp_path):
    trace = tmp_path / "future.gz"
    with gzip.open(trace, "wt", encoding="utf-8") as stream:
        print(json.dumps({"format": "flydrones.senses.v99"}), file=stream)
    with pytest.raises(ValueError, match="Unsupported sensory trace format"):
        list(replay_rates(trace))


def test_recording_error_preserves_previous_trace(tmp_path):
    trace = tmp_path / "previous.gz"
    trace.write_bytes(b"previous recording")
    with pytest.raises(KeyboardInterrupt), record_senses(trace, load_config()):
        raise KeyboardInterrupt
    assert trace.read_bytes() == b"previous recording"
