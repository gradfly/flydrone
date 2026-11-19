"""Record demo observations at the sensory encoder and replay them through a brain."""

from __future__ import annotations

import gzip
import json
import math
from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from . import __version__
from .brain import Brain, load_connectome
from .senses import InputEncoder
from .senses.retina import EyeFeatures, VisualFrame

FORMAT = "flydrones.senses.v1"


@contextmanager
def record_senses(path, cfg):
    """Stream a compressed JSON-lines trace. Publish it once recording finishes."""
    path = Path(path)
    settings = {key: cfg[key] for key in ("brain", "inputs", "outputs", "imu")}
    # Keep a portable model label. Replay of an external connectome takes an explicit path.
    source = str(cfg["brain"]["source"])
    builtin = source.lower() in {"minifly", "mini", "synthetic"}
    settings["brain"] = dict(settings["brain"], source=Path(source).name)
    with TemporaryDirectory(dir=path.parent) as tmp:
        staged = Path(tmp) / "trace.gz"
        with gzip.open(staged, "wt", encoding="utf-8", compresslevel=3) as stream:
            header = {"format": FORMAT, "package_version": __version__, "builtin_minifly": builtin, "settings": settings}
            print(json.dumps(header), file=stream)

            def observe(t, dt, vision, yaw_rate):
                eyes = {
                    side: {name: [grid.dtype.name, grid.tolist()] for name, grid in eye.grids.items()}
                    for side, eye in vision.eyes.items()
                }
                tick = {"t": t, "dt": dt, "yaw_rate_dps": yaw_rate, "eyes": eyes}
                print(json.dumps(tick, separators=(",", ":")), file=stream)

            yield observe
        staged.replace(path)


def replay_rates(path, brain_source=None):
    """Yield one row per flight tick. Negative timestamps advance the brain during warm-up."""
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        header = json.loads(stream.readline())
        if header.get("format") != FORMAT:
            raise ValueError("Unsupported sensory trace format")
        cfg = header["settings"]
        if brain_source is None and not header["builtin_minifly"]:
            raise ValueError("Pass --brain with the path to the recorded or an alternative connectome")
        brain = Brain(load_connectome(brain_source or "minifly"), cfg)
        encoder = InputEncoder(brain.connectome, cfg)
        for line in stream:
            tick = json.loads(line)
            if not math.isfinite(tick["dt"]) or tick["dt"] <= 0:
                raise ValueError("Tick duration must be positive and finite")
            eyes = {}
            for side, grids in tick["eyes"].items():
                arrays = {}
                for name, (dtype, values) in grids.items():
                    if dtype not in {"float32", "float64"}:
                        raise ValueError(f"Unsupported sensory grid dtype: {dtype}")
                    arrays[name] = np.asarray(values, dtype=dtype)
                eyes[side] = EyeFeatures(arrays)
            inputs = encoder.encode(VisualFrame(eyes), tick["yaw_rate_dps"])
            rates = brain.tick(inputs, ms=tick["dt"] * 1000)
            if tick["t"] >= 0:
                yield {"t": tick["t"], **{f"hz_{name}": rates[name] for name in brain.output_specs}}
