#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Check Skystars bench overlays without loading parameters or contacting hardware."""

import hashlib
import math
from pathlib import Path


REFERENCE_SHA256 = "7f39b20c60bc1622a8bb68432408b7f540642e186e762ae6f795db047b71f421"
EXPECTED = {
    "bench_A0_A.param": {"LOG_DISARMED": 1, "UTB_ENABLE": 0, "UTB_SHADOW": 0},
    "bench_B.param": {"LOG_DISARMED": 1, "UTB_ENABLE": 1, "UTB_SHADOW": 0},
    "bench_C.param": {"LOG_DISARMED": 1, "UTB_ENABLE": 1, "UTB_SHADOW": 1, "UTB_LOG_RATE": 200},
    "bench_D.param": {"LOG_DISARMED": 1, "UTB_ENABLE": 1, "UTB_SHADOW": 1, "UTB_LOG_RATE": 400},
}


def read_params(path):
    params = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        name, value = line.split(",")
        assert name not in params, (path.name, "duplicate", name)
        number = float(value)
        assert math.isfinite(number), (path.name, name)
        params[name] = number
    return params


def main():
    directory = Path(__file__).resolve().parents[2] / "configs/skystars_5inch"
    reference = directory / "default_config_skystar_funkcni_let.param"
    assert hashlib.sha256(reference.read_bytes()).hexdigest() == REFERENCE_SHA256, "Reference bytes changed"
    params = read_params(reference)
    assert len(params) == 1190 and params["FRAME_CLASS"] == 1 and params["FRAME_TYPE"] == 18
    assert params["LOG_DISARMED"] == 0
    assert {p.name for p in directory.glob("bench_*.param")} == set(EXPECTED), "Unexpected/missing bench overlay"
    for filename, expected in EXPECTED.items():
        assert read_params(directory / filename) == expected, (filename, "allowlist/value mismatch")
        print("PASS", filename)
    print("PASS original reference SHA-256 / 1190 parameters / frame18 / LOG_DISARMED=0")


if __name__ == "__main__":
    main()
