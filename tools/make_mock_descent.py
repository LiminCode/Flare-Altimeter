"""Generate mock TF02-Pro descents for replay testing.

Written as a generator rather than hand-edited CSVs so the awkward parts are
deliberate and reproducible, not accidents of whatever happened to get recorded.

Each file carries its own **expectations** in the header, as `# expect_*:` lines.
The checker reads them instead of hard-coding what a good run looks like, because
a good run is not the same shape for every scenario -- "dropouts appear as gaps"
is a pass for the hostile file and a failure for the clean one. Keeping the
expectations next to the data is also what makes a new scenario a data change
rather than a code change.

中文说明（为什么期望值写在数据文件里，而不是写在检查代码里）：
  因为「一次好的运行」在不同场景下形状根本不同 ——
  「丢帧应该表现为断线」对恶劣场景是通过，对完美场景就是失败。
  期望值跟数据放在一起，还有一个好处：以后加一个新场景是改数据，不是改代码。

Usage:
    python tools/make_mock_descent.py            # write every scenario
    python tools/make_mock_descent.py perfect    # just one
"""

from __future__ import annotations

import os
import random
import sys

SAMPLE_HZ = 100
DT_MS = 1000 // SAMPLE_HZ
GROUND_CM = 12.0
SEED = 20260822

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mock")


def strength_for(height_cm: float, scale: float = 1.0) -> int:
    """Return strength falls off roughly as 1/r^1.6 over grass."""
    s = int(2600.0 * scale * (300.0 / max(height_cm, 50.0)) ** 1.6)
    return max(120, min(2800, s))


def rate_at(h: float, cruise: float, flare_from: float) -> float:
    """Descent rate at a height: cruise, then the canopy round-out near ground."""
    if h <= flare_from:
        return cruise * max(0.12, h / flare_from)
    return cruise


def build(name: str, *, start_cm: float, cruise_cm_s: float,
          cruise_end_cm_s: float | None = None, flare_from_cm: float = 250.0,
          noise_cm: float = 2.5, strength_scale: float = 1.0,
          dropouts: list[tuple[float, float]] | None = None,
          weak: list[tuple[float, float]] | None = None,
          ground_dwell_s: float = 4.0, expect: dict[str, str] | None = None,
          description: str = "") -> tuple[list[tuple[int, int, int]], dict]:
    rng = random.Random(SEED + sum(ord(c) for c in name))
    dropouts = dropouts or []
    weak = weak or []

    rows: list[tuple[int, int, int]] = []
    t_ms = 0
    h = start_cm

    def in_bands(height: float, bands: list[tuple[float, float]]) -> bool:
        return any(lo <= height <= hi for lo, hi in bands)

    while h > GROUND_CM:
        # Cruise rate may ramp across the descent: a canopy that is accelerating
        # (a turn, a dive) must make the beeps speed up, and that is the single
        # property the whole alert model rests on.
        if cruise_end_cm_s is None:
            cruise = cruise_cm_s
        else:
            frac = 1.0 - (h - GROUND_CM) / max(1.0, start_cm - GROUND_CM)
            cruise = cruise_cm_s + (cruise_end_cm_s - cruise_cm_s) * frac
        h -= rate_at(h, cruise, flare_from_cm) * (DT_MS / 1000.0)
        h = max(h, GROUND_CM)

        measured = h + rng.gauss(0.0, noise_cm)
        if in_bands(h, dropouts):
            # No echo at all: the sensor still emits frames, the distance is
            # garbage and the strength sits at the floor.
            rows.append((t_ms, 65535, 3))
        elif in_bands(h, weak):
            rows.append((t_ms, int(measured), rng.randint(20, 95)))
        else:
            rows.append((t_ms, int(measured), strength_for(h, strength_scale)))
        t_ms += DT_MS

    for _ in range(int(ground_dwell_s * SAMPLE_HZ)):
        rows.append((t_ms, int(GROUND_CM + rng.gauss(0.0, 1.0)),
                     strength_for(GROUND_CM, strength_scale)))
        t_ms += DT_MS

    meta = dict(expect or {})
    meta["scenario"] = name
    meta["description"] = description
    return rows, meta


# --- the scenarios ---------------------------------------------------------
#
# Between them these cover: everything working, everything hostile, the descent
# rate changing under the alert model, and a case where the correct behaviour is
# to do nothing at all.
SCENARIOS = {
    # Clean 20 m descent. Nothing to trip over -- this is the file that should
    # fail loudly if something basic breaks, because it has no excuses in it.
    "perfect": dict(
        description="clean 20 m descent, strong returns, no dropouts",
        start_cm=2000.0, cruise_cm_s=450.0, noise_cm=1.0, strength_scale=1.4,
        expect={
            "expect_arm": "yes",
            "expect_flare": "yes",
            "expect_gaps": "no",
            "expect_peak_cm_min": "1800",
            "expect_floor_cm_max": "60",
            "expect_stage_order": "0,1,2,3,4,0",
            "expect_flare_above_cm": "none",
            "expect_beeps_min": "25",
        },
    ),
    # Hostile 10 m descent: a dark or wet field, a swinging foot, a sensor near
    # the edge of its range. Repeated dropouts and weak returns, heavy noise.
    # The device must still get to a flare cue without ever crying wolf at
    # altitude -- the dropouts are placed high on purpose.
    "bad": dict(
        description="hostile 10 m descent: repeated dropouts, weak returns, noisy",
        start_cm=1000.0, cruise_cm_s=520.0, noise_cm=6.0, strength_scale=0.45,
        dropouts=[(880.0, 940.0), (600.0, 640.0), (330.0, 360.0)],
        weak=[(760.0, 820.0), (430.0, 470.0)],
        expect={
            "expect_arm": "yes",
            "expect_flare": "yes",
            "expect_gaps": "yes",
            "expect_peak_cm_min": "850",
            "expect_floor_cm_max": "60",
            # No 'far' stage (1): that band is above 10 m, and this scenario
            # starts AT 10 m of slant range -- which the tilt correction turns
            # into about 9.85 m of height before the alert engine ever sees it.
            # Expecting stage 1 here would be expecting the firmware to be wrong.
            # 中文：没有 far 档（1）。那一档在 10 m 以上，而这个场景的起点就是
            #       10 m 斜距 —— 经倾角修正后约 9.85 m，警报引擎看到的从一开始
            #       就低于那条线。在这里期待出现 far 档，等于期待固件算错。
            "expect_stage_order": "0,2,3,4,0",
            "expect_flare_above_cm": "none",
            "expect_beeps_min": "8",
        },
    ),
    # Descent that accelerates from 2 m/s to 7 m/s -- a canopy diving, or simply
    # a heavier wing loading late. Beep rate must rise with it, because rate IS
    # the descent-rate channel. This is the scenario that would expose a design
    # that silently degraded into fixed-cadence beeping.
    "accel": dict(
        description="20 m descent accelerating 2 m/s -> 7 m/s; beep rate must follow",
        start_cm=2000.0, cruise_cm_s=200.0, cruise_end_cm_s=700.0, noise_cm=2.0,
        expect={
            "expect_arm": "yes",
            "expect_flare": "yes",
            "expect_gaps": "no",
            "expect_peak_cm_min": "1800",
            "expect_floor_cm_max": "60",
            "expect_stage_order": "0,1,2,3,4,0",
            "expect_flare_above_cm": "none",
            "expect_beeps_min": "25",
            "expect_beep_rate_rises": "yes",
        },
    ),
    # Negative space: a slow drift downwards that never gets high enough to arm,
    # which is what walking around the DZ with the unit on your ankle looks like.
    # The correct behaviour is total silence, and nothing else in the suite tests
    # for the absence of sound over a long run.
    "noarm": dict(
        description="low slow drift that must never arm -- walking around the DZ",
        start_cm=600.0, cruise_cm_s=40.0, flare_from_cm=100.0, noise_cm=3.0,
        ground_dwell_s=3.0,
        expect={
            "expect_arm": "no",
            "expect_flare": "no",
            "expect_gaps": "no",
            "expect_stage_order": "0",
            "expect_beeps_max": "0",
        },
    ),
}


def write_scenario(name: str) -> str:
    cfg = dict(SCENARIOS[name])
    description = cfg.pop("description")
    expect = cfg.pop("expect")
    rows, _meta = build(name, description=description, expect=expect, **cfg)

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"{name}.csv")
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# scenario: {name}\n")
        f.write(f"# description: {description}\n")
        f.write(f"# {SAMPLE_HZ} Hz. dist_cm is the SLANT range as the sensor\n")
        f.write("# reports it; strength < 100 or == 65535 carries no usable range.\n")
        for k, v in expect.items():
            f.write(f"# {k}: {v}\n")
        f.write("t_ms,dist_cm,strength\n")
        for t, d, s in rows:
            f.write(f"{t},{d},{s}\n")

    dur = rows[-1][0] / 1000.0
    print(f"{name:8s} {len(rows):5d} samples  {dur:5.1f} s  -> {path}")
    print(f"         {description}")
    return path


def main() -> int:
    wanted = sys.argv[1:] or list(SCENARIOS)
    for name in wanted:
        if name not in SCENARIOS:
            print(f"unknown scenario {name!r}; known: {', '.join(SCENARIOS)}")
            return 2
        write_scenario(name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
