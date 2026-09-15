"""Side-by-side table of measure_bench.py JSON outputs.

    python bench_table.py name1=a.json name2=b.json ...
"""
import json
import sys

ROWS = [
    ("flat: settles at % of command", lambda m: m["response"]["steady_pct"], "{:.1f}"),
    ("flat: 90 % rise time, s", lambda m: m["response"]["t90"], "{:.2f}"),
    ("flat: stop distance, cm", lambda m: m["response"]["stop_dist_cm"], "{:.1f}"),
    ("flat: backward after request, %", lambda m: m["response"]["backward_pct"], "{:.1f}"),
    ("turn 1.0 rad/s tracking, %", lambda m: m["turn"]["pct"], "{:.1f}"),
    ("turn drift, m/s", lambda m: m["turn"]["drift"], "{:.3f}"),
    ("flat front td vs hip, cm", lambda m: m["td_flat_front"]["td_cm"], "{:+.1f}"),
    ("flat rear td vs hip, cm", lambda m: m["td_flat_rear"]["td_cm"], "{:+.1f}"),
    ("flat front td - neutral, cm", lambda m: m["td_flat_front"]["td_minus_neutral_cm"], "{:+.1f}"),
    ("flat rear td - neutral, cm", lambda m: m["td_flat_rear"]["td_minus_neutral_cm"], "{:+.1f}"),
    ("flat front-rear gap at td, cm", lambda m: m["td_flat_front"]["gap_cm"], "{:.1f}"),
    ("flat stride, cm", lambda m: m["td_flat_front"]["stride_cm"], "{:.1f}"),
    ("flat stance, ms", lambda m: m["td_flat_front"]["stance_ms"], "{:.0f}"),
    ("flat pitch wobble, deg", lambda m: m["stab_flat"]["wobble_deg"], "{:.2f}"),
    ("flat slipping, % loaded", lambda m: m["stab_flat"]["slipping_pct"], "{:.1f}"),
    ("up4: first step, %", lambda m: m["stairs_up4"]["one"], "{:.0f}"),
    ("up4: whole flight, %", lambda m: m["stairs_up4"]["finish"], "{:.0f}"),
    ("up4: fell, %", lambda m: m["stairs_up4"]["fell"], "{:.0f}"),
    ("up4: s per step", lambda m: m["stairs_up4"]["s_per_step"], "{:.2f}"),
    ("up11: 4 steps, %", lambda m: m["stairs_up11"]["flight"], "{:.0f}"),
    ("up11: whole flight, %", lambda m: m["stairs_up11"]["finish"], "{:.0f}"),
    ("down4: whole flight, %", lambda m: m["stairs_down4"]["finish"], "{:.0f}"),
    ("down11: whole flight, %", lambda m: m["stairs_down11"]["finish"], "{:.0f}"),
    ("down11: fell, %", lambda m: m["stairs_down11"]["fell"], "{:.0f}"),
    ("up4 footfalls mid-tread, %", lambda m: m["footfall_up4_all"]["tread"], "{:.0f}"),
    ("up4 footfalls on face, %", lambda m: m["footfall_up4_all"]["face"], "{:.0f}"),
    ("down11 footfalls over nosing, %", lambda m: m["footfall_down11_all"]["front"], "{:.0f}"),
    ("down11 footfalls on face, %", lambda m: m["footfall_down11_all"]["face"], "{:.0f}"),
    ("down11 slipping, % loaded", lambda m: m["stab_down11"]["slipping_pct"], "{:.1f}"),
]


def main():
    cols = []
    for a in sys.argv[1:]:
        name, path = a.split("=", 1)
        with open(path) as f:
            cols.append((name, json.load(f)))
    w = max(12, *(len(n) + 1 for n, _ in cols))
    print(f"{'':34}" + "".join(f"{n:>{w}}" for n, _ in cols))
    for label, fn, fmt in ROWS:
        cells = []
        for _, m in cols:
            try:
                cells.append(fmt.format(fn(m)))
            except (KeyError, TypeError, ValueError):
                cells.append("-")
        print(f"{label:34}" + "".join(f"{c:>{w}}" for c in cells))


if __name__ == "__main__":
    main()
