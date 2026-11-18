"""Build opening and closing curves from local measurements using the standard library."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


FIELDS = ["direction", "angle_rad", "opening_mm"]
DIRECTIONS = ("opening", "closing")


def load_measurements(path):
    observations = {direction: {} for direction in DIRECTIONS}
    with path.open(encoding="utf-8-sig", newline="") as source:
        rows = csv.DictReader(source, strict=True)
        if rows.fieldnames != FIELDS:
            raise ValueError(f"expected columns: {','.join(FIELDS)}")
        for row in rows:
            line = rows.line_num
            direction = (row["direction"] or "").strip().lower()
            if None in row or direction not in observations:
                raise ValueError(f"line {line}: fill all columns, using opening or closing")
            try:
                angle = float(row["angle_rad"])
                opening = float(row["opening_mm"])
            except (ValueError, TypeError):
                raise ValueError(f"line {line}: angle and opening must be numbers") from None
            if not math.isfinite(angle) or not math.isfinite(opening) or opening < 0:
                raise ValueError(f"line {line}: use finite numbers and nonnegative opening")
            observations[direction].setdefault(angle, []).append(opening)
    return observations


def make_curve(observations, direction):
    if len(observations) < 3:
        raise ValueError(f"{direction}: at least three distinct measured angles required")
    knots = []
    for angle, readings in sorted(observations.items()):
        width = median(readings)
        spread = max(readings) - min(readings)
        if not math.isfinite(width) or not math.isfinite(spread):
            raise ValueError(f"{direction}: opening values are outside the numeric range")
        knots.append(
            {
                "angle_rad": angle,
                "opening_mm": width,
                "repeat_count": len(readings),
                "repeat_span_mm": spread,
            }
        )
    steps = {
        (right["opening_mm"] > left["opening_mm"]) - (right["opening_mm"] < left["opening_mm"])
        for left, right in zip(knots, knots[1:])
    } - {0}
    if len(steps) != 1:
        raise ValueError(f"{direction}: opening must vary monotonically over the measured range")
    return {
        "opening_increases_with_angle": steps == {1},
        "measured_angle_interval_rad": [knots[0]["angle_rad"], knots[-1]["angle_rad"]],
        "knots": knots,
    }


def fit(path):
    observations = load_measurements(path)
    curves = {
        direction: make_curve(observations[direction], direction)
        for direction in DIRECTIONS
    }
    if curves["opening"]["opening_increases_with_angle"] != curves["closing"]["opening_increases_with_angle"]:
        raise ValueError("the two curves disagree on the angle direction")
    return {
        "schema_version": 1,
        "method": "piecewise_linear",
        "extrapolation": "disabled",
        "angle_unit": "rad",
        "opening_unit": "mm",
        "angle_reference": "same continuous motor output angle used in both sweeps",
        "curves": curves,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurements", type=Path)
    parser.add_argument("--out", type=Path, required=True, help="new JSON output file")
    args = parser.parse_args()
    try:
        result = fit(args.measurements)
        with args.out.open("x", encoding="utf-8", newline="\n") as output:
            json.dump(result, output, ensure_ascii=False, allow_nan=False, indent=2)
            output.write("\n")
    except (OSError, ValueError, csv.Error) as error:
        parser.exit(1, f"{parser.prog}: {error}\n")
    print(f"Wrote {args.out.name}")


if __name__ == "__main__":
    main()
