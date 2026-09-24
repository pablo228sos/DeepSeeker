#!/usr/bin/env python3
"""Recalculate DeepSeeker mission summaries and route diagnostics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable


EARTH_RADIUS_M = 6_371_000.0
REQUIRED_COLUMNS = {
    "image",
    "utc",
    "latitude",
    "longitude",
    "satellites",
    "hdop",
    "speed_kmph",
    "stable",
    "capture_ms",
}


def parse_utc(value: str) -> datetime:
    return datetime.fromisoformat(value.strip().replace("Z", "+00:00"))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        columns = set(reader.fieldnames or [])
        missing = REQUIRED_COLUMNS - columns
        if missing:
            names = ", ".join(sorted(missing))
            raise ValueError(f"{path} is missing required columns: {names}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} contains no telemetry rows")
    return rows


def numeric(rows: Iterable[dict[str, str]], column: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(column, "").strip()
        if value:
            values.append(float(value))
    return values


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def local_xy(rows: list[dict[str, str]]) -> tuple[list[float], list[float]]:
    lat0 = float(rows[0]["latitude"])
    lon0 = float(rows[0]["longitude"])
    lat0_rad = math.radians(lat0)
    east: list[float] = []
    north: list[float] = []
    for row in rows:
        lat = float(row["latitude"])
        lon = float(row["longitude"])
        east.append(math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(lat0_rad))
        north.append(math.radians(lat - lat0) * EARTH_RADIUS_M)
    return east, north


def mission_summary(rows: list[dict[str, str]]) -> dict[str, Any]:
    timestamps = [parse_utc(row["utc"]) for row in rows]
    latitudes = numeric(rows, "latitude")
    longitudes = numeric(rows, "longitude")
    speeds = numeric(rows, "speed_kmph")
    captures = numeric(rows, "capture_ms")
    satellites = numeric(rows, "satellites")
    hdop = numeric(rows, "hdop")
    stable = numeric(rows, "stable")
    jpeg_bytes = numeric(rows, "jpeg_bytes")

    raw_track_length = sum(
        haversine_m(latitudes[i - 1], longitudes[i - 1], latitudes[i], longitudes[i])
        for i in range(1, len(rows))
    )
    endpoint_distance = haversine_m(
        latitudes[0], longitudes[0], latitudes[-1], longitudes[-1]
    )
    east, north = local_xy(rows)

    return {
        "frames": len(rows),
        "first_utc": min(timestamps).isoformat().replace("+00:00", "Z"),
        "last_utc": max(timestamps).isoformat().replace("+00:00", "Z"),
        "duration_s": (max(timestamps) - min(timestamps)).total_seconds(),
        "median_speed_kmph": statistics.median(speeds),
        "median_speed_mps": statistics.median(speeds) / 3.6,
        "median_capture_ms": statistics.median(captures),
        "median_satellites": statistics.median(satellites),
        "min_satellites": min(satellites),
        "median_hdop": statistics.median(hdop),
        "max_hdop": max(hdop),
        "stable_fraction": sum(1 for value in stable if value >= 0.5) / len(stable),
        "median_jpeg_bytes": statistics.median(jpeg_bytes) if jpeg_bytes else None,
        "raw_gps_polyline_m": raw_track_length,
        "endpoint_distance_m": endpoint_distance,
        "east_span_m": max(east) - min(east),
        "north_span_m": max(north) - min(north),
    }


def quality_values(path: Path, images: list[str]) -> list[float]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    by_image = {row["image"]: float(row["quality_score"]) for row in rows}
    return [by_image.get(image, math.nan) for image in images]


def plot_mission(
    telemetry_path: Path,
    quality_path: Path | None,
    output_path: Path,
    turn_start: int | None,
    turn_end: int | None,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("Plotting requires matplotlib. Install analysis/requirements.txt") from exc

    rows = read_csv(telemetry_path)
    east, north = local_xy(rows)
    frame_numbers = list(range(1, len(rows) + 1))
    images = [row["image"] for row in rows]
    quality = (
        quality_values(quality_path, images)
        if quality_path
        else [float(row["stable"]) for row in rows]
    )

    fig, (track_ax, quality_ax) = plt.subplots(1, 2, figsize=(13, 5.2), constrained_layout=True)
    track_ax.plot(east, north, color="#94a3b8", linewidth=1.0, zorder=1)
    points = track_ax.scatter(
        east,
        north,
        c=frame_numbers,
        cmap="viridis",
        s=18,
        edgecolors="none",
        zorder=2,
    )
    track_ax.scatter(east[0], north[0], marker="o", s=70, color="#16a34a", label="Start")
    track_ax.scatter(east[-1], north[-1], marker="x", s=70, color="#dc2626", label="End")
    track_ax.set_title("GNSS track in local coordinates")
    track_ax.set_xlabel("East (m)")
    track_ax.set_ylabel("North (m)")
    track_ax.axis("equal")
    track_ax.grid(alpha=0.25)
    track_ax.legend(loc="best")
    fig.colorbar(points, ax=track_ax, label="Frame number")

    quality_ax.plot(frame_numbers, quality, color="#0f6aa6", linewidth=1.4)
    if turn_start is not None and turn_end is not None:
        quality_ax.axvspan(turn_start, turn_end, color="#bae6fd", alpha=0.6, label="Turn excluded")
        quality_ax.legend(loc="best")
    quality_ax.set_title("Image quality through mission")
    quality_ax.set_xlabel("Frame")
    quality_ax.set_ylabel("Normalized quality score" if quality_path else "Stable flag")
    quality_ax.set_ylim(bottom=0)
    quality_ax.grid(alpha=0.25)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def summarize_command(args: argparse.Namespace) -> int:
    summary = mission_summary(read_csv(args.telemetry))
    rendered = json.dumps(summary, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


def plot_command(args: argparse.Namespace) -> int:
    plot_mission(
        telemetry_path=args.telemetry,
        quality_path=args.quality_csv,
        output_path=args.output,
        turn_start=args.turn_start,
        turn_end=args.turn_end,
    )
    print(args.output)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    summarize = subparsers.add_parser("summarize", help="print mission statistics as JSON")
    summarize.add_argument("telemetry", type=Path)
    summarize.add_argument("--output", type=Path)
    summarize.set_defaults(handler=summarize_command)

    plot = subparsers.add_parser("plot", help="create a route and image-quality diagnostic")
    plot.add_argument("telemetry", type=Path)
    plot.add_argument("--quality-csv", type=Path)
    plot.add_argument("--output", type=Path, required=True)
    plot.add_argument("--turn-start", type=int)
    plot.add_argument("--turn-end", type=int)
    plot.set_defaults(handler=plot_command)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
