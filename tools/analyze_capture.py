#!/usr/bin/env python3
"""Offline analyzer for XPlaneTruthCapture run folders."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


KEY_COLUMNS = {
    "sim/time/total_flight_time_sec": "sim_time_s",
    "sim/operation/misc/frame_rate_period": "frame_period_s",
    "sim/flightmodel/position/latitude": "lat",
    "sim/flightmodel/position/longitude": "lon",
    "sim/flightmodel/position/local_x": "local_x",
    "sim/flightmodel/position/local_y": "local_y",
    "sim/flightmodel/position/local_z": "local_z",
    "sim/flightmodel/position/elevation": "alt_m",
    "sim/flightmodel/position/y_agl": "agl_m",
    "sim/flightmodel/position/local_vx": "local_vx_mps",
    "sim/flightmodel/position/groundspeed": "speed_mps",
    "sim/flightmodel/position/local_vy": "vertical_speed_mps",
    "sim/flightmodel/position/local_vz": "local_vz_mps",
    "sim/flightmodel/position/hpath": "true_track_deg",
    "sim/cockpit2/gauges/indicators/ground_track_mag_copilot": "mag_track_deg",
    "sim/flightmodel/position/phi": "roll_deg",
    "sim/flightmodel/position/theta": "pitch_deg",
    "sim/flightmodel/position/psi": "yaw_deg",
    "sim/flightmodel/position/mag_psi": "mag_yaw_deg",
    "sim/flightmodel/position/magnetic_variation": "mag_variation_deg",
    "sim/flightmodel/position/Prad": "p_rad_s",
    "sim/flightmodel/position/Qrad": "q_rad_s",
    "sim/flightmodel/position/Rrad": "r_rad_s",
    "sim/flightmodel/forces/g_axil": "g_axil",
    "sim/flightmodel/forces/g_side": "g_side",
    "sim/flightmodel/forces/g_nrml": "g_nrml",
    "sim/flightmodel/position/indicated_airspeed": "ias",
    "sim/flightmodel/position/true_airspeed": "tas",
}


class OnlineStats:
    def __init__(self) -> None:
        self.count = 0
        self.array_rows = 0
        self.blank_count = 0
        self.non_numeric_count = 0
        self.minimum = math.nan
        self.maximum = math.nan
        self.mean = 0.0
        self.m2 = 0.0
        self.first = ""
        self.last = ""

    def observe_raw(self, raw: str) -> None:
        if raw == "":
            self.blank_count += 1
            return
        if self.first == "":
            self.first = raw
        self.last = raw

        values = numeric_values(raw)
        if not values:
            self.non_numeric_count += 1
            return
        if ";" in raw:
            self.array_rows += 1

        for value in values:
            self.count += 1
            if self.count == 1:
                self.minimum = value
                self.maximum = value
                self.mean = value
                self.m2 = 0.0
                continue
            self.minimum = min(self.minimum, value)
            self.maximum = max(self.maximum, value)
            delta = value - self.mean
            self.mean += delta / self.count
            self.m2 += delta * (value - self.mean)

    def row(self, path: str, metadata: dict[str, str]) -> dict[str, object]:
        stddev = math.sqrt(self.m2 / (self.count - 1)) if self.count > 1 else math.nan
        return {
            "path": path,
            "group": metadata.get("group", ""),
            "unit_hint": metadata.get("unit_hint", ""),
            "exists": metadata.get("exists", ""),
            "type_names": metadata.get("type_names", ""),
            "count": self.count,
            "array_rows": self.array_rows,
            "blank_count": self.blank_count,
            "non_numeric_count": self.non_numeric_count,
            "min": finite_or_blank(self.minimum),
            "max": finite_or_blank(self.maximum),
            "mean": finite_or_blank(self.mean if self.count else math.nan),
            "stddev": finite_or_blank(stddev),
            "first": self.first,
            "last": self.last,
        }


def to_float(raw: str) -> float:
    if raw == "" or ";" in raw or raw.startswith("<"):
        return math.nan
    try:
        value = float(raw)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def numeric_values(raw: str) -> list[float]:
    if raw == "" or raw.startswith("<"):
        return []
    if ";" not in raw:
        value = to_float(raw)
        return [value] if math.isfinite(value) else []
    values: list[float] = []
    for part in raw.split(";"):
        try:
            value = float(part)
        except ValueError:
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def finite_or_blank(value: float) -> object:
    return value if math.isfinite(value) else ""


def quantile(values: list[float], q: float) -> float:
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    idx = round((len(values) - 1) * q)
    return values[max(0, min(len(values) - 1, idx))]


def stats(values: list[float]) -> dict[str, object]:
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return {"count": 0, "min": None, "p50": None, "p95": None, "p99": None, "max": None, "mean": None}
    return {
        "count": len(values),
        "min": min(values),
        "p50": quantile(values, 0.50),
        "p95": quantile(values, 0.95),
        "p99": quantile(values, 0.99),
        "max": max(values),
        "mean": sum(values) / len(values),
    }


def read_json(path: Path) -> object:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def read_events(path: Path) -> list[dict[str, object]]:
    events: list[dict[str, object]] = []
    if not path.exists():
        return events
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                events.append({"type": "parse_warning", "message": line[:200]})
    return events


def read_datarefs(path: Path) -> tuple[list[dict[str, str]], dict[str, dict[str, str]]]:
    if not path.exists():
        return [], {}
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    return rows, {row.get("path", ""): row for row in rows}


def write_jsonl(path: Path, issues: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for issue in issues:
            f.write(json.dumps(issue, ensure_ascii=False, separators=(",", ":")) + "\n")


def analyze(run_dir: Path) -> dict[str, object]:
    frames_path = run_dir / "frames.csv"
    if not frames_path.exists():
        raise FileNotFoundError(f"frames.csv not found in {run_dir}")

    manifest = read_json(run_dir / "manifest.json")
    summary = read_json(run_dir / "summary.json")
    events = read_events(run_dir / "events.jsonl")
    dataref_rows, dataref_by_path = read_datarefs(run_dir / "datarefs.csv")

    issues: list[dict[str, object]] = []
    for row in dataref_rows:
        if row.get("required") == "true" and row.get("exists") != "true":
            issues.append({"severity": "error", "type": "missing_required_dataref", "path": row.get("path", "")})

    event_counts = Counter(str(event.get("type", "")) for event in events)
    timing_dt: list[float] = []
    frame_period: list[float] = []
    frame_ids: list[int] = []
    sim_times: list[float] = []
    segments: list[dict[str, object]] = []
    segment_id = 0
    segment_start_frame: int | None = None
    segment_start_sim_time = math.nan
    segment_last_frame: int | None = None
    segment_last_sim_time = math.nan
    segment_rows = 0
    monotonic_sim_duration_s = 0.0
    sim_time_resets = 0
    stats_by_column: dict[str, OnlineStats] = {}
    derived_rows: list[dict[str, object]] = []
    previous_frame_id: int | None = None
    previous_sim_time = math.nan
    first_host_ns = math.nan

    def close_segment() -> None:
        nonlocal segment_start_frame, segment_start_sim_time, segment_last_frame, segment_last_sim_time, segment_rows
        if segment_rows <= 0 or segment_start_frame is None or segment_last_frame is None:
            return
        segments.append({
            "segment_id": segment_id,
            "start_frame_id": segment_start_frame,
            "end_frame_id": segment_last_frame,
            "rows": segment_rows,
            "start_sim_time_s": finite_or_blank(segment_start_sim_time),
            "end_sim_time_s": finite_or_blank(segment_last_sim_time),
            "duration_s": finite_or_blank(segment_last_sim_time - segment_start_sim_time
                                          if math.isfinite(segment_start_sim_time) and math.isfinite(segment_last_sim_time)
                                          else math.nan),
        })
        segment_start_frame = None
        segment_start_sim_time = math.nan
        segment_last_frame = None
        segment_last_sim_time = math.nan
        segment_rows = 0

    with frames_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row_index, row in enumerate(reader, start=1):
            frame_id = int(to_float(row.get("frame_id", "")) or 0)
            frame_ids.append(frame_id)
            if previous_frame_id is not None and frame_id != previous_frame_id + 1:
                issues.append({"severity": "warning", "type": "frame_id_gap", "frame_id": frame_id, "previous": previous_frame_id})
            previous_frame_id = frame_id

            dt = to_float(row.get("elapsed_since_last_call_s", ""))
            if row_index > 1 and math.isfinite(dt):
                timing_dt.append(dt)
                if dt > 0.2:
                    issues.append({"severity": "warning", "type": "host_frame_stall", "frame_id": frame_id, "dt_s": dt})

            period = to_float(row.get("sim/operation/misc/frame_rate_period", ""))
            if math.isfinite(period):
                frame_period.append(period)

            sim_time = to_float(row.get("sim/time/total_flight_time_sec", ""))
            if math.isfinite(sim_time):
                sim_times.append(sim_time)
                if math.isfinite(previous_sim_time) and sim_time < previous_sim_time:
                    issues.append({"severity": "warning", "type": "non_monotonic_sim_time", "frame_id": frame_id})
                    sim_time_resets += 1
                    close_segment()
                    segment_id += 1
                elif math.isfinite(previous_sim_time):
                    monotonic_sim_duration_s += max(0.0, sim_time - previous_sim_time)
                previous_sim_time = sim_time

            if segment_start_frame is None:
                segment_start_frame = frame_id
                segment_start_sim_time = sim_time
            segment_last_frame = frame_id
            segment_last_sim_time = sim_time
            segment_rows += 1

            for column, raw in row.items():
                if column not in stats_by_column:
                    stats_by_column[column] = OnlineStats()
                stats_by_column[column].observe_raw(raw or "")

            host_ns = to_float(row.get("host_ns", ""))
            if math.isfinite(host_ns) and not math.isfinite(first_host_ns):
                first_host_ns = host_ns
            host_elapsed_s = (host_ns - first_host_ns) / 1e9 if math.isfinite(host_ns) and math.isfinite(first_host_ns) else math.nan
            segment_time_s = (sim_time - segment_start_sim_time
                              if math.isfinite(sim_time) and math.isfinite(segment_start_sim_time)
                              else math.nan)

            derived = {
                "frame_id": frame_id,
                "segment_id": segment_id,
                "segment_time_s": finite_or_blank(segment_time_s),
                "host_elapsed_s": finite_or_blank(host_elapsed_s),
            }
            for source, target in KEY_COLUMNS.items():
                derived[target] = finite_or_blank(to_float(row.get(source, "")))
            derived_rows.append(derived)
    close_segment()

    if isinstance(summary, dict) and int(summary.get("rows_dropped", 0) or 0) > 0:
        issues.append({"severity": "warning", "type": "rows_dropped", "rows_dropped": summary.get("rows_dropped")})
    if event_counts.get("marker", 0) == 0:
        issues.append({"severity": "info", "type": "no_user_markers", "message": "Use Mark Event or command markers for controlled sign tests."})

    dataref_stats_path = run_dir / "dataref_stats.csv"
    with dataref_stats_path.open("w", encoding="utf-8", newline="") as f:
        fieldnames = [
            "path", "group", "unit_hint", "exists", "type_names", "count", "array_rows", "blank_count",
            "non_numeric_count", "min", "max", "mean", "stddev", "first", "last",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for column, column_stats in stats_by_column.items():
            writer.writerow(column_stats.row(column, dataref_by_path.get(column, {})))

    derived_path = run_dir / "derived.csv"
    with derived_path.open("w", encoding="utf-8", newline="") as f:
        fieldnames = ["frame_id", "segment_id", "segment_time_s", "host_elapsed_s", *KEY_COLUMNS.values()]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(derived_rows)

    write_jsonl(run_dir / "issues.jsonl", issues)

    sim_duration = monotonic_sim_duration_s if len(sim_times) >= 2 else math.nan
    fps_values = [1.0 / p for p in frame_period if p > 0]
    analysis = {
        "schema": 1,
        "tool": "XPlaneTruthCapture analyze_capture.py",
        "run_dir": str(run_dir),
        "manifest": manifest,
        "summary": summary,
        "rows": len(frame_ids),
        "sim_duration_s": finite_or_blank(sim_duration),
        "sim_time_resets": sim_time_resets,
        "segments": segments,
        "timing": {
            "elapsed_since_last_call_s": stats(timing_dt),
            "frame_rate_period_s": stats(frame_period),
            "fps_from_frame_period": stats(fps_values),
        },
        "datarefs": {
            "requested": len(dataref_rows),
            "exists": sum(1 for row in dataref_rows if row.get("exists") == "true"),
            "missing_required": [row.get("path", "") for row in dataref_rows if row.get("required") == "true" and row.get("exists") != "true"],
        },
        "events": dict(event_counts),
        "issues": {
            "count": len(issues),
            "by_severity": dict(Counter(str(issue["severity"]) for issue in issues)),
        },
        "outputs": {
            "dataref_stats_csv": dataref_stats_path.name,
            "derived_csv": derived_path.name,
            "issues_jsonl": "issues.jsonl",
        },
    }
    with (run_dir / "analysis_summary.json").open("w", encoding="utf-8") as f:
        json.dump(analysis, f, indent=2)
        f.write("\n")
    return analysis


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze an XPlaneTruthCapture run folder.")
    parser.add_argument("run_dir", type=Path, help="Path to a run folder containing frames.csv")
    args = parser.parse_args()
    analysis = analyze(args.run_dir)
    print(json.dumps({
        "rows": analysis["rows"],
        "sim_duration_s": analysis["sim_duration_s"],
        "issues": analysis["issues"],
        "outputs": analysis["outputs"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
