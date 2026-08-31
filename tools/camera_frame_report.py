"""Summarize capture/send CSV files produced by DexFull Teleimager."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def _read(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _summary(path: Path) -> tuple[int, float, int, float]:
    rows = _read(path)
    if not rows:
        return 0, 0.0, 0, 0.0
    sequences = [int(row["sequence"]) for row in rows]
    timestamp_field = (
        "send_timestamp_ns" if "send_timestamp_ns" in rows[0] else "record_timestamp_ns"
    )
    timestamps = [int(row[timestamp_field]) for row in rows]
    duration = max(0, timestamps[-1] - timestamps[0]) / 1_000_000_000.0
    hz = (len(rows) - 1) / duration if duration > 0 and len(rows) > 1 else 0.0
    missing = sum(max(0, current - previous - 1) for previous, current in zip(sequences, sequences[1:]))
    max_gap_ms = max(
        (current - previous) / 1_000_000.0
        for previous, current in zip(timestamps, timestamps[1:])
    ) if len(timestamps) > 1 else 0.0
    return len(rows), hz, missing, max_gap_ms


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    args = parser.parse_args()

    print(f"{'file':48} {'frames':>8} {'Hz':>8} {'seq_missing':>12} {'max_gap_ms':>12}")
    for path in sorted(args.session_dir.glob("*_capture.csv")) + sorted(
        args.session_dir.glob("*_send.csv")
    ):
        count, hz, missing, max_gap_ms = _summary(path)
        print(f"{path.name:48} {count:8d} {hz:8.2f} {missing:12d} {max_gap_ms:12.2f}")


if __name__ == "__main__":
    main()

