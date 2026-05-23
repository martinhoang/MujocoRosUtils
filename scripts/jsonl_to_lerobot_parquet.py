#!/usr/bin/env python3
"""Convert a SimDataAggregator JSONL episode file to LeRobot-compatible Parquet.

Requirements:
    pip install pyarrow          # already a hard dep of lerobot
    # OR:
    pip install polars

Usage:
    python3 jsonl_to_lerobot_parquet.py <episode.jsonl> <episode.parquet>
"""
import json
import sys
from pathlib import Path


def convert(jsonl_path: str, parquet_path: str) -> None:
    rows = []
    with open(jsonl_path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))

    if not rows:
        print(f"[jsonl_to_lerobot_parquet] No rows found in {jsonl_path}", file=sys.stderr)
        sys.exit(1)

    # ── Try pyarrow first (lerobot hard-dependency) ──────────────────────────
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq

        # Build per-column arrays with explicit typing so LeRobot's loader is happy.
        n = len(rows)
        first = rows[0]

        def float32_list_col(key):
            return pa.array([r[key] for r in rows],
                            type=pa.list_(pa.float32()))

        def float32_col(key):
            return pa.array([float(r[key]) for r in rows], type=pa.float32())

        def int64_col(key):
            return pa.array([int(r[key]) for r in rows], type=pa.int64())

        def bool_col(key):
            return pa.array([bool(r[key]) for r in rows], type=pa.bool_())

        arrays = {
            "observation.state": float32_list_col("observation.state"),
            "action":            float32_list_col("action"),
            "timestamp":         float32_col("timestamp"),
            "frame_index":       int64_col("frame_index"),
            "episode_index":     int64_col("episode_index"),
            "index":             int64_col("index"),
            "task_index":        int64_col("task_index"),
            "next.done":         bool_col("next.done"),
        }

        table = pa.table(arrays)
        pq.write_table(table, parquet_path, compression="snappy")
        print(f"[jsonl_to_lerobot_parquet] Written {n} rows → {parquet_path}  (pyarrow)")
        return

    except ImportError:
        pass

    # ── Fallback: polars ─────────────────────────────────────────────────────
    try:
        import polars as pl

        df = pl.from_dicts(rows)
        df.write_parquet(parquet_path, compression="snappy")
        print(f"[jsonl_to_lerobot_parquet] Written {len(rows)} rows → {parquet_path}  (polars)")
        return

    except ImportError:
        pass

    print(
        "[jsonl_to_lerobot_parquet] ERROR: Neither pyarrow nor polars is installed.\n"
        "  Install one:  pip install pyarrow\n"
        f"  The raw JSONL is still at: {jsonl_path}",
        file=sys.stderr,
    )
    sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <episode.jsonl> <episode.parquet>", file=sys.stderr)
        sys.exit(1)
    convert(sys.argv[1], sys.argv[2])
