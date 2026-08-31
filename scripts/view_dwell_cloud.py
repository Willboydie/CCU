#!/usr/bin/env python3
"""Visualise a WDL1 dwell point cloud with Open3D.

  pip install open3d numpy
  python scripts/view_dwell_cloud.py dwell_x3.00_y0.00_20260821_172530.bin
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

NEUTRAL_X = 3.54
NEUTRAL_Y = 3.39
HEADER_FMT = "<4sHHI13f"  # 4+2+2+4+13*4 = 64
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECORD_FMT = "<ffHH"
RECORD_SIZE = struct.calcsize(RECORD_FMT)

assert HEADER_SIZE == 64
assert RECORD_SIZE == 12


def load_wdl1(path: Path) -> tuple[dict, np.ndarray]:
    data = path.read_bytes()
    if len(data) < HEADER_SIZE:
        raise ValueError(f"{path} is too short to be a WDL1 file")

    magic, version, rec_size, count, *rest = struct.unpack_from(HEADER_FMT, data, 0)
    if magic != b"WDL1":
        raise ValueError(f"{path} is not a WDL1 file (magic={magic!r})")
    if version != 1 or rec_size != RECORD_SIZE:
        raise ValueError(f"unsupported WDL1 version={version} rec_size={rec_size}")

    keys = (
        "target_x",
        "target_y",
        "centre_az",
        "centre_el",
        "centre_mx",
        "centre_my",
        "amp_x",
        "amp_y",
        "freq_x",
        "freq_y",
        "phase_x",
        "phase_y",
        "duration_s",
    )
    header = dict(zip(keys, rest))
    header["count"] = count

    need = HEADER_SIZE + count * RECORD_SIZE
    if len(data) < need:
        raise ValueError(f"{path} truncated: expected {need} bytes, got {len(data)}")

    records = np.frombuffer(data, dtype=np.dtype([
        ("angle_x", "<f4"),
        ("angle_y", "<f4"),
        ("distance_cm", "<u2"),
        ("flags", "<u2"),
    ]), count=count, offset=HEADER_SIZE)
    return header, records


def records_to_xyz(records: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    valid = (records["flags"] & 1) != 0
    rec = records[valid]
    r = rec["distance_cm"].astype(np.float64) * 0.01
    theta = rec["angle_x"].astype(np.float64) - NEUTRAL_X
    phi = rec["angle_y"].astype(np.float64) - NEUTRAL_Y
    cphi = np.cos(phi)
    xyz = np.column_stack((
        r * np.cos(theta) * cphi,
        r * np.sin(theta) * cphi,
        r * np.sin(phi),
    ))
    return xyz, r


def colour_by_range(r: np.ndarray) -> np.ndarray:
    if r.size == 0:
        return np.zeros((0, 3))
    lo, hi = float(np.min(r)), float(np.max(r))
    span = hi - lo if hi > lo else 1.0
    t = (r - lo) / span
    # cyan → yellow → red
    return np.column_stack((t, 1.0 - 0.4 * t, 1.0 - t))


def print_header(path: Path, header: dict, n_valid: int, n_total: int) -> None:
    print(f"{path}")
    print(f"  target XY     ({header['target_x']:.3f}, {header['target_y']:.3f}) m")
    print(f"  pixel centre  az {header['centre_az']:.4f}  el {header['centre_el']:.4f} rad")
    print(f"  mechanical    ({header['centre_mx']:.3f}, {header['centre_my']:.3f}) rad")
    print(f"  amp           ({header['amp_x']:.3f}, {header['amp_y']:.3f}) rad")
    print(f"  freq          ({header['freq_x']:.2f}, {header['freq_y']:.2f}) Hz")
    print(f"  dwell         {header['duration_s']:.2f} s")
    print(f"  samples       {n_valid} valid / {n_total} total")


def visualise(xyz: np.ndarray, colours: np.ndarray, header: dict) -> None:
    import open3d as o3d

    geometries = []

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(colours)
    geometries.append(cloud)

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
    geometries.append(axes)

    tx, ty = float(header["target_x"]), float(header["target_y"])
    zmin = float(xyz[:, 2].min()) - 0.2 if xyz.size else -0.5
    zmax = float(xyz[:, 2].max()) + 0.2 if xyz.size else 0.5
    target_line = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([[tx, ty, zmin], [tx, ty, zmax]]),
        lines=o3d.utility.Vector2iVector([[0, 1]]),
    )
    target_line.colors = o3d.utility.Vector3dVector([[1.0, 0.85, 0.15]])
    geometries.append(target_line)

    o3d.visualization.draw_geometries(
        geometries,
        window_name="Dwell point cloud",
        width=1280,
        height=800,
        point_show_normal=False,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="View a WDL1 dwell cloud in Open3D")
    parser.add_argument("path", type=Path, help="path to a dwell_*.bin file")
    args = parser.parse_args()

    if not args.path.is_file():
        print(f"not found: {args.path}", file=sys.stderr)
        return 1

    header, records = load_wdl1(args.path)
    xyz, r = records_to_xyz(records)
    print_header(args.path, header, xyz.shape[0], records.shape[0])
    if xyz.shape[0] == 0:
        print("no valid returns to plot", file=sys.stderr)
        return 1

    print(f"  xyz range     x[{xyz[:, 0].min():.3f}, {xyz[:, 0].max():.3f}]  "
          f"y[{xyz[:, 1].min():.3f}, {xyz[:, 1].max():.3f}]  "
          f"z[{xyz[:, 2].min():.3f}, {xyz[:, 2].max():.3f}]")

    visualise(xyz, colour_by_range(r), header)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
