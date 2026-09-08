#!/usr/bin/env python3
"""Fit a radial cone to projector slope-map calibration data.

Each .bin record is packed little-endian (see lib/utils/projector_calibration.hpp):
  uint8  axis        0 = X, 1 = Y
  uint16 start       DAC at Begin measure
  float  derivative  f'(start) in rad / DAC count

The cone is a linear radial trend from the galvo origin (DAC 32768),
fit to derivative magnitude only:

  |f'(r)| = a + b * r
  r       = |DAC - origin|

X and Y samples share the same (a, b); they are treated as axis slices of one
radially symmetric surface.

  pip install numpy matplotlib
  python scripts/fit_galvo_cone.py path/to/galvo_slope_YYYYMMDD_HHMMSS.bin
  python scripts/fit_galvo_cone.py path/to/calibration_dir
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

RECORD_DTYPE = np.dtype({
    "names": ["axis", "start", "derivative"],
    "formats": ["u1", "<u2", "<f4"],
    "offsets": [0, 1, 3],
    "itemsize": 7,
})
RECORD_SIZE = 7
assert RECORD_DTYPE.itemsize == RECORD_SIZE

DAC_ORIGIN = 32768
AXIS_X = 0
AXIS_Y = 1


def load_calib(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) == 0:
        raise ValueError(f"{path} is empty")
    if len(data) % RECORD_SIZE != 0:
        raise ValueError(
            f"{path} length {len(data)} is not a multiple of {RECORD_SIZE} bytes"
        )
    rec = np.frombuffer(data, dtype=RECORD_DTYPE)
    unknown = rec[(rec["axis"] != AXIS_X) & (rec["axis"] != AXIS_Y)]
    if unknown.size:
        raise ValueError(f"{path} has {unknown.size} records with unknown axis id")
    return rec


def resolve_input(path: Path) -> Path:
    if path.is_file():
        return path
    if path.is_dir():
        bins = sorted(path.glob("galvo_slope_*.bin")) or sorted(path.glob("*.bin"))
        if not bins:
            raise FileNotFoundError(f"no .bin files in {path}")
        return bins[-1]
    raise FileNotFoundError(path)


def derivative_magnitude(rec: np.ndarray) -> np.ndarray:
    return np.abs(rec["derivative"].astype(np.float64))


def fit_cone(r: np.ndarray, z: np.ndarray) -> tuple[float, float, dict]:
    """Least-squares |f'(r)| = a + b r. Returns a, b, and residual stats."""
    if r.size < 2:
        raise ValueError("need at least two samples to fit a + b r")
    a_mat = np.column_stack((np.ones(r.size), r))
    coef, residuals, rank, _ = np.linalg.lstsq(a_mat, z, rcond=None)
    a, b = float(coef[0]), float(coef[1])
    pred = a + b * r
    err = z - pred
    ss_res = float(np.dot(err, err))
    ss_tot = float(np.sum((z - np.mean(z)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    rmse = float(np.sqrt(ss_res / r.size))
    stats = {
        "n": int(r.size),
        "rank": int(rank),
        "rmse": rmse,
        "r2": r2,
        "ss_res": ss_res,
        "max_abs_err": float(np.max(np.abs(err))),
        "residual": residuals,
    }
    return a, b, stats


def summary_lines(
    path: Path,
    rec: np.ndarray,
    origin: int,
    a: float,
    b: float,
    stats: dict,
) -> list[str]:
    nx = int(np.count_nonzero(rec["axis"] == AXIS_X))
    ny = int(np.count_nonzero(rec["axis"] == AXIS_Y))
    r = np.abs(rec["start"].astype(np.float64) - origin)
    z = derivative_magnitude(rec)
    return [
        f"file          {path}",
        f"samples       {stats['n']}  (X {nx}, Y {ny})",
        f"origin        DAC {origin}",
        f"radius        [{r.min():.0f}, {r.max():.0f}] DAC counts",
        f"|f'| range    [{z.min():.6e}, {z.max():.6e}] rad/count",
        f"cone          |f'(r)| = {a:.6e} + {b:.6e} * r",
        f"  a (origin)  {a:.6e} rad/count",
        f"  b (slope)   {b:.6e} rad/count per DAC-count of radius",
        f"RMSE          {stats['rmse']:.6e} rad/count",
        f"R²            {stats['r2']:.4f}",
        f"max |err|     {stats['max_abs_err']:.6e} rad/count",
    ]


def plot_fit(
    rec: np.ndarray,
    origin: int,
    a: float,
    b: float,
    lines: list[str],
    title: str,
    out: Path | None,
) -> None:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    start = rec["start"].astype(np.float64)
    z = derivative_magnitude(rec)
    off = start - origin
    r = np.abs(off)
    is_x = rec["axis"] == AXIS_X
    is_y = rec["axis"] == AXIS_Y

    fig = plt.figure(figsize=(12.5, 6.8))
    fig.suptitle(title, fontsize=12)

    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    r_max = float(max(r.max() if r.size else 1.0, 1.0))
    rr = np.linspace(0.0, r_max, 40)
    th = np.linspace(0.0, 2.0 * np.pi, 72)
    rr_g, th_g = np.meshgrid(rr, th)
    xs = rr_g * np.cos(th_g)
    ys = rr_g * np.sin(th_g)
    zs = a + b * rr_g
    ax3.plot_surface(xs, ys, zs, cmap="bone", alpha=0.35, linewidth=0, antialiased=True)
    ax3.scatter(off[is_x], np.zeros(np.count_nonzero(is_x)), z[is_x],
                c="#3aa0ff", s=36, depthshade=True, label="X")
    ax3.scatter(np.zeros(np.count_nonzero(is_y)), off[is_y], z[is_y],
                c="#ff8c42", s=36, depthshade=True, label="Y")
    ax3.set_xlabel("X − origin (DAC)")
    ax3.set_ylabel("Y − origin (DAC)")
    ax3.set_zlabel("|f'| (rad / count)")
    ax3.set_title("cone  a + b r")
    ax3.legend(loc="upper left")

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.scatter(r[is_x], z[is_x], c="#3aa0ff", s=36, label="X", zorder=3)
    ax2.scatter(r[is_y], z[is_y], c="#ff8c42", s=36, label="Y", zorder=3)
    r_line = np.linspace(0.0, r_max, 200)
    ax2.plot(r_line, a + b * r_line, color="#e8e8ea", lw=1.8, label="fit", zorder=2)
    ax2.set_xlabel("r = |DAC − origin|")
    ax2.set_ylabel("|f'| (rad / count)")
    ax2.set_title("radial slice")
    ax2.legend(loc="best")
    ax2.grid(True, alpha=0.25)
    text = "\n".join(lines)
    ax2.text(
        0.02, 0.02, text,
        transform=ax2.transAxes,
        va="bottom", ha="left",
        fontsize=8,
        family="monospace",
        bbox={"boxstyle": "round", "facecolor": "#14161c", "edgecolor": "#444", "alpha": 0.88},
        color="#d8dde6",
    )

    fig.tight_layout()
    if out is not None:
        fig.savefig(out, dpi=140)
        print(f"wrote {out}")
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit a radial cone a + b r to |f'| from galvo slope-map .bin data"
    )
    parser.add_argument(
        "path",
        type=Path,
        help="galvo_slope_*.bin file, or a directory (newest matching file)",
    )
    parser.add_argument(
        "--origin",
        type=int,
        default=DAC_ORIGIN,
        help=f"DAC origin for radius (default {DAC_ORIGIN})",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="optional path to save the figure (still shown)",
    )
    args = parser.parse_args()

    try:
        path = resolve_input(args.path)
        rec = load_calib(path)
    except (OSError, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1

    r = np.abs(rec["start"].astype(np.float64) - args.origin)
    z = derivative_magnitude(rec)
    try:
        a, b, stats = fit_cone(r, z)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    lines = summary_lines(path, rec, args.origin, a, b, stats)
    print("\n".join(lines))
    plot_fit(rec, args.origin, a, b, lines, path.name, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
