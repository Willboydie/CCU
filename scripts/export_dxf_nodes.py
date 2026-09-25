"""Write a promo node file from a DXF.

Line and node extraction is `parse_dxf` in the frontend:

    SmartMarkerFrontend/app/src/main/python/dxf_parser.py

That function returns drawing coordinates. This script then does what the
tablet does after the call: scale drawing units to metres (default 0.001,
millimetres) and pair walls at each node. Every up value is written as 0.

The first hand is the incident wall with the smaller bearing, clockwise from
north. The inner angle is the short turn from that hand onto the other wall,
so the path engine's exit arm lies on the wall.

    py -3 scripts/export_dxf_nodes.py plan.dxf -o nodes.txt
    py -3 scripts/export_dxf_nodes.py plan.dxf --southwest -o nodes.txt
    py -3 scripts/export_dxf_nodes.py --self-test
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

CORNER_VERTEX_M = 0.04
COLLINEAR_RAD = 0.08
CLOSED_M = 0.0005

PARSER = (
    Path(__file__).resolve().parents[2]
    / "SmartMarkerFrontend"
    / "app"
    / "src"
    / "main"
    / "python"
    / "dxf_parser.py"
)


def wrap_2pi(angle: float) -> float:
    two_pi = math.tau
    wrapped = math.fmod(angle, two_pi)
    if wrapped < 0.0:
        wrapped += two_pi
    if wrapped >= two_pi:
        wrapped = 0.0
    return wrapped


def wrap_pi(angle: float) -> float:
    wrapped = wrap_2pi(angle)
    if wrapped > math.pi:
        wrapped -= math.tau
    return wrapped


def bearing_cw(east: float, north: float) -> float:
    return wrap_2pi(math.atan2(east, north))


def _unit(east: float, north: float) -> tuple[float, float]:
    length = math.hypot(east, north)
    if length < 1e-6:
        return (0.0, 0.0)
    return (east / length, north / length)


def _add_incident(vertex, start, end, out: list) -> None:
    ve = vertex[0] - start[0]
    vn = vertex[1] - start[1]
    at_start = math.hypot(ve, vn) < CORNER_VERTEX_M
    at_end = math.hypot(vertex[0] - end[0], vertex[1] - end[1]) < CORNER_VERTEX_M
    se = end[0] - start[0]
    sn = end[1] - start[1]
    span = max(math.hypot(se, sn), 0.001)
    if at_start:
        out.append(_unit(se, sn))
        return
    if at_end:
        out.append(_unit(-se, -sn))
        return
    t = (ve * se + vn * sn) / (span * span)
    if t <= 0.0 or t >= 1.0:
        return
    px = start[0] + t * se
    py = start[1] + t * sn
    if math.hypot(vertex[0] - px, vertex[1] - py) < CORNER_VERTEX_M:
        along = _unit(se, sn)
        out.append(along)
        out.append((-along[0], -along[1]))


def _unique_dirs(raw: list) -> list:
    kept = []
    for direction in raw:
        if math.hypot(direction[0], direction[1]) < 1e-6:
            continue
        if any(item[0] * direction[0] + item[1] * direction[1] > 0.98 for item in kept):
            continue
        kept.append(direction)
    return kept


def _acute(a, b) -> float:
    na = math.hypot(a[0], a[1])
    nb = math.hypot(b[0], b[1])
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    cos = (a[0] * b[0] + a[1] * b[1]) / (na * nb)
    cos = max(-1.0, min(1.0, cos))
    theta = math.acos(cos)
    return min(theta, math.pi - theta)


def outlines_from_paths(paths: list) -> list:
    outlines = []
    for path in paths:
        points = [(float(p[0]), float(p[1])) for p in path]
        if len(points) < 2:
            continue
        closed = (
            len(points) >= 3
            and abs(points[0][0] - points[-1][0]) < CLOSED_M
            and abs(points[0][1] - points[-1][1]) < CLOSED_M
        )
        if closed:
            points = points[:-1]
        outlines.append((points, closed))
    return outlines


def corner_angles(east: float, north: float, outlines: list) -> tuple[float, float]:
    raw = []
    vertex = (east, north)
    for points, closed in outlines:
        if len(points) < 2:
            continue
        count = len(points) if closed else len(points) - 1
        for index in range(count):
            _add_incident(vertex, points[index], points[(index + 1) % len(points)], raw)
    dirs = _unique_dirs(raw)
    if len(dirs) < 2:
        return 0.0, 0.0
    best_gap = math.inf
    pair = None
    for i, left in enumerate(dirs):
        for right in dirs[i + 1 :]:
            gap = _acute(left, right)
            if gap < COLLINEAR_RAD:
                continue
            if gap < best_gap:
                best_gap = gap
                pair = (left, right)
    if pair is None:
        return 0.0, 0.0
    first_b = bearing_cw(pair[0][0], pair[0][1])
    second_b = bearing_cw(pair[1][0], pair[1][1])
    if second_b < first_b:
        first_b, second_b = second_b, first_b
    return wrap_pi(second_b - first_b), first_b


def wire_id(label: str, index: int) -> int:
    digits = "".join(ch for ch in str(label) if ch.isdigit())
    if not digits:
        return index & 0xFFFF
    return int(digits) & 0xFFFF


def load_parser():
    if not PARSER.is_file():
        raise FileNotFoundError(f"frontend parser not found: {PARSER}")
    spec = importlib.util.spec_from_file_location("frontend_dxf_parser", PARSER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {PARSER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _node_xy(node) -> tuple[str, float, float]:
    if isinstance(node, dict):
        label = str(node.get("id", ""))
        return label, float(node["x"]), float(node["y"])
    if len(node) >= 3 and not isinstance(node[0], (int, float)):
        return str(node[0]), float(node[1]), float(node[2])
    return "", float(node[0]), float(node[1])


def build_nodes(plan_line_points, nodes, scale: float, origin_xy):
    ox, oy = origin_xy

    def metres(x, y):
        return (float(x) - ox) * scale, (float(y) - oy) * scale

    paths = []
    for path in plan_line_points or []:
        scaled = [metres(point[0], point[1]) for point in path]
        if len(scaled) >= 2:
            paths.append(scaled)
    outlines = outlines_from_paths(paths)
    rows = []
    for index, node in enumerate(nodes or []):
        label, x, y = _node_xy(node)
        east, north = metres(x, y)
        inner, orient = corner_angles(east, north, outlines)
        rows.append(
            {
                "id": wire_id(label, index),
                "label": label or f"V{index}",
                "east": east,
                "north": north,
                "inner": inner,
                "orient": orient,
            }
        )
    segments = []
    for path in paths:
        for start, end in zip(path, path[1:]):
            segments.append((start[0], start[1], end[0], end[1]))
    return rows, segments


def write_file(path: Path, rows, segments, scale: float, origin_xy) -> None:
    lines = [
        "# promo nodes",
        "# parser: SmartMarkerFrontend/app/src/main/python/dxf_parser.py",
        f"# scale_m_per_du: {scale:.9g}",
        f"# origin_du: {origin_xy[0]:.9g} {origin_xy[1]:.9g}",
        "# up_m is 0",
        "# id east_m north_m up_m corner_inner_angle_rad corner_orientation_rad",
    ]
    for row in rows:
        lines.append(
            f"{row['id']} {row['east']:.6f} {row['north']:.6f} 0 "
            f"{row['inner']:.8f} {row['orient']:.8f}  # {row['label']}"
        )
    lines.append("# lines")
    for east1, north1, east2, north2 in segments:
        lines.append(f"{east1:.6f} {north1:.6f} {east2:.6f} {north2:.6f}")
    text = "\n".join(lines) + "\n"
    if str(path) == "-":
        sys.stdout.write(text)
    else:
        path.write_text(text, encoding="utf-8")


def export_dxf(dxf: Path, scale: float, origin_xy, southwest: bool):
    parser = load_parser()
    try:
        plan_line_points, nodes = parser.parse_dxf(str(dxf))
    except UnboundLocalError as exc:
        raise SystemExit("frontend parser found no modelspace entities") from exc
    if southwest:
        if not nodes:
            raise SystemExit("no nodes, so there is no southwest corner")
        xs, ys = [], []
        for node in nodes:
            _, x, y = _node_xy(node)
            xs.append(x)
            ys.append(y)
        origin_xy = (min(xs), min(ys))
    rows, segments = build_nodes(plan_line_points, nodes, scale, origin_xy)
    return rows, segments, origin_xy


def self_test() -> None:
    square = [
        [[0.0, 0.0], [1000.0, 0.0]],
        [[1000.0, 0.0], [1000.0, 1000.0]],
        [[1000.0, 1000.0], [0.0, 1000.0]],
        [[0.0, 1000.0], [0.0, 0.0]],
    ]
    parsed_nodes = [
        {"id": "V0", "x": 0.0, "y": 0.0},
        {"id": "V1", "x": 1000.0, "y": 0.0},
        {"id": "V2", "x": 1000.0, "y": 1000.0},
        {"id": "V3", "x": 0.0, "y": 1000.0},
    ]
    rows, segments = build_nodes(square, parsed_nodes, 0.001, (0.0, 0.0))
    if len(rows) != 4 or len(segments) != 4:
        raise SystemExit(f"self-test: expected 4 nodes and 4 lines, got {len(rows)} {len(segments)}")
    origin = min(rows, key=lambda row: math.hypot(row["east"], row["north"]))
    if abs(origin["east"]) > 1e-6 or abs(origin["north"]) > 1e-6:
        raise SystemExit("self-test: origin node is not at (0, 0)")
    if abs(origin["orient"]) > 1e-6:
        raise SystemExit(f"self-test: origin orientation {origin['orient']}")
    if abs(abs(origin["inner"]) - math.pi / 2) > 1e-6:
        raise SystemExit(f"self-test: origin inner {origin['inner']}")
    east_node = min(rows, key=lambda row: math.hypot(row["east"] - 1.0, row["north"]))
    exit_b = wrap_2pi(east_node["orient"] + east_node["inner"])
    # Walls at this corner run west and north. The exit hand is one of those.
    ok_exit = min(abs(wrap_pi(exit_b - 0.0)), abs(wrap_pi(exit_b - 3 * math.pi / 2))) < 1e-5
    ok_start = min(
        abs(wrap_pi(east_node["orient"] - 0.0)),
        abs(wrap_pi(east_node["orient"] - 3 * math.pi / 2)),
    ) < 1e-5
    if not (ok_exit and ok_start):
        raise SystemExit(
            f"self-test: east corner arms {east_node['orient']} {east_node['inner']}"
        )
    print("self-test PASS")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dxf", nargs="?", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument(
        "--scale",
        type=float,
        default=0.001,
        help="metres per drawing unit (default 0.001, the tablet's millimetre scale)",
    )
    parser.add_argument(
        "--origin",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="drawing-unit point placed at plan (0, 0)",
    )
    parser.add_argument(
        "--southwest",
        action="store_true",
        help="place the minimum east and north of the nodes at plan (0, 0)",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        self_test()
        if args.dxf is None:
            return 0
    if args.dxf is None:
        parser.error("a DXF path is required")
    if not args.dxf.is_file():
        print(f"cannot read {args.dxf}", file=sys.stderr)
        return 1
    if not (args.scale > 0.0) or not math.isfinite(args.scale):
        print("scale must be a positive finite number", file=sys.stderr)
        return 1
    origin = (0.0, 0.0) if args.origin is None else (args.origin[0], args.origin[1])
    rows, segments, origin = export_dxf(args.dxf, args.scale, origin, args.southwest)
    if not rows:
        print("parser returned no nodes", file=sys.stderr)
        return 1
    output = args.output or args.dxf.with_suffix(".nodes.txt")
    write_file(output, rows, segments, args.scale, origin)
    corners = sum(1 for row in rows if abs(row["inner"]) > 1e-6)
    print(
        f"wrote {len(rows)} nodes ({corners} with a corner), {len(segments)} lines -> {output}",
        file=sys.stderr if str(output) == "-" else sys.stdout,
    )
    print(
        f"origin drawing units ({origin[0]:.6g}, {origin[1]:.6g}), scale {args.scale:g} m/unit",
        file=sys.stderr if str(output) == "-" else sys.stdout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
