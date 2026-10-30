#!/usr/bin/env python3
"""Release only geometrically verified low-profile tunnel cells in the PGM.

The YAML is the source of tunnel bounds and the robot's lowered height. Each
candidate cell must lie in the configured tunnel/straight approach corridor,
have mesh floor support, and not intersect any triangle in the robot height
band. An actual ceiling at the configured height must exist for each tunnel.
This keeps columns/side walls occupied, including cells that straddle a wall.
The mesh and Gazebo collisions are never modified. Re-running is idempotent.

Usage: python3 tools/prepare_rmul2027_tunnels.py [--check]
Dependencies: numpy, Pillow, PyYAML (no ROS/Gazebo runtime required).
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import struct
import sys

import numpy as np
from PIL import Image, ImageDraw
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = (
    ROOT / "icd2027sentry_nav_bringup/config/simulation/rmul_2027_tunnels.yaml"
)
DEFAULT_MAP = ROOT / "icd2027sentry_nav_bringup/map/simulation/rmul_2027.yaml"
DEFAULT_MESH = (
    ROOT
    / "external/icd2027sentry_gazebo_simulator/icd2027sentry_gazebo_simulator/resource/models/rmul_2027/meshes/rmul_2027.stl"
)
DEFAULT_ARTIFACTS = ROOT / "tools/artifacts/rmul2027_tunnels"
EPS = 1e-7


def load_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {path}")
    dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")]
    )
    return np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].astype(
        float
    )


def clip_axis(
    polygon: np.ndarray, axis: int, bound: float, keep_above: bool
) -> np.ndarray:
    """Clip a convex polygon, preserving degenerate vertical-wall projections."""
    if len(polygon) == 0:
        return polygon
    result = []
    previous = polygon[-1]
    previous_inside = (
        (previous[axis] >= bound - EPS)
        if keep_above
        else (previous[axis] <= bound + EPS)
    )
    for current in polygon:
        inside = (
            (current[axis] >= bound - EPS)
            if keep_above
            else (current[axis] <= bound + EPS)
        )
        if inside != previous_inside:
            distance = current[axis] - previous[axis]
            if abs(distance) > EPS:
                result.append(
                    previous
                    + (current - previous) * ((bound - previous[axis]) / distance)
                )
        if inside:
            result.append(current)
        previous, previous_inside = current, inside
    return np.asarray(result, dtype=float).reshape((-1, polygon.shape[1]))


def clip_height(triangle: np.ndarray, low: float, high: float) -> np.ndarray:
    return clip_axis(clip_axis(triangle, 2, low, True), 2, high, False)


def intersects_cell(
    polygon: np.ndarray, x0: float, y0: float, resolution: float
) -> bool:
    for axis, bound, above in (
        (0, x0, True),
        (0, x0 + resolution, False),
        (1, y0, True),
        (1, y0 + resolution, False),
    ):
        polygon = clip_axis(polygon, axis, bound, above)
        if len(polygon) == 0:
            return False
    return True


def point_on_projected_triangle(x: float, y: float, triangle: np.ndarray) -> bool:
    a, b, c = triangle[:, :2]
    cross = np.cross(b - a, c - a)
    if abs(cross) < EPS:
        return False
    point = np.array([x, y])
    u = np.cross(point - a, c - a) / cross
    v = np.cross(b - a, point - a) / cross
    return u >= -EPS and v >= -EPS and u + v <= 1 + EPS


def supported_floor(
    triangles: np.ndarray, x0: float, y0: float, resolution: float
) -> bool:
    # Nine samples avoid accepting a cell with only a tiny floor sliver.
    a, b, c = triangles[:, 0, :2], triangles[:, 1, :2], triangles[:, 2, :2]
    ba, ca = b - a, c - a
    cross = ba[:, 0] * ca[:, 1] - ba[:, 1] * ca[:, 0]
    valid = np.abs(cross) > EPS
    a, ba, ca, cross = a[valid], ba[valid], ca[valid], cross[valid]
    for sx in (0.05, 0.5, 0.95):
        for sy in (0.05, 0.5, 0.95):
            pa = np.array([x0 + sx * resolution, y0 + sy * resolution]) - a
            u = (pa[:, 0] * ca[:, 1] - pa[:, 1] * ca[:, 0]) / cross
            v = (ba[:, 0] * pa[:, 1] - ba[:, 1] * pa[:, 0]) / cross
            if not np.any((u >= -EPS) & (v >= -EPS) & (u + v <= 1 + EPS)):
                return False
    return True


def classify_tunnel_cells(
    mesh: np.ndarray, tunnel: dict, config: dict, map_info: dict, shape: tuple[int, int]
) -> list[dict]:
    resolution = float(map_info["resolution"])
    ox, oy, yaw = map_info["origin"]
    if abs(yaw) > EPS:
        raise ValueError("Rotated occupancy maps are not supported")
    height, width = shape
    approach = config.get("wait_offset", 0.0)
    x_min, x_max = tunnel["x_min"] - approach, tunnel["x_max"] + approach
    y_min, y_max = tunnel["y_min"], tunnel["y_max"]
    floor, ceiling = tunnel["floor_z"], tunnel["ceiling_z"]
    robot_band_top = floor + config["lowered_height"] + config["height_margin"]
    if robot_band_top >= ceiling - EPS:
        raise ValueError(f"{tunnel['id']}: lowered robot has insufficient headroom")
    # Exclude floor/cosmetic field lines, not low vertical obstacles. A 1 mm
    # floor tolerance is far below the robot's physical ground clearance.
    robot_band_low = floor + 0.001
    local = mesh[
        (mesh[:, :, 0].max(1) >= x_min - resolution)
        & (mesh[:, :, 0].min(1) <= x_max + resolution)
        & (mesh[:, :, 1].max(1) >= y_min - resolution)
        & (mesh[:, :, 1].min(1) <= y_max + resolution)
    ]
    ceiling_triangles = local[np.max(np.abs(local[:, :, 2] - ceiling), axis=1) < 1e-5]
    floor_triangles = local[
        np.max(np.abs(local[:, :, 2] - floor), axis=1) <= 0.001 + EPS
    ]
    if len(ceiling_triangles) == 0 or len(floor_triangles) == 0:
        raise ValueError(f"{tunnel['id']}: configured floor/ceiling not found in STL")
    blockers = []
    for triangle in local:
        polygon = clip_height(triangle, robot_band_low, robot_band_top)
        if len(polygon):
            blockers.append(polygon[:, :2])
    blocker_min = np.asarray([p.min(0) for p in blockers]).reshape((-1, 2))
    blocker_max = np.asarray([p.max(0) for p in blockers]).reshape((-1, 2))
    rows = []
    for my in range(
        max(0, int(np.floor((y_min - oy) / resolution))),
        min(height, int(np.floor((y_max - oy) / resolution)) + 1),
    ):
        for mx in range(
            max(0, int(np.floor((x_min - ox) / resolution))),
            min(width, int(np.floor((x_max - ox) / resolution)) + 1),
        ):
            x0, y0 = ox + mx * resolution, oy + my * resolution
            ceiling_overlap = any(
                intersects_cell(t[:, :2], x0, y0, resolution) for t in ceiling_triangles
            )
            near = np.flatnonzero(
                (blocker_min[:, 0] <= x0 + resolution + EPS)
                & (blocker_max[:, 0] >= x0 - EPS)
                & (blocker_min[:, 1] <= y0 + resolution + EPS)
                & (blocker_max[:, 1] >= y0 - EPS)
            )
            blocked = any(
                intersects_cell(blockers[i], x0, y0, resolution) for i in near
            )
            floor_supported = supported_floor(floor_triangles, x0, y0, resolution)
            free = floor_supported and not blocked
            reason = (
                ("clear_below_ceiling" if ceiling_overlap else "clear_approach")
                if free
                else (
                    "low_mesh_or_side_wall"
                    if blocked
                    else "no_floor"
                    if not floor_supported
                    else "no_ceiling"
                )
            )
            rows.append(
                {
                    "tunnel": tunnel["id"],
                    "map_x": mx,
                    "map_y": my,
                    "image_row": height - 1 - my,
                    "x": round(x0 + resolution / 2, 6),
                    "y": round(y0 + resolution / 2, 6),
                    "free": free,
                    "reason": reason,
                }
            )
    return rows


def write_artifacts(
    directory: Path,
    before: np.ndarray,
    after: np.ndarray,
    rows: list[dict],
    map_info: dict,
    config: dict,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    # Preserve the first before image, so repeated runs retain the reviewable
    # original projection while after/diff always describe the current output.
    before_path = directory / "before.png"
    if not before_path.exists():
        Image.fromarray(before).save(before_path)
    original = np.asarray(Image.open(before_path).convert("L"))
    if original.shape != after.shape:
        raise ValueError("Existing before artifact has incompatible map dimensions")
    Image.fromarray(after).save(directory / "after.png")
    diff = np.repeat(after[:, :, None], 3, axis=2)
    diff[original != after] = [0, 210, 80]
    nearest = getattr(Image, "Resampling", Image).NEAREST
    Image.fromarray(diff).resize(
        (after.shape[1] * 4, after.shape[0] * 4), nearest
    ).save(directory / "diff.png")
    # Labelled close-ups show the ceiling boundary, traversable centreline,
    # and both waiting points against the actual rasterized side walls.
    panels = Image.new("RGB", (760, 400), "white")
    for number, tunnel in enumerate(config["tunnels"]):
        ox, oy, _ = map_info["origin"]
        resolution = map_info["resolution"]
        cx = (tunnel["x_min"] + tunnel["x_max"]) / 2
        cy = tunnel.get("center_y", (tunnel["y_min"] + tunnel["y_max"]) / 2)
        centre_px = int((cx - ox) / resolution)
        centre_py = after.shape[0] - 1 - int((cy - oy) / resolution)
        crop_box = (centre_px - 30, centre_py - 23, centre_px + 31, centre_py + 24)
        crop = Image.fromarray(diff).crop(crop_box).resize((366, 282), nearest)
        draw = ImageDraw.Draw(crop)

        def pixel(x, y):
            return (
                ((x - ox) / resolution - crop_box[0]) * 6,
                (after.shape[0] - (y - oy) / resolution - crop_box[1]) * 6,
            )

        draw.rectangle(
            (
                *pixel(tunnel["x_min"], tunnel["y_max"]),
                *pixel(tunnel["x_max"], tunnel["y_min"]),
            ),
            outline=(255, 140, 0),
            width=2,
        )
        for direction in (-1, 1):
            end = cx + direction * (
                0.5 * (tunnel["x_max"] - tunnel["x_min"]) + config["wait_offset"]
            )
            draw.line((pixel(cx, cy), pixel(end, cy)), fill=(20, 90, 255), width=2)
            px, py = pixel(end, cy)
            draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill=(20, 90, 255))
        panels.paste(crop, (10 + number * 380, 40))
        label = ImageDraw.Draw(panels)
        label.text(
            (10 + number * 380, 15),
            tunnel["id"] + f" / centre y={cy:.2f} m",
            fill="black",
        )
    ImageDraw.Draw(panels).text(
        (10, 345),
        "Green: released cells. Orange: ceiling. Blue: centreline and waiting points.",
        fill="black",
    )
    panels.save(directory / "tunnel_review.png")
    with (directory / "cells.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "map_resolution": map_info["resolution"],
        "map_origin": map_info["origin"],
        "robot_height_with_margin": config["lowered_height"] + config["height_margin"],
        "changed_from_original": int(np.count_nonzero(original != after)),
        "tunnels": {},
    }
    for tunnel in config["tunnels"]:
        tunnel_rows = [row for row in rows if row["tunnel"] == tunnel["id"]]
        report["tunnels"][tunnel["id"]] = {
            "candidate_cells": len(tunnel_rows),
            "verified_free_cells": sum(row["free"] for row in tunnel_rows),
            "preserved_blocked_cells": sum(
                row["reason"] == "low_mesh_or_side_wall" for row in tunnel_rows
            ),
            "net_clearance": round(
                tunnel["ceiling_z"] - tunnel["floor_z"] - config["lowered_height"], 6
            ),
        }
        # Derive conservative raster free bounds at the roof's centre column.
        cx = (tunnel["x_min"] + tunnel["x_max"]) / 2
        centre_column = int((cx - map_info["origin"][0]) / map_info["resolution"])
        centre_cells = [
            r for r in tunnel_rows if r["map_x"] == centre_column and r["free"]
        ]
        free_min = min(r["y"] for r in centre_cells) - map_info["resolution"] / 2
        free_max = max(r["y"] for r in centre_cells) + map_info["resolution"] / 2
        centre_y = tunnel.get("center_y", (tunnel["y_min"] + tunnel["y_max"]) / 2)
        footprint_y = np.asarray(config["footprint"])[:, 1]
        lower_margin = centre_y + footprint_y.min() - free_min
        upper_margin = free_max - centre_y - footprint_y.max()
        if min(lower_margin, upper_margin) < 0.01 - EPS:
            raise ValueError(
                f"{tunnel['id']}: configured centreline has under 1 cm raster side clearance"
            )
        report["tunnels"][tunnel["id"]].update(
            {
                "raster_y_bounds": [round(free_min, 6), round(free_max, 6)],
                "centre_y": centre_y,
                "footprint_side_margins": [
                    round(lower_margin, 6),
                    round(upper_margin, 6),
                ],
            }
        )
    (directory / "geometry_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--mesh", type=Path, default=DEFAULT_MESH)
    parser.add_argument("--artifacts", type=Path, default=DEFAULT_ARTIFACTS)
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify current PGM without modifying any files",
    )
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    map_info = yaml.safe_load(args.map.read_text())
    image_path = args.map.parent / map_info["image"]
    before = np.asarray(Image.open(image_path).convert("L")).copy()
    mesh = load_binary_stl(args.mesh)
    rows = [
        row
        for tunnel in config["tunnels"]
        for row in classify_tunnel_cells(mesh, tunnel, config, map_info, before.shape)
    ]
    after = before.copy()
    for row in rows:
        if row["free"]:
            after[row["image_row"], row["map_x"]] = 254
    if args.check:
        pending = int(np.count_nonzero(before != after))
        print(f"{len(rows)} cells checked; {pending} pending map corrections")
        return 1 if pending else 0
    write_artifacts(args.artifacts, before, after, rows, map_info, config)
    if np.any(before != after):
        Image.fromarray(after).save(image_path)
    print(f"Updated {int(np.count_nonzero(before != after))} cells in {image_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
