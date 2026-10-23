#!/usr/bin/env python3
"""Import ROBOTIS TurtleBot3 Burger meshes into this repo's Y-up, meters kit.

Upstream: https://github.com/ROBOTIS-GIT/turtlebot3 (Apache-2.0)
STLs are authored in millimeters, ROS Z-up (X forward, Y left).
This script converts them to meters, Chrono Y-up (X forward, Z right),
centers each part, and scales visuals to the existing collision hulls.

Physics sizes are not changed.
"""

from __future__ import annotations

import os
import struct
import urllib.request

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(ROOT, "assets", "parts", "turtlebot3")
CACHE = os.path.join(ROOT, "assets", "parts", "turtlebot3", ".cache")
SRC_BASE = "https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/main"

SOURCES = {
    "burger_base.stl": "turtlebot3_description/meshes/bases/burger_base.stl",
    "left_tire.stl": "turtlebot3_description/meshes/wheels/left_tire.stl",
    "right_tire.stl": "turtlebot3_description/meshes/wheels/right_tire.stl",
    "lds.stl": "turtlebot3_description/meshes/sensors/lds.stl",
}


def load_stl(path: str) -> list[tuple[tuple[float, float, float], tuple[tuple[float, float, float], ...]]]:
    with open(path, "rb") as f:
        header = f.read(80)
        if header.startswith(b"solid") and not header.startswith(b"solid\x00"):
            # ASCII STL is uncommon for these files; fall through to binary if count works.
            f.seek(0)
            text = f.read().decode("latin1")
            return _load_ascii(text)
        (count,) = struct.unpack("<I", f.read(4))
        tris = []
        for _ in range(count):
            data = f.read(50)
            if len(data) < 50:
                break
            nums = struct.unpack("<12fH", data)
            n = (nums[0], nums[1], nums[2])
            v0 = (nums[3], nums[4], nums[5])
            v1 = (nums[6], nums[7], nums[8])
            v2 = (nums[9], nums[10], nums[11])
            tris.append((n, (v0, v1, v2)))
        return tris


def _load_ascii(text: str):
    tris = []
    verts: list[tuple[float, float, float]] = []
    normal = (0.0, 0.0, 1.0)
    for line in text.splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] == "facet" and parts[1] == "normal":
            normal = (float(parts[2]), float(parts[3]), float(parts[4]))
            verts = []
        elif parts[0] == "vertex":
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif parts[0] == "endfacet" and len(verts) == 3:
            tris.append((normal, tuple(verts)))
    return tris


def write_stl(path: str, tris) -> None:
    with open(path, "wb") as f:
        f.write(b"TB3 Y-up meters".ljust(80, b"\x00"))
        f.write(struct.pack("<I", len(tris)))
        for n, (v0, v1, v2) in tris:
            f.write(struct.pack("<12fH", *n, *v0, *v1, *v2, 0))


def _key(v) -> tuple[int, int, int]:
    return (int(round(v[0] * 1e5)), int(round(v[1] * 1e5)), int(round(v[2] * 1e5)))


def write_obj(path: str, tris, name: str) -> None:
    """Wavefront OBJ — ChVisualShapeModelFile's documented format."""
    verts: list[tuple[float, float, float]] = []
    norms: list[tuple[float, float, float]] = []
    vmap: dict[tuple[int, int, int], int] = {}
    nmap: dict[tuple[int, int, int], int] = {}
    faces: list[tuple[tuple[int, int], tuple[int, int], tuple[int, int]]] = []

    def add_v(v):
        k = _key(v)
        i = vmap.get(k)
        if i is None:
            verts.append(v)
            i = len(verts)
            vmap[k] = i
        return i

    def add_n(n):
        k = _key(n)
        i = nmap.get(k)
        if i is None:
            norms.append(n)
            i = len(norms)
            nmap[k] = i
        return i

    for n, (v0, v1, v2) in tris:
        ni = add_n(n)
        faces.append(((add_v(v0), ni), (add_v(v1), ni), (add_v(v2), ni)))

    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write(f"# TurtleBot3 {name} - meters, Y-up (from ROBOTIS Apache-2.0 mesh)\n")
        f.write(f"o {name}\n")
        for x, y, z in verts:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for x, y, z in norms:
            f.write(f"vn {x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in faces:
            f.write(f"f {a[0]} {b[0]} {c[0]}\n")


def xform_vertex(v, scale: float, ros_to_yup: bool):
    x, y, z = v[0] * scale, v[1] * scale, v[2] * scale
    if ros_to_yup:
        # ROS (X fwd, Y left, Z up) -> Chrono (X fwd, Y up, Z right)
        return (x, z, -y)
    return (x, y, z)


def xform_normal(n, ros_to_yup: bool):
    if not ros_to_yup:
        return n
    x, y, z = n
    nx, ny, nz = x, z, -y
    length = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
    return (nx / length, ny / length, nz / length)


def bounds(tris):
    xs, ys, zs = [], [], []
    for _, verts in tris:
        for x, y, z in verts:
            xs.append(x)
            ys.append(y)
            zs.append(z)
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))


def transform_mesh(tris, *, mm_to_m: bool, ros_to_yup: bool, extra_scale: float,
                   center: bool, offset=(0.0, 0.0, 0.0)):
    s = (0.001 if mm_to_m else 1.0) * extra_scale
    out = []
    for n, verts in tris:
        nv = tuple(xform_vertex(v, s, ros_to_yup) for v in verts)
        nn = xform_normal(n, ros_to_yup)
        out.append((nn, nv))
    if center:
        xmin, xmax, ymin, ymax, zmin, zmax = bounds(out)
        cx = 0.5 * (xmin + xmax) + offset[0]
        cy = 0.5 * (ymin + ymax) + offset[1]
        cz = 0.5 * (zmin + zmax) + offset[2]
        centered = []
        for n, verts in out:
            centered.append((n, tuple((v[0] - cx, v[1] - cy, v[2] - cz) for v in verts)))
        out = centered
    return out


def fit_scale(tris, target_dx=None, target_dy=None, target_dz=None, target_radius=None):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds(tris)
    dx, dy, dz = xmax - xmin, ymax - ymin, zmax - zmin
    scales = []
    if target_dx and dx > 1e-9:
        scales.append(target_dx / dx)
    if target_dy and dy > 1e-9:
        scales.append(target_dy / dy)
    if target_dz and dz > 1e-9:
        scales.append(target_dz / dz)
    if target_radius is not None:
        r = 0.5 * max(dx, dz)
        if r > 1e-9:
            scales.append(target_radius / r)
    if not scales:
        return 1.0
    return min(scales) if target_radius is None else scales[-1]


def decimate(tris, max_tris: int):
    """Grid-cluster vertices until the mesh is at most max_tris faces."""
    if len(tris) <= max_tris:
        return tris

    def cluster(cell: float):
        vmap: dict[tuple[int, int, int], tuple[float, float, float]] = {}

        def quant(v):
            k = (round(v[0] / cell), round(v[1] / cell), round(v[2] / cell))
            pt = vmap.get(k)
            if pt is None:
                pt = (k[0] * cell, k[1] * cell, k[2] * cell)
                vmap[k] = pt
            return pt

        out = []
        seen: set[tuple] = set()
        for _, (a, b, c) in tris:
            a, b, c = quant(a), quant(b), quant(c)
            if a == b or b == c or a == c:
                continue
            key = tuple(sorted((a, b, c)))
            if key in seen:
                continue
            seen.add(key)
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            out.append(((nx, ny, nz), (a, b, c)))
        return out

    lo, hi = 0.0004, 0.025
    best = tris
    for _ in range(14):
        mid = (lo + hi) * 0.5
        cand = cluster(mid)
        if len(cand) > max_tris:
            lo = mid
        else:
            best = cand
            hi = mid
    return best


def print_bounds(label: str, tris) -> None:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds(tris)
    print(
        f"  {label:16s}  "
        f"X[{xmin:+.4f},{xmax:+.4f}] ({xmax-xmin:.4f})  "
        f"Y[{ymin:+.4f},{ymax:+.4f}] ({ymax-ymin:.4f})  "
        f"Z[{zmin:+.4f},{zmax:+.4f}] ({zmax-zmin:.4f})"
    )


def ensure_sources() -> None:
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(CACHE, exist_ok=True)
    license_path = os.path.join(OUT, "LICENSE")
    if not os.path.isfile(license_path):
        print("Downloading Apache-2.0 LICENSE")
        urllib.request.urlretrieve(f"{SRC_BASE}/LICENSE", license_path)
    for name, rel in SOURCES.items():
        dest = os.path.join(CACHE, name)
        if os.path.isfile(dest) and os.path.getsize(dest) > 0:
            continue
        url = f"{SRC_BASE}/{rel}"
        print(f"Downloading {rel}")
        urllib.request.urlretrieve(url, dest)


def main() -> None:
    ensure_sources()

    burger = load_stl(os.path.join(CACHE, "burger_base.stl"))
    left = load_stl(os.path.join(CACHE, "left_tire.stl"))
    right = load_stl(os.path.join(CACHE, "right_tire.stl"))
    lds = load_stl(os.path.join(CACHE, "lds.stl"))

    print("Source (mm, ROS Z-up):")
    print_bounds("burger", burger)
    print_bounds("left_tire", left)
    print_bounds("right_tire", right)
    print_bounds("lds", lds)

    # Meters, Y-up, centered, still TurtleBot3 real size.
    burger_m = transform_mesh(burger, mm_to_m=True, ros_to_yup=True, extra_scale=1.0, center=True)
    # Tire STL is Z-up standing with axle along ROS Y. Keep that (axle = Y) for
    # our wheel body, whose cylinder axis is local Y.
    left_m = transform_mesh(left, mm_to_m=True, ros_to_yup=False, extra_scale=1.0, center=True)
    right_m = transform_mesh(right, mm_to_m=True, ros_to_yup=False, extra_scale=1.0, center=True)
    lds_m = transform_mesh(lds, mm_to_m=True, ros_to_yup=True, extra_scale=1.0, center=True)

    print("Meters, Y-up, real TB3 size:")
    print_bounds("burger", burger_m)
    print_bounds("left_tire", left_m)
    print_bounds("right_tire", right_m)
    print_bounds("lds", lds_m)

    # Fit to existing DiffDrive hulls (collision stays 0.5 x 0.15 x 0.3, r=0.1).
    # Uniform scale to chassis length/width so the Burger silhouette fills the
    # box without flattening height; height may extend slightly past the hull.
    chassis_scale = fit_scale(burger_m, target_dx=0.50, target_dz=0.28)
    wheel_scale = fit_scale(left_m, target_radius=0.10)
    lidar_scale = fit_scale(lds_m, target_dx=0.10, target_dz=0.10)

    burger_fit = decimate(
        transform_mesh(burger_m, mm_to_m=False, ros_to_yup=False, extra_scale=chassis_scale, center=True), 10000)
    left_fit = decimate(
        transform_mesh(left_m, mm_to_m=False, ros_to_yup=False, extra_scale=wheel_scale, center=True), 4000)
    right_fit = decimate(
        transform_mesh(right_m, mm_to_m=False, ros_to_yup=False, extra_scale=wheel_scale, center=True), 4000)
    lds_fit = decimate(
        transform_mesh(lds_m, mm_to_m=False, ros_to_yup=False, extra_scale=lidar_scale, center=True), 3000)

    print(f"Fit scales: chassis={chassis_scale:.4f}  wheel={wheel_scale:.4f}  lidar={lidar_scale:.4f}")
    print(
        f"Decimated tris: burger={len(burger_fit)}  "
        f"left={len(left_fit)}  right={len(right_fit)}  lds={len(lds_fit)}"
    )
    print("Fitted to DiffDrive hulls:")
    print_bounds("burger", burger_fit)
    print_bounds("left_tire", left_fit)
    print_bounds("right_tire", right_fit)
    print_bounds("lds", lds_fit)

    write_obj(os.path.join(OUT, "burger_base.obj"), burger_fit, "burger_base")
    write_obj(os.path.join(OUT, "left_tire.obj"), left_fit, "left_tire")
    write_obj(os.path.join(OUT, "right_tire.obj"), right_fit, "right_tire")
    write_obj(os.path.join(OUT, "lds.obj"), lds_fit, "lds")

    notice = os.path.join(OUT, "NOTICE")
    with open(notice, "w", encoding="utf-8") as f:
        f.write(
            "TurtleBot3 Burger visual meshes\n"
            "================================\n\n"
            "Source: ROBOTIS-GIT/turtlebot3 (turtlebot3_description)\n"
            "https://github.com/ROBOTIS-GIT/turtlebot3\n"
            "License: Apache License 2.0 (see LICENSE in this directory)\n\n"
            "Files used:\n"
            "  meshes/bases/burger_base.stl\n"
            "  meshes/wheels/left_tire.stl\n"
            "  meshes/wheels/right_tire.stl\n"
            "  meshes/sensors/lds.stl\n\n"
            "Modifications for RoboticsSimulator:\n"
            "  - scaled millimeters to meters\n"
            "  - converted ROS Z-up chassis/lidar to Chrono Y-up\n"
            "  - centered each mesh on its AABB\n"
            "  - uniformly scaled visuals to the existing DiffDrive hulls\n"
            "    (chassis 0.5 x 0.15 x 0.3, wheel radius 0.1)\n"
            "Collision / mass / mount frames are unchanged and stay in catalog.json.\n"
        )
    print(f"Wrote converted meshes to {OUT}")


if __name__ == "__main__":
    main()
