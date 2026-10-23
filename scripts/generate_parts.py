#!/usr/bin/env python3
"""Generate the first robotics mesh kit (OBJ + BMP textures + catalog.json).

Meshes are authored in meters, Y-up, origin at each part's geometric center
unless noted. Collision hulls stay boxes/cylinders/spheres in catalog.json.
"""

from __future__ import annotations

import json
import math
import os
import struct
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PARTS = os.path.join(ROOT, "assets", "parts")
TEX = os.path.join(ROOT, "assets", "textures")


class Mesh:
    def __init__(self) -> None:
        self.v: list[tuple[float, float, float]] = []
        self.vn: list[tuple[float, float, float]] = []
        self.vt: list[tuple[float, float]] = []
        self.faces: list[tuple[str, list[tuple[int, int, int]]]] = []
        self._group = "default"

    def usemtl(self, name: str) -> None:
        self._group = name

    def add_v(self, x: float, y: float, z: float) -> int:
        self.v.append((x, y, z))
        return len(self.v)

    def add_vn(self, x: float, y: float, z: float) -> int:
        n = math.sqrt(x * x + y * y + z * z) or 1.0
        self.vn.append((x / n, y / n, z / n))
        return len(self.vn)

    def add_vt(self, u: float, v: float) -> int:
        self.vt.append((u, v))
        return len(self.vt)

    def tri(self, a: tuple[int, int, int], b: tuple[int, int, int], c: tuple[int, int, int]) -> None:
        self.faces.append((self._group, [a, b, c]))

    def quad(self, a, b, c, d) -> None:
        self.tri(a, b, c)
        self.tri(a, c, d)


def write_png(path: str, width: int, height: int, pixels: bytes) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + pixels[y * width * 3 : (y + 1) * width * 3] for y in range(height))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def hash_noise(x: int, y: int, seed: int) -> float:
    n = (x * 374761393 + y * 668265263 + seed * 1274126177) & 0x7FFFFFFF
    n = (n ^ (n >> 13)) * 1274126177
    return ((n ^ (n >> 16)) & 0xFFFF) / 65535.0


def make_studio_floor(name: str, size: int = 512) -> str:
    """Light concrete with a 1 m checker so the arena reads in Irrlicht Phong."""
    px = bytearray()
    cells = 8
    cell = size // cells
    for y in range(size):
        for x in range(size):
            n = hash_noise(x, y, 41)
            n2 = hash_noise(x // 6, y // 6, 7)
            cx, cy = x // cell, y // cell
            checker = (cx + cy) & 1
            base = (198, 194, 186) if checker == 0 else (176, 172, 164)
            t = 0.88 + 0.12 * n + 0.06 * n2
            edge = (x % cell < 2) or (y % cell < 2)
            if edge:
                t *= 0.82
            r = max(0, min(255, int(base[0] * t)))
            g = max(0, min(255, int(base[1] * t)))
            b = max(0, min(255, int(base[2] * t)))
            px.extend((r, g, b))
    write_png(os.path.join(TEX, f"{name}.png"), size, size, bytes(px))
    return f"../textures/{name}.png"


def make_texture(name: str, base: tuple[int, int, int], variation: int, size: int = 128) -> str:
    px = bytearray()
    for y in range(size):
        for x in range(size):
            n = hash_noise(x, y, size + variation)
            n2 = hash_noise(x // 4, y // 4, variation)
            t = 0.65 + 0.35 * n + 0.15 * n2
            r = max(0, min(255, int(base[0] * t)))
            g = max(0, min(255, int(base[1] * t)))
            b = max(0, min(255, int(base[2] * t)))
            px.extend((r, g, b))
    rel = f"{name}.png"
    write_png(os.path.join(TEX, rel), size, size, bytes(px))
    return f"../textures/{rel}"


def write_obj(path: str, mesh: Mesh, mtl_name: str, materials: dict[str, tuple[str, tuple[float, float, float]]]) -> None:
    mtl_path = path[:-4] + ".mtl"
    with open(mtl_path, "w", encoding="utf-8") as f:
        for name, (tex, color) in materials.items():
            f.write(f"newmtl {name}\n")
            f.write(f"Kd {color[0]:.3f} {color[1]:.3f} {color[2]:.3f}\n")
            f.write("Ka 0.15 0.15 0.15\nKs 0.25 0.25 0.25\nNs 40\n")
            if tex:
                f.write(f"map_Kd {tex}\n")
            f.write("\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"mtllib {os.path.basename(mtl_path)}\n")
        for x, y, z in mesh.v:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for x, y, z in mesh.vn:
            f.write(f"vn {x:.6f} {y:.6f} {z:.6f}\n")
        for u, v in mesh.vt:
            f.write(f"vt {u:.5f} {v:.5f}\n")
        current = None
        for group, verts in mesh.faces:
            if group != current:
                f.write(f"usemtl {group}\n")
                current = group
            f.write("f " + " ".join(f"{vi}/{ti}/{ni}" for vi, ti, ni in verts) + "\n")


def add_box(mesh: Mesh, cx, cy, cz, sx, sy, sz, u_scale=1.0) -> None:
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    corners = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    faces = [
        ((0, 1, 2, 3), (0, 0, -1)),
        ((5, 4, 7, 6), (0, 0, 1)),
        ((4, 0, 3, 7), (-1, 0, 0)),
        ((1, 5, 6, 2), (1, 0, 0)),
        ((3, 2, 6, 7), (0, 1, 0)),
        ((4, 5, 1, 0), (0, -1, 0)),
    ]
    uvs = [(0, 0), (u_scale, 0), (u_scale, u_scale), (0, u_scale)]
    for idx, normal in faces:
        ni = mesh.add_vn(*normal)
        vis = [mesh.add_v(*corners[i]) for i in idx]
        tis = [mesh.add_vt(*uv) for uv in uvs]
        mesh.quad((vis[0], tis[0], ni), (vis[1], tis[1], ni), (vis[2], tis[2], ni), (vis[3], tis[3], ni))


def add_cyl(mesh: Mesh, cx, cy, cz, radius, height, axis="y", segs=28, cap=True) -> None:
    h = height * 0.5
    ring_a: list[int] = []
    ring_b: list[int] = []
    nrm_a: list[int] = []
    nrm_b: list[int] = []
    uv_a: list[int] = []
    uv_b: list[int] = []
    for i in range(segs):
        a = 2.0 * math.pi * i / segs
        c, s = math.cos(a), math.sin(a)
        if axis == "y":
            p0, p1 = (cx + radius * c, cy - h, cz + radius * s), (cx + radius * c, cy + h, cz + radius * s)
            n = (c, 0.0, s)
        elif axis == "z":
            p0, p1 = (cx + radius * c, cy + radius * s, cz - h), (cx + radius * c, cy + radius * s, cz + h)
            n = (c, s, 0.0)
        else:
            p0, p1 = (cx - h, cy + radius * s, cz + radius * c), (cx + h, cy + radius * s, cz + radius * c)
            n = (0.0, s, c)
        ring_a.append(mesh.add_v(*p0))
        ring_b.append(mesh.add_v(*p1))
        nrm_a.append(mesh.add_vn(*n))
        nrm_b.append(mesh.add_vn(*n))
        u = i / segs
        uv_a.append(mesh.add_vt(u, 0.0))
        uv_b.append(mesh.add_vt(u, 1.0))
    for i in range(segs):
        j = (i + 1) % segs
        mesh.quad(
            (ring_a[i], uv_a[i], nrm_a[i]),
            (ring_a[j], uv_a[j], nrm_a[j]),
            (ring_b[j], uv_b[j], nrm_b[j]),
            (ring_b[i], uv_b[i], nrm_b[i]),
        )
    if cap:
        if axis == "y":
            c0, c1 = (cx, cy - h, cz), (cx, cy + h, cz)
            n0, n1 = (0, -1, 0), (0, 1, 0)
        elif axis == "z":
            c0, c1 = (cx, cy, cz - h), (cx, cy, cz + h)
            n0, n1 = (0, 0, -1), (0, 0, 1)
        else:
            c0, c1 = (cx - h, cy, cz), (cx + h, cy, cz)
            n0, n1 = (-1, 0, 0), (1, 0, 0)
        v0, v1 = mesh.add_v(*c0), mesh.add_v(*c1)
        nn0, nn1 = mesh.add_vn(*n0), mesh.add_vn(*n1)
        t0 = mesh.add_vt(0.5, 0.5)
        for i in range(segs):
            j = (i + 1) % segs
            mesh.tri((v0, t0, nn0), (ring_a[j], uv_a[j], nn0), (ring_a[i], uv_a[i], nn0))
            mesh.tri((v1, t0, nn1), (ring_b[i], uv_b[i], nn1), (ring_b[j], uv_b[j], nn1))


def add_sphere(mesh: Mesh, cx, cy, cz, radius, slices=16, stacks=12) -> None:
    rings: list[list[int]] = []
    nrms: list[list[int]] = []
    uvs: list[list[int]] = []
    for i in range(stacks + 1):
        v = i / stacks
        phi = math.pi * v
        sp, cp = math.sin(phi), math.cos(phi)
        row_v, row_n, row_t = [], [], []
        for j in range(slices + 1):
            u = j / slices
            th = 2.0 * math.pi * u
            x, y, z = math.cos(th) * sp, cp, math.sin(th) * sp
            row_v.append(mesh.add_v(cx + radius * x, cy + radius * y, cz + radius * z))
            row_n.append(mesh.add_vn(x, y, z))
            row_t.append(mesh.add_vt(u, 1.0 - v))
        rings.append(row_v)
        nrms.append(row_n)
        uvs.append(row_t)
    for i in range(stacks):
        for j in range(slices):
            mesh.quad(
                (rings[i][j], uvs[i][j], nrms[i][j]),
                (rings[i][j + 1], uvs[i][j + 1], nrms[i][j + 1]),
                (rings[i + 1][j + 1], uvs[i + 1][j + 1], nrms[i + 1][j + 1]),
                (rings[i + 1][j], uvs[i + 1][j], nrms[i + 1][j]),
            )


def chassis_mesh(sx, sy, sz) -> Mesh:
    m = Mesh()
    m.usemtl("paint")
    add_box(m, 0, 0, 0, sx, sy, sz)
    m.usemtl("deck")
    add_box(m, 0, sy * 0.42, 0, sx * 0.88, sy * 0.18, sz * 0.82)
    m.usemtl("nose")
    add_box(m, sx * 0.42, sy * 0.12, 0, sx * 0.22, sy * 0.45, sz * 0.42)
    m.usemtl("metal")
    add_box(m, -sx * 0.38, -sy * 0.22, 0, sx * 0.18, sy * 0.28, sz * 0.70)
    add_box(m, 0, -sy * 0.38, sz * 0.48, sx * 0.70, sy * 0.12, sz * 0.08)
    add_box(m, 0, -sy * 0.38, -sz * 0.48, sx * 0.70, sy * 0.12, sz * 0.08)
    add_cyl(m, sx * 0.18, sy * 0.62, 0, 0.012, sy * 0.55, "y", 12)
    return m


def wheel_mesh(radius, width) -> Mesh:
    m = Mesh()
    m.usemtl("rubber")
    add_cyl(m, 0, 0, 0, radius, width, "y", 32)
    m.usemtl("rim")
    add_cyl(m, 0, 0, 0, radius * 0.62, width * 0.55, "y", 24)
    m.usemtl("metal")
    add_cyl(m, 0, 0, 0, radius * 0.18, width * 0.85, "y", 16)
    for k in range(5):
        a = 2.0 * math.pi * k / 5.0
        m.usemtl("metal")
        add_box(m, math.cos(a) * radius * 0.36, 0, math.sin(a) * radius * 0.36, radius * 0.10, width * 0.28, radius * 0.42)
    return m


def caster_mesh(radius) -> Mesh:
    m = Mesh()
    m.usemtl("rubber")
    add_sphere(m, 0, 0, 0, radius, 18, 12)
    m.usemtl("metal")
    add_cyl(m, 0, radius * 0.85, 0, radius * 0.22, radius * 0.55, "y", 12)
    add_box(m, 0, radius * 1.15, 0, radius * 1.1, radius * 0.18, radius * 0.35)
    return m


def motor_hub_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("metal")
    add_cyl(m, 0, 0, 0, 0.035, 0.06, "z", 20)
    add_cyl(m, 0, 0, 0, 0.048, 0.012, "z", 20)
    m.usemtl("paint")
    add_box(m, 0, 0, -0.04, 0.055, 0.055, 0.02)
    return m


def lidar_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("metal")
    add_cyl(m, 0, -0.015, 0, 0.04, 0.02, "y", 20)
    m.usemtl("sensor")
    add_cyl(m, 0, 0.02, 0, 0.032, 0.045, "y", 24)
    m.usemtl("lens")
    add_cyl(m, 0, 0.03, 0.028, 0.012, 0.01, "z", 12)
    return m


def camera_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("paint")
    add_box(m, 0, 0, 0, 0.05, 0.04, 0.035)
    m.usemtl("lens")
    add_cyl(m, 0.028, 0, 0, 0.012, 0.018, "x", 16)
    m.usemtl("metal")
    add_box(m, -0.02, -0.025, 0, 0.02, 0.01, 0.03)
    return m


def arm_link_mesh(length, radius) -> Mesh:
    m = Mesh()
    m.usemtl("paint")
    add_cyl(m, 0, 0, 0, radius, length, "y", 20)
    m.usemtl("metal")
    add_cyl(m, 0, -length * 0.42, 0, radius * 1.35, radius * 1.1, "z", 16)
    add_cyl(m, 0, length * 0.42, 0, radius * 1.15, radius * 0.9, "z", 16)
    return m


def gripper_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("metal")
    add_box(m, 0, 0, 0, 0.07, 0.04, 0.05)
    m.usemtl("paint")
    add_box(m, 0.04, 0.03, 0.02, 0.015, 0.08, 0.018)
    add_box(m, 0.04, 0.03, -0.02, 0.015, 0.08, 0.018)
    add_box(m, 0.055, 0.07, 0.02, 0.03, 0.015, 0.016)
    add_box(m, 0.055, 0.07, -0.02, 0.03, 0.015, 0.016)
    return m


def crate_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("wood")
    add_box(m, 0, 0, 0, 1.0, 1.0, 1.0)
    m.usemtl("metal")
    add_box(m, 0, 0.48, 0, 1.02, 0.06, 1.02)
    add_box(m, 0, -0.48, 0, 1.02, 0.06, 1.02)
    return m


def barrel_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("paint")
    add_cyl(m, 0, 0, 0, 0.5, 0.6, "y", 28)
    m.usemtl("metal")
    add_cyl(m, 0, 0.22, 0, 0.52, 0.04, "y", 24)
    add_cyl(m, 0, -0.22, 0, 0.52, 0.04, "y", 24)
    return m


def ground_mesh() -> Mesh:
    m = Mesh()
    m.usemtl("asphalt")
    add_box(m, 0, 0, 0, 40.0, 0.02, 40.0, u_scale=20.0)
    return m


def main() -> None:
    os.makedirs(PARTS, exist_ok=True)
    os.makedirs(TEX, exist_ok=True)

    asphalt = make_texture("asphalt", (48, 50, 54), 11, 256)
    make_studio_floor("studio_floor", 512)
    grass = make_texture("grass", (62, 96, 48), 23, 256)
    aluminum = make_texture("aluminum", (170, 174, 180), 7)
    rubber = make_texture("rubber", (28, 28, 30), 19)
    paint_blue = make_texture("paint_blue", (42, 92, 168), 5)
    paint_yellow = make_texture("paint_yellow", (214, 170, 42), 3)
    paint_red = make_texture("paint_red", (168, 48, 40), 9)
    wood = make_texture("wood", (132, 92, 52), 17)
    lens = make_texture("lens", (30, 80, 110), 2)

    def mats(**kwargs):
        out = {}
        for k, v in kwargs.items():
            out[k] = v
        return out

    kit = [
        ("chassis_small.obj", chassis_mesh(0.35, 0.08, 0.20), mats(
            paint=(paint_blue, (0.22, 0.42, 0.78)),
            deck=(aluminum, (0.55, 0.58, 0.62)),
            nose=(paint_yellow, (0.92, 0.74, 0.18)),
            metal=(aluminum, (0.65, 0.67, 0.70)),
        )),
        ("chassis_medium.obj", chassis_mesh(0.50, 0.15, 0.30), mats(
            paint=(paint_blue, (0.20, 0.45, 0.85)),
            deck=(aluminum, (0.55, 0.58, 0.62)),
            nose=(paint_yellow, (0.95, 0.75, 0.15)),
            metal=(aluminum, (0.65, 0.67, 0.70)),
        )),
        ("chassis_large.obj", chassis_mesh(0.60, 0.12, 0.35), mats(
            paint=(paint_blue, (0.18, 0.38, 0.72)),
            deck=(aluminum, (0.50, 0.52, 0.56)),
            nose=(paint_yellow, (0.90, 0.70, 0.16)),
            metal=(aluminum, (0.62, 0.64, 0.68)),
        )),
        ("chassis_arm.obj", chassis_mesh(0.40, 0.12, 0.40), mats(
            paint=(aluminum, (0.45, 0.48, 0.52)),
            deck=(paint_blue, (0.22, 0.40, 0.70)),
            nose=(paint_yellow, (0.85, 0.70, 0.20)),
            metal=(aluminum, (0.60, 0.62, 0.66)),
        )),
        ("wheel_std.obj", wheel_mesh(0.10, 0.05), mats(
            rubber=(rubber, (0.08, 0.08, 0.08)),
            rim=(aluminum, (0.70, 0.72, 0.75)),
            metal=(aluminum, (0.55, 0.56, 0.58)),
        )),
        ("wheel_caster.obj", caster_mesh(0.05), mats(
            rubber=(rubber, (0.20, 0.20, 0.22)),
            metal=(aluminum, (0.55, 0.55, 0.58)),
        )),
        ("motor_hub.obj", motor_hub_mesh(), mats(
            metal=(aluminum, (0.60, 0.62, 0.64)),
            paint=(paint_red, (0.70, 0.20, 0.16)),
        )),
        ("lidar.obj", lidar_mesh(), mats(
            metal=(aluminum, (0.50, 0.52, 0.54)),
            sensor=(paint_red, (0.15, 0.15, 0.16)),
            lens=(lens, (0.15, 0.45, 0.55)),
        )),
        ("camera.obj", camera_mesh(), mats(
            paint=(rubber, (0.12, 0.12, 0.14)),
            lens=(lens, (0.10, 0.25, 0.40)),
            metal=(aluminum, (0.55, 0.55, 0.58)),
        )),
        ("arm_link_a.obj", arm_link_mesh(0.35, 0.035), mats(
            paint=(paint_yellow, (0.75, 0.70, 0.25)),
            metal=(aluminum, (0.55, 0.56, 0.58)),
        )),
        ("arm_link_b.obj", arm_link_mesh(0.30, 0.030), mats(
            paint=(paint_yellow, (0.72, 0.66, 0.22)),
            metal=(aluminum, (0.55, 0.56, 0.58)),
        )),
        ("gripper.obj", gripper_mesh(), mats(
            metal=(aluminum, (0.60, 0.62, 0.64)),
            paint=(paint_red, (0.55, 0.18, 0.14)),
        )),
        ("obstacle_crate.obj", crate_mesh(), mats(
            wood=(wood, (0.82, 0.32, 0.22)),
            metal=(aluminum, (0.40, 0.40, 0.42)),
        )),
        ("obstacle_barrel.obj", barrel_mesh(), mats(
            paint=(paint_red, (0.72, 0.22, 0.45)),
            metal=(aluminum, (0.50, 0.50, 0.52)),
        )),
        ("ground_plane.obj", ground_mesh(), mats(
            asphalt=(asphalt, (0.28, 0.30, 0.32)),
        )),
    ]

    for name, mesh, materials in kit:
        write_obj(os.path.join(PARTS, name), mesh, name, materials)

    # grass leftover is referenced by catalog ground material option
    _ = grass

    catalog = {
        "version": 1,
        "parts": [
            {
                "id": "chassis_small",
                "name": "Small chassis",
                "category": "chassis",
                "mesh": "parts/chassis_small.obj",
                "collision": {"shape": "box", "size": [0.35, 0.08, 0.20]},
                "mass": 3.0,
                "mounts": [
                    {"id": "wheel_left", "pos": [0.0, -0.02, -0.12], "role": "wheel"},
                    {"id": "wheel_right", "pos": [0.0, -0.02, 0.12], "role": "wheel"},
                    {"id": "caster_rear", "pos": [-0.12, -0.02, 0.0], "role": "caster"},
                    {"id": "deck_front", "pos": [0.12, 0.05, 0.0], "role": "sensor"},
                    {"id": "deck_center", "pos": [0.0, 0.05, 0.0], "role": "arm"},
                ],
            },
            {
                "id": "chassis_medium",
                "name": "TurtleBot3 Burger chassis",
                "category": "chassis",
                "mesh": "parts/turtlebot3/burger_base.obj",
                "visual": {
                    "mesh": "parts/turtlebot3/burger_base.obj",
                    "scale": 1.0,
                    "color": [0.30, 0.31, 0.34],
                    "metallic": 0.05,
                    "roughness": 0.72,
                },
                "collision": {"shape": "box", "size": [0.50, 0.15, 0.30]},
                "mass": 5.0,
                "mounts": [
                    {"id": "wheel_left", "pos": [0.0, 0.0, -0.16], "role": "wheel"},
                    {"id": "wheel_right", "pos": [0.0, 0.0, 0.16], "role": "wheel"},
                    {"id": "caster_rear", "pos": [-0.16, 0.0, 0.0], "role": "caster"},
                    {"id": "deck_front", "pos": [0.18, 0.09, 0.0], "role": "sensor"},
                    {"id": "deck_center", "pos": [0.0, 0.09, 0.0], "role": "arm"},
                    {"id": "hub_left", "pos": [0.0, 0.0, -0.12], "role": "hub"},
                    {"id": "hub_right", "pos": [0.0, 0.0, 0.12], "role": "hub"},
                ],
            },
            {
                "id": "chassis_large",
                "name": "Large chassis",
                "category": "chassis",
                "mesh": "parts/chassis_large.obj",
                "collision": {"shape": "box", "size": [0.60, 0.12, 0.35]},
                "mass": 8.0,
                "mounts": [
                    {"id": "wheel_fl", "pos": [0.20, 0.0, -0.22], "role": "wheel"},
                    {"id": "wheel_fr", "pos": [0.20, 0.0, 0.22], "role": "wheel"},
                    {"id": "wheel_rl", "pos": [-0.20, 0.0, -0.22], "role": "wheel"},
                    {"id": "wheel_rr", "pos": [-0.20, 0.0, 0.22], "role": "wheel"},
                    {"id": "deck_front", "pos": [0.22, 0.08, 0.0], "role": "sensor"},
                    {"id": "deck_center", "pos": [0.0, 0.08, 0.0], "role": "arm"},
                ],
            },
            {
                "id": "wheel_std",
                "name": "TurtleBot3 tire",
                "category": "wheel",
                "mesh": "parts/turtlebot3/left_tire.obj",
                "visual": {
                    "mesh": "parts/turtlebot3/left_tire.obj",
                    "scale": 1.0,
                    "color": [0.07, 0.07, 0.08],
                    "metallic": 0.0,
                    "roughness": 0.92,
                },
                "collision": {"shape": "cylinder", "radius": 0.10, "height": 0.05},
                "mass": 0.4,
                "mounts": [],
            },
            {
                "id": "wheel_caster",
                "name": "Caster wheel",
                "category": "caster",
                "mesh": "parts/wheel_caster.obj",
                "collision": {"shape": "sphere", "radius": 0.05},
                "mass": 0.15,
                "mounts": [],
            },
            {
                "id": "motor_hub",
                "name": "Motor hub",
                "category": "hub",
                "mesh": "parts/motor_hub.obj",
                "collision": {"shape": "cylinder", "radius": 0.035, "height": 0.06},
                "mass": 0.12,
                "visual_only": True,
                "mounts": [],
            },
            {
                "id": "lidar",
                "name": "TurtleBot3 LDS",
                "category": "sensor",
                "mesh": "parts/turtlebot3/lds.obj",
                "visual": {
                    "mesh": "parts/turtlebot3/lds.obj",
                    "scale": 1.0,
                    "color": [0.14, 0.15, 0.17],
                    "metallic": 0.15,
                    "roughness": 0.55,
                },
                "collision": {"shape": "cylinder", "radius": 0.04, "height": 0.07},
                "mass": 0.08,
                "visual_only": True,
                "mounts": [],
            },
            {
                "id": "camera",
                "name": "Camera",
                "category": "sensor",
                "mesh": "parts/camera.obj",
                "collision": {"shape": "box", "size": [0.05, 0.04, 0.035]},
                "mass": 0.05,
                "visual_only": True,
                "mounts": [],
            },
            {
                "id": "arm_link_a",
                "name": "Arm link A",
                "category": "arm",
                "mesh": "parts/arm_link_a.obj",
                "collision": {"shape": "box", "size": [0.08, 0.35, 0.08]},
                "mass": 1.2,
                "mounts": [
                    {"id": "end", "pos": [0.0, 0.32, 0.0], "role": "arm"},
                ],
            },
            {
                "id": "arm_link_b",
                "name": "Arm link B",
                "category": "arm",
                "mesh": "parts/arm_link_b.obj",
                "collision": {"shape": "box", "size": [0.07, 0.30, 0.07]},
                "mass": 0.8,
                "mounts": [
                    {"id": "end", "pos": [0.0, 0.20, 0.0], "role": "arm"},
                ],
            },
            {
                "id": "gripper",
                "name": "Gripper",
                "category": "arm",
                "mesh": "parts/gripper.obj",
                "collision": {"shape": "sphere", "radius": 0.05},
                "mass": 0.3,
                "joint": "fixed",
                "mounts": [],
            },
            {
                "id": "obstacle_crate",
                "name": "Crate",
                "category": "obstacle",
                "mesh": "parts/obstacle_crate.obj",
                "collision": {"shape": "box", "size": [1.0, 1.0, 1.0]},
                "mass": 0.0,
                "mounts": [],
            },
            {
                "id": "obstacle_barrel",
                "name": "Barrel",
                "category": "obstacle",
                "mesh": "parts/obstacle_barrel.obj",
                "collision": {"shape": "cylinder", "radius": 0.5, "height": 0.6},
                "mass": 0.0,
                "mounts": [],
            },
            {
                "id": "ground_asphalt",
                "name": "Asphalt ground",
                "category": "ground",
                "mesh": "parts/ground_plane.obj",
                "texture": "textures/asphalt.png",
                "collision": {"shape": "box", "size": [200.0, 0.2, 200.0]},
                "mass": 0.0,
                "mounts": [],
            },
        ],
    }

    with open(os.path.join(PARTS, "catalog.json"), "w", encoding="utf-8") as f:
        json.dump(catalog, f, indent=2)
        f.write("\n")

    print(f"Wrote {len(kit)} meshes and catalog to {PARTS}")


if __name__ == "__main__":
    main()
