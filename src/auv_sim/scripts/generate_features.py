#!/usr/bin/env python3
"""Generate the meshes of the objects on the seabed: rock fields, a pipeline
lying along the seabed and a ship wreck. All of them are placed on the
terrain.py surface, so nothing is floating or fully buried."""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain  # noqa: E402


def _noise3(x, y, z, seed):
    n = math.sin(x * 12.9898 + y * 78.233 + z * 37.719 + seed * 4.1414) * 43758.5453
    return (n - math.floor(n)) - 0.5


def icosphere(subdivisions):
    t = (1.0 + math.sqrt(5.0)) / 2.0
    verts = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
             (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    verts = [tuple(c / math.sqrt(sum(k * k for k in v)) for c in v) for v in verts]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
             (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
             (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    for _ in range(subdivisions):
        cache = {}

        def midpoint(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = [(verts[a][i] + verts[b][i]) / 2 for i in range(3)]
                length = math.sqrt(sum(c * c for c in m))
                verts.append(tuple(c / length for c in m))
                cache[key] = len(verts) - 1
            return cache[key]
        new_faces = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    return verts, faces


def add_mesh(obj, vertices, triangles):
    normals = terrain.smooth_normals(vertices, triangles)
    base = len(obj.v)
    for p, n in zip(vertices, normals):
        obj.add_vertex(p, n)
    for a, b, c in triangles:
        obj.add_triangle(base + a + 1, base + b + 1, base + c + 1)


def rock(obj, cx, cy, radii, yaw, seed):
    sphere, faces = icosphere(2)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    ground = terrain.height(cx, cy)
    vertices = []
    for x, y, z in sphere:
        # lumpy surface, and the bottom is flattened where it sits on the sand
        k = 1.0 + 0.35 * _noise3(2 * x, 2 * y, 2 * z, seed) + 0.15 * _noise3(5 * x, 5 * y, 5 * z, seed + 1)
        px, py, pz = x * radii[0] * k, y * radii[1] * k, max(z, -0.3) * radii[2] * k
        wx = cx + cos_y * px - sin_y * py
        wy = cy + sin_y * px + cos_y * py
        vertices.append((wx, wy, ground + 0.15 * radii[2] + pz))
    add_mesh(obj, vertices, faces)


def rock_fields(obj):
    fields = [((35.0, 30.0), 14, 12.0), ((-40.0, -35.0), 12, 10.0), ((70.0, -50.0), 9, 8.0),
              ((-80.0, 70.0), 8, 9.0)]
    seed = 0
    for (fx, fy), count, spread in fields:
        for i in range(count):
            seed += 1
            r = lambda s: _noise3(seed, i, s, seed) + 0.5    # 0 .. 1, repeatable
            x = fx + spread * (2 * r(1) - 1)
            y = fy + spread * (2 * r(2) - 1)
            size = 0.5 + 2.0 * r(3) ** 2
            radii = (size * (0.8 + 0.5 * r(4)), size * (0.6 + 0.4 * r(5)), size * (0.4 + 0.3 * r(6)))
            rock(obj, x, y, radii, 6.28 * r(7), seed)


def pipeline(obj, start, end, radius, step=2.0, sides=14):
    length = math.dist(start, end)
    count = int(length / step) + 1
    dx, dy = (end[0] - start[0]) / length, (end[1] - start[1]) / length
    side = (-dy, dx)
    vertices, triangles = [], []
    for i in range(count):
        s = i * length / (count - 1)
        x, y = start[0] + dx * s, start[1] + dy * s
        # lying on the seabed, some stretches are partly buried in sand
        burial = 0.35 * radius * (1 + math.sin(s / 23.0))
        cz = terrain.height(x, y) + radius - burial
        for k in range(sides):
            a = 2 * math.pi * k / sides
            off_h, off_v = radius * math.cos(a), radius * math.sin(a)
            vertices.append((x + side[0] * off_h, y + side[1] * off_h, cz + off_v))
    for i in range(count - 1):
        for k in range(sides):
            a = i * sides + k
            b = i * sides + (k + 1) % sides
            c, d = a + sides, b + sides
            triangles += [(a, c, b), (b, c, d)]
    add_mesh(obj, vertices, triangles)


def wreck(obj, cx, cy, heading, roll, length=24.0, beam=6.0, depth=3.2):
    """Simple ship hull: sections from a round bow to a flat stern, with a deck,
    a deck house and a broken mast. Lying on its side partly and sunk in sand."""
    sections, per_side = 26, 9
    vertices, triangles = [], []

    def half_beam(u):                        # u = 0 at stern, 1 at bow
        return beam / 2 * (min(1.0, 1.4 * (1 - u)) ** 0.5 if u > 0.3 else 0.85 + 0.5 * u)

    for i in range(sections):
        u = i / (sections - 1)
        x = (u - 0.5) * length
        hb = max(half_beam(u), 0.05)
        keel = -depth * (1 - 0.25 * u * u)
        for k in range(per_side * 2 + 1):     # port gunwale -> keel -> starboard gunwale
            t = k / (per_side * 2)
            a = math.pi * t
            vertices.append((x, -hb * math.cos(a), keel * math.sin(a)))
    ring = per_side * 2 + 1
    for i in range(sections - 1):
        for k in range(ring - 1):
            a = i * ring + k
            b, c = a + 1, a + ring
            d = c + 1
            triangles += [(a, b, d), (a, d, c)]
    # deck, between the two gunwales
    for i in range(sections - 1):
        a, b = i * ring, i * ring + ring - 1
        c, d = a + ring, b + ring
        triangles += [(a, c, d), (a, d, b)]

    def box(x0, x1, y0, y1, z0, z1):
        base = len(vertices)
        for x in (x0, x1):
            for y in (y0, y1):
                for z in (z0, z1):
                    vertices.append((x, y, z))
        for f in ((0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
                  (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)):
            triangles.append(tuple(base + j for j in f))

    box(-9.0, -4.0, -2.0, 2.0, 0.0, 2.6)        # deck house
    box(-8.0, -6.0, -1.2, 1.2, 2.6, 3.6)        # wheel house
    box(2.0, 2.4, -0.2, 0.2, 0.0, 5.5)          # broken mast

    # roll onto the side, turn to heading and sink into the sand
    cr, sr = math.cos(roll), math.sin(roll)
    ch, sh = math.cos(heading), math.sin(heading)
    sink = depth * 0.55
    placed = []
    for x, y, z in vertices:
        y, z = cr * y - sr * z, sr * y + cr * z
        wx, wy = cx + ch * x - sh * y, cy + sh * x + ch * y
        placed.append((wx, wy, terrain.height(cx, cy) + depth - sink + z))
    add_mesh(obj, placed, triangles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', required=True)
    args = parser.parse_args()
    meshes = os.path.join(args.out_dir, 'meshes')
    os.makedirs(meshes, exist_ok=True)

    rocks = terrain.ObjWriter()
    rock_fields(rocks)
    rocks.write(os.path.join(meshes, 'rocks.obj'), 'rocks')

    pipe = terrain.ObjWriter()
    pipeline(pipe, start=(-130.0, -70.0), end=(130.0, 25.0), radius=0.45)
    pipe.write(os.path.join(meshes, 'pipeline.obj'), 'pipeline')

    ship = terrain.ObjWriter()
    wreck(ship, cx=-55.0, cy=40.0, heading=0.6, roll=0.35)
    ship.write(os.path.join(meshes, 'wreck.obj'), 'wreck')


if __name__ == '__main__':
    main()
