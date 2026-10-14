"""Seabed height and small mesh helpers, shared by the world generators.

Everything here is deterministic, so every build gives the same world and the
objects which are placed on the seabed (rocks, pipeline, wreck) sit on the same
surface as the seabed mesh.
"""

import math

MEAN_DEPTH = -40.0      # m, mean z of the seabed


def _hash_noise(x, y):
    n = math.sin(x * 12.9898 + y * 78.233) * 43758.5453
    return (n - math.floor(n)) - 0.5


def height(x, y):
    """Seabed z at world position (x, y)."""
    z = MEAN_DEPTH
    z += 3.0 * math.sin(x / 37.0) * math.cos(y / 29.0)              # large scale relief
    z += 1.2 * math.sin(x / 13.0 + 0.7) * math.sin(y / 11.0)       # dunes
    z += 0.4 * math.sin((0.8 * x + 0.6 * y) / 3.1)                  # sand waves, one direction
    z += 0.02 * x                                                   # gentle slope, deeper to the west
    z += 0.08 * _hash_noise(x, y)
    return z


def normal(x, y, step=0.5):
    dzdx = (height(x + step, y) - height(x - step, y)) / (2 * step)
    dzdy = (height(x, y + step) - height(x, y - step)) / (2 * step)
    n = math.sqrt(dzdx * dzdx + dzdy * dzdy + 1.0)
    return (-dzdx / n, -dzdy / n, 1.0 / n)


class ObjWriter:
    """Collects vertices, normals, uvs and triangles and writes one OBJ file."""

    def __init__(self):
        self.v = []
        self.vn = []
        self.vt = []
        self.f = []

    def add_vertex(self, p, n, uv=(0.0, 0.0)):
        self.v.append(p)
        self.vn.append(n)
        self.vt.append(uv)
        return len(self.v)        # OBJ indices start from 1

    def add_triangle(self, a, b, c):
        self.f.append((a, b, c))

    def write(self, path, name):
        with open(path, 'w') as out:
            out.write(f'# {name}, generated, do not edit\no {name}\n')
            for x, y, z in self.v:
                out.write(f'v {x:.3f} {y:.3f} {z:.3f}\n')
            for x, y, z in self.vn:
                out.write(f'vn {x:.4f} {y:.4f} {z:.4f}\n')
            for u, v in self.vt:
                out.write(f'vt {u:.4f} {v:.4f}\n')
            for a, b, c in self.f:
                out.write(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n')


def smooth_normals(vertices, triangles):
    """Per vertex normals as the area weighted mean of the face normals."""
    normals = [[0.0, 0.0, 0.0] for _ in vertices]
    for a, b, c in triangles:
        pa, pb, pc = vertices[a], vertices[b], vertices[c]
        u = [pb[i] - pa[i] for i in range(3)]
        w = [pc[i] - pa[i] for i in range(3)]
        n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        for k in (a, b, c):
            for i in range(3):
                normals[k][i] += n[i]
    result = []
    for n in normals:
        length = math.sqrt(sum(c * c for c in n)) or 1.0
        result.append(tuple(c / length for c in n))
    return result
