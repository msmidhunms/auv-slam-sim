#!/usr/bin/env python3
"""Generate the seabed mesh (OBJ with normals and texture coordinates) and a
tileable sand texture (PNG). Height comes from terrain.py."""

import argparse
import math
import os
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain  # noqa: E402


def write_mesh(path, size, step, texture_tile):
    n = int(round(size / step)) + 1
    half = size / 2.0
    coords = [-half + i * step for i in range(n)]
    obj = terrain.ObjWriter()
    for y in coords:
        for x in coords:
            obj.add_vertex((x, y, terrain.height(x, y)), terrain.normal(x, y),
                           (x / texture_tile, y / texture_tile))
    for row in range(n - 1):
        for col in range(n - 1):
            a = row * n + col + 1
            b, c = a + 1, a + n
            d = c + 1
            obj.add_triangle(a, b, d)
            obj.add_triangle(a, d, c)
    obj.write(path, 'seabed')


def write_png(path, width, height, pixel):
    rows = b''
    for y in range(height):
        rows += b'\x00' + b''.join(bytes(pixel(x, y)) for x in range(width))

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))

    with open(path, 'wb') as out:
        out.write(b'\x89PNG\r\n\x1a\n')
        out.write(chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)))
        out.write(chunk(b'IDAT', zlib.compress(rows, 9)))
        out.write(chunk(b'IEND', b''))


def sand_pixel(size):
    """Sand colour with grain noise and soft ripple bands, tileable."""
    def value(x, y):
        grain = math.sin(x * 12.9898 + y * 78.233) * 43758.5453
        grain = grain - math.floor(grain)
        ripple = 0.5 + 0.5 * math.sin(2 * math.pi * (x + 0.3 * y) * 4 / size)
        shade = 0.82 + 0.10 * ripple + 0.16 * (grain - 0.5)
        r, g, b = 196, 178, 138
        return (min(255, int(r * shade)), min(255, int(g * shade)), min(255, int(b * shade)))
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--size', type=float, default=300.0, help='side length in m')
    parser.add_argument('--step', type=float, default=1.5, help='grid spacing in m')
    parser.add_argument('--texture-tile', type=float, default=4.0, help='m per texture repeat')
    args = parser.parse_args()

    os.makedirs(os.path.join(args.out_dir, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(args.out_dir, 'materials', 'textures'), exist_ok=True)
    write_mesh(os.path.join(args.out_dir, 'meshes', 'seabed.obj'),
               args.size, args.step, args.texture_tile)
    write_png(os.path.join(args.out_dir, 'materials', 'textures', 'sand.png'), 128, 128,
              sand_pixel(128))


if __name__ == '__main__':
    main()
