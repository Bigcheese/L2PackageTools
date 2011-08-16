#!/usr/bin/python

# This program takes a file that is USize * VSize uint16_t's and converts it
# to a ply mesh. You must set the varibles at the top to match the TerrainInfo
# actor you wish to export. You must also manually extract the G16 data into
# a file for this to read.

import sys
import struct
import array

# ======= Properties from TerrainInfo actor ======
TerrainScale = (128.0, 128.0, 76.0)
Location = (81920.0, 147456.0, 160.6513)
DrawScale = 10.0
USize = 256
VSize = 256
# ================================================

Translation = (Location[0] - ((USize >> 1 )* TerrainScale[0]), Location[1] - ((VSize >> 1 ) * TerrainScale[1]), Location[2] - (128 * TerrainScale[2]))

# Read file as an array of uint16_t's.
f = open(sys.argv[1], "rb")
data = f.read()
height_map = array.array('H')
for i in xrange(0, USize * VSize * 2, 2):
  height_map.append(struct.unpack_from("<H", data, i)[0])

# Calculate terrain vertexes.
points = []
for y in xrange(0, VSize):
  for x in xrange(0, USize):
    points.append(((x * TerrainScale[0]) + Translation[0], (y * TerrainScale[1]) + Translation[1], ((height_map[(y * USize) + x] * TerrainScale[2]) / 256.0) + Translation[2]))

# Generate triangular mesh via vertex indecies.
# NOTE: This currently ignores edge flipping.
triangles = []
for y in xrange(0, VSize - 1):
  for x in xrange(0, USize - 1):
    triangles.append((x + (y * USize), (x + 1) + (y * USize), (x + 1) + ((y + 1) * USize)))
    triangles.append((x + (y * USize), (x + 1) + ((y + 1) * USize), x + ((y + 1) * USize)))

# Print ply file.
print("""ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
element face %d
property list uchar int vertex_indices
end_header""" % (len(points), len(triangles)))

for p in points:
  print('%f %f %f' % p)

for t in triangles:
  print('%d %d %d %d' % (len(t), t[0], t[1], t[2]))
