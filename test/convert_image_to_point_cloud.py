#!/usr/bin/env python3

import sys

from PIL import Image

# Color of background pixels
backgroundpixel = (255, 255, 255)
# Scale the point cloud to fit inside (-1,-1) - (1,1)
scaling = True

with Image.open(sys.argv[1]) as source:
    image = source.convert("RGB")

points = []
for x in range(image.width):
    for y in range(image.height):
        color = image.getpixel((x, y))
        if color != backgroundpixel:
            points.append((x, y, color))

if scaling and points:
    xmin = min(p[0] for p in points)
    xmax = max(p[0] for p in points)
    ymin = min(p[1] for p in points)
    ymax = max(p[1] for p in points)
    span = max(xmax - xmin, ymax - ymin, 1)

print("VERSION .7")
print("FIELDS x y z rgb")
print("SIZE 4 4 4 4")
print("TYPE F F F U")
print("COUNT 1 1 1 1")
print("WIDTH", len(points))
print("HEIGHT 1")
print("VIEWPOINT 0 0 0 1 0 0 0")
print("POINTS", len(points))
print("DATA ascii")

for x, y, (r, g, b) in points:
    if scaling:
        x = (2 * x - xmin - xmax) / span
        y = (2 * y - ymin - ymax) / span
    print(x, y, 0, r << 16 | g << 8 | b)
