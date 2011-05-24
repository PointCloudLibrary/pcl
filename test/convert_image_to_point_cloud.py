#/usr/bin/env python

import sys
import Image

# Settings:
# Color of background pixels
backgroundpixel = (255, 255, 255)
# scale the point cloud to fit inside (-1,-1) - (1,1)
scaling = True 

im = Image.open (sys.argv[1])

# resulting point cloud vector
points = []

# minimal/maximal pixel positions that contain non-background
minimum = [d for d in im.size]
maximum = [0, 0]

for x in range (im.size[0]):
  for y in range (im.size[1]):
    px = im.getpixel((x,y))
    if (px != backgroundpixel):
      points.append ([x, y, 0, px[0], px[1], px[2]])
      if maximum[0] < x:
        maximum[0] = x
      if minimum[0] > x:
        minimum[0] = x
      if maximum[1] < y:
        maximum[1] = y
      if minimum[1] > y:
        minimum[1] = y

size_x = maximum[0] - minimum[0]
size_y = maximum[1] - minimum[1]

if (size_x > size_y):
  scale = [1.0, float (size_y) / size_x]
else:
  scale = [float (size_x) / size_y, 1.0]

if scaling:
  #aspect_ratio = 
  for p in points:
    for d in range(2):
      p[d] = scale[d] * ((float (p[d]) - minimum[d]) / (maximum[d] - minimum[d]) * 2.0 - 1.0)

print "VERSION .7"
print "FIELDS x y z rgb "
print "SIZE 4 4 4 4"
print "TYPE F F F F"
print "COUNT 1 1 1 1"
print "WIDTH %i" % len (points)
print "HEIGHT 1"
print "VIEWPOINT 0 0 0 1 0 0 0"
print "POINTS %i" % len (points)
print "DATA ascii"

for p in points:
  print p[0], p[1], p[2], p[3]<<16 | p[4]<<8 | p[5]
