#pragma once


#include "cuda.h"
#include "internal.h"

#include <pcl/point_cloud.h>

inline void getProjectedRadiusSearchBox (int rows, int cols, const pcl::device::Intr& intr, const pcl::PointXYZ& point, float squared_radius, 
                                  int &minX, int &maxX, int &minY, int &maxY)
{  
  int min, max;

  float3 q;
  q.x = intr.fx * point.x + intr.cx * point.z;
  q.y = intr.fy * point.y + intr.cy * point.z;
  q.z = point.z;

  // http://www.wolframalpha.com/input/?i=%7B%7Ba%2C+0%2C+b%7D%2C+%7B0%2C+c%2C+d%7D%2C+%7B0%2C+0%2C+1%7D%7D+*+%7B%7Ba%2C+0%2C+0%7D%2C+%7B0%2C+c%2C+0%7D%2C+%7Bb%2C+d%2C+1%7D%7D

  float coeff8 = 1;                                   //K_KT_.coeff (8);
  float coeff7 = intr.cy;                             //K_KT_.coeff (7);
  float coeff4 = intr.fy * intr.fy + intr.cy*intr.cy; //K_KT_.coeff (4);

  float coeff6 = intr.cx;                             //K_KT_.coeff (6);
  float coeff0 = intr.fx * intr.fx + intr.cx*intr.cx; //K_KT_.coeff (0);

  float a = squared_radius * coeff8 - q.z * q.z;
  float b = squared_radius * coeff7 - q.y * q.z;
  float c = squared_radius * coeff4 - q.y * q.y;
    
  // a and c are multiplied by two already => - 4ac -> - ac
  float det = b * b - a * c;
  
  if (det < 0)
  {
    minY = 0;
    maxY = rows - 1;
  }
  else
  {
    float y1 = (b - sqrt (det)) / a;
    float y2 = (b + sqrt (det)) / a;

    min = (int)std::min(floor(y1), floor(y2));
    max = (int)std::max( ceil(y1),  ceil(y2));
    minY = std::min (rows - 1, std::max (0, min));
    maxY = std::max (std::min (rows - 1, max), 0);
  }

  b = squared_radius * coeff6 - q.x * q.z;
  c = squared_radius * coeff0 - q.x * q.x;

  det = b * b - a * c;
  if (det < 0)
  {
    minX = 0;
    maxX = cols - 1;
  }
  else
  {
    float x1 = (b - sqrt (det)) / a;
    float x2 = (b + sqrt (det)) / a;

    min = (int)std::min (floor(x1), floor(x2));
    max = (int)std::max ( ceil(x1),  ceil(x2));
    minX = std::min (cols- 1, std::max (0, min));
    maxX = std::max (std::min (cols - 1, max), 0);
  }
}