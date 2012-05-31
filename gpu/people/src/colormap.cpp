/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author: Koen Buys, Anatoly Baksheev
 */

#include <pcl/gpu/people/colormap.h>
#include <vector>
#include "internal.h"

using namespace std;
using namespace pcl::gpu::people;

pcl::RGB pcl::gpu::people::getLColor (unsigned char l)
{
  const unsigned char* c = LUT_COLOR_LABEL + 3 * l;
  pcl::RGB p;
  p.r = c[0];
  p.g = c[1];
  p.b = c[2];
  return p;
}

pcl::RGB pcl::gpu::people::getLColor (pcl::Label l)
{
  return getLColor(static_cast<unsigned char> (l.label));
}

void pcl::gpu::people::colorLMap ( int W, int H, const trees::Label* l, unsigned char* c )
{
  int numPix = W * H;
  for(int pi = 0; pi < numPix; ++pi) 
  {
    const unsigned char* color = LUT_COLOR_LABEL + 3 * l[pi];
    c[3*pi+0] = color[0];
    c[3*pi+1] = color[1];
    c[3*pi+2] = color[2];
  }
}

void pcl::gpu::people::colorLMap (const pcl::PointCloud<pcl::Label>& cloud_in, pcl::PointCloud<pcl::RGB>& colormap_out)
{
  colormap_out.resize(cloud_in.size());
  for(size_t i = 0; i < cloud_in.size (); i++)
    colormap_out.points[i] = getLColor(cloud_in.points[i]);

  colormap_out.width = cloud_in.width;
  colormap_out.height = cloud_in.height;
}

void pcl::gpu::people::uploadColorMap(DeviceArray<pcl::RGB>& color_map)
{
  // Copy the list of label colors into the devices
  vector<pcl::RGB> rgba(LUT_COLOR_LABEL_LENGTH);
  for(int i = 0; i < LUT_COLOR_LABEL_LENGTH; ++i)
  {
    // !!!! generate in RGB format, not BGR
    rgba[i].r = LUT_COLOR_LABEL[i*3 + 2];
    rgba[i].g = LUT_COLOR_LABEL[i*3 + 1];
    rgba[i].b = LUT_COLOR_LABEL[i*3 + 0];
    rgba[i].a = 255;
  }
  color_map.upload(rgba);
}

void pcl::gpu::people::colorizeLabels(const DeviceArray<pcl::RGB>& color_map, const DeviceArray2D<unsigned char>& labels, DeviceArray2D<pcl::RGB>& color_labels)
{
  color_labels.create(labels.rows(), labels.cols());

  const DeviceArray<uchar4>& map = (const DeviceArray<uchar4>&)color_map;
  device::Image& img = (device::Image&)color_labels;  
  device::colorLMap(labels, map, img);
}

void pcl::gpu::people::colorizeMixedLabels(const DeviceArray<pcl::RGB>& color_map, const DeviceArray2D<unsigned char>& labels, 
                                           const DeviceArray2D<pcl::RGB>& image, DeviceArray2D<pcl::RGB>& color_labels)
{
  color_labels.create(labels.rows(), labels.cols());

  const DeviceArray<uchar4>& map = (const DeviceArray<uchar4>&)color_map;
  device::Image& img = (device::Image&)color_labels;
  device::Image& rgba = (device::Image&)image;
  device::mixedColorMap(labels, map, rgba, img);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const unsigned char pcl::gpu::people::LUT_COLOR_LABEL[] = 
{
     50, 34, 22,
     24,247,196,
     33,207,189,
    254,194,127,
     88,115,175,
    158, 91, 64,
     14, 90,  2,
    100,156,227,
    243,167, 17,
    145,194,184,
    234,171,147,
    220,112, 93,
     93,132,163,
    122,  4, 85,
     75,168, 46,
     15,  5,108,
    180,125,107,
    157, 77,167,
    214, 89, 73,
     52,183, 58,
     54,155, 75,
    249, 61,187,
    143, 57, 11,
    246,198,  0,
    202,177,251,
    229,115, 80,
    159,185,  1,
    186,213,229,
     82, 47,144,
    140, 69,139,
    189,115,117,
     80, 57,150 
};
        
const int pcl::gpu::people::LUT_COLOR_LABEL_LENGTH = sizeof(LUT_COLOR_LABEL)/(sizeof(LUT_COLOR_LABEL[0]) * 3);
