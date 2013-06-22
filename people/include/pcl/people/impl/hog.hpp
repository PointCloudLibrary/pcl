// Adaptation of "gradientMex.cpp" provided with Piotr Dollar's MATLAB toolbox:
/*******************************************************************************
* Piotr's Image&Video Toolbox      Version 3.00
* Copyright 2012 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License
*
* Copyright (c) 2012, Piotr Dollar
* Copyright (c) 2013-, Open Perception, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met: 
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution. 
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the FreeBSD Project.
*
*******************************************************************************/

#include <pcl/people/hog.h>

#ifndef PCL_PEOPLE_HOG_HPP_
#define PCL_PEOPLE_HOG_HPP_

/** \brief Constructor. */
pcl::people::HOG::HOG () {}	

/** \brief Destructor. */
pcl::people::HOG::~HOG () {}

/**
* \brief snap to one of oBin orientations using binary search
**/
int
pcl::people::HOG::compOrient (double dx, double dy, double *ux, double *uy, int oBin)
{
  if(oBin <= 1)
    return 0;
  int o0 = 0, o1 = oBin - 1;
  double s0 = std::fabs(ux[o0] * dx + uy[o0] * dy);
  double s1 = std::fabs(ux[o1] * dx + uy[o1] * dy);
  while(1)
  {
    if(o0 == o1 - 1)
      return (s0 > s1 ? o0 : o1);
    if(s0 < s1)
    {
      o0 += ( o1 - o0 + 1) >> 1;
      s0 = std::fabs(ux[o0] * dx + uy[o0] * dy);
    }
    else
    {
      o1 -= (o1 - o0 + 1) >> 1;
      s1 = std::fabs(ux[o1] * dx + uy[o1] * dy);
    }
  }
  return 0;
}

/**
* \brief compute gradient magnitude (*2) and orientation
**/
void
pcl::people::HOG::compGradImg (double *I, double *G, int *O, int h, int w, int nCh, int oBin)
{
  // compute unit vectors evenly distributed at oBin orientations
  double *ux = new double[oBin];
  double *uy = new double[oBin];

  for(int o = 0; o < oBin; o++)
    ux[o] = std::cos(double(o) / double(oBin) * M_PI);
  for(int o = 0; o < oBin; o++)
    uy[o] = std::sin(double(o) / double(oBin) * M_PI);

  // compute gradients for each channel, pick strongest gradient
  int y, x, c;
  double *I1, v, dx = 0.0, dy = 0.0, dx1, dy1, v1;

#define COMPGRAD(x0, x1, rx, y0, y1, ry)      \
{                                             \
  v=-1;                                       \
  for(c = 0; c < nCh; c++)                    \
  {                                           \
    I1 = I + c * h * w + x * h + y;           \
    dy1 = (*(I1 + y1) - *(I1 - y0)) * ry;     \
    dx1 = (*(I1 + x1*h)-*(I1 - x0 * h)) * rx; \
    v1=dx1 * dx1 + dy1 * dy1;                 \
    if(v1 > v)                                \
    {                                         \
      v = v1;                                 \
      dx = dx1;                               \
      dy = dy1;                               \
    }                                         \
  }                                           \
  *(G + x * h + y) = std::sqrt(v);            \
  *(O + x * h + y) = compOrient(dx, dy, ux, uy, oBin);\
}

  // centered differences on interior points
  for(x = 1; x < w - 1; x++)
    for(y = 1; y < h - 1; y++)
      COMPGRAD(1, 1, 1, 1, 1, 1);

  // uncentered differences along each edge
  x = 0;
  for(y = 1; y < h - 1; y++)
    COMPGRAD(0, 1, 2, 1, 1, 1);
  y = 0;
  for(x = 1; x < w - 1; x++)
    COMPGRAD(1, 1, 1, 0, 1, 2);
  x = w - 1;
  for(y = 1; y < h - 1; y++)
    COMPGRAD(1, 0, 2, 1, 1, 1);
  y = h - 1;
  for(x = 1; x < w - 1; x++)
    COMPGRAD(1, 1, 1, 1, 0, 2);

  // finally uncentered differences at corners
  x = 0;
  y = 0;
  COMPGRAD(0, 1, 2, 0, 1, 2);
  x = w - 1;
  y = 0;
  COMPGRAD(1, 0, 2, 0, 1, 2);
  x = 0;
  y = h - 1;
  COMPGRAD(0, 1, 2, 1, 0, 2);
  x = w - 1;
  y = h - 1;
  COMPGRAD(1, 0, 2, 1, 0, 2);

  delete [] ux;
  delete [] uy;
}

/**
* \brief compute HOG features
*/
void
pcl::people::HOG::compute (double *I, int h, int w, int nCh, int sBin, int oBin, int oGran, double* H)
{
  // compute gradient magnitude (*2) and orientation for each location in I
  double *G = new double[h * w];
  int *O = new int[h * w];
  compGradImg(I, G, O, h, w, nCh, oBin * oGran);

  // compute gradient histograms use trilinear interpolation on spatial and orientation bins
  const int hb = h / sBin, wb = w / sBin, h0 = hb * sBin, w0 = wb * sBin, nb = wb * hb;

  // compute energy in each block by summing over orientations
  double* norm = new double[nb];
  for(int i = 0; i < nb; i++)
    norm[i]=0;

  double *hist = new double[nb*oBin];
  for(int i = 0; i < nb * oBin; i++)
    hist[i]=0;

  if(oGran == 1)
  {
    for(int x = 0; x < w0; x++)
    {
      for(int y = 0; y < h0; y++)
      {
        // bilinear interp.
        double v = *(G + x * h + y);
        int o = *(O + x * h + y);
        double xb = (double(x) + 0.5) / double(sBin) - 0.5;
        int xb0 = (xb < 0) ? -1 : int(xb);
        double yb = (double(y) + 0.5) / double(sBin) - 0.5;
        int yb0 = (yb < 0) ? -1 : int(yb);
        double xd0 = xb - xb0, xd1 = 1.0 - xd0;
        double yd0 = yb - yb0, yd1 = 1.0 - yd0;
        double *dst = hist + o * nb + xb0 * hb + yb0;
        if(xb0 >= 0 && yb0 >= 0)
          *(dst) += xd1 * yd1 * v;
        if(xb0 + 1 < wb && yb0 >= 0)
          *(dst + hb) += xd0 * yd1 * v;
        if(xb0 >= 0 && yb0 + 1 < hb)
          *(dst + 1) += xd1 * yd0 * v;
        if(xb0 + 1 < wb && yb0 + 1 < hb)
          *(dst + hb + 1) += xd0 * yd0 * v;
      }
    }
  }
  else
  {
    for(int x = 0; x < w0; x++)
    {
      for(int y = 0; y < h0; y++)
      { // trilinear interp.
        double v = *(G + x * h + y);
        double o = double(*(O + x * h + y)) / double(oGran);
        int o0 = int(o);
        int o1 = (o0 + 1) % oBin;
        double od0 = o - o0, od1 = 1.0 - od0;
        double xb = (double(x) + 0.5) / double(sBin) - 0.5;
        int xb0 = (xb < 0) ? -1 : int(xb);
        double yb = (double(y) + 0.5) / double(sBin) - 0.5;
        int yb0 = (yb < 0) ? -1 : int(yb);
        double xd0 = xb - xb0, xd1 = 1.0 - xd0;
        double yd0 = yb - yb0, yd1 = 1.0 - yd0;
        double *dst = hist + xb0 * hb + yb0;
        if(xb0 >= 0 && yb0 >= 0)
          *(dst + o0 * nb) += od1 * xd1 * yd1 * v;
        if(xb0 + 1 < wb && yb0 >= 0)
          *(dst + hb + o0 * nb) += od1 * xd0 * yd1 * v;
        if(xb0 >= 0 && yb0 + 1 < hb)
          *(dst + 1 + o0 * nb) += od1 * xd1 * yd0 * v;
        if(xb0 + 1 < wb && yb0 + 1 < hb)
          *(dst + hb + 1 + o0 * nb) += od1 * xd0 * yd0 * v;
        if(xb0 >= 0 && yb0 >= 0)
          *(dst + o1 * nb) += od0 * xd1 * yd1 * v;
        if(xb0 + 1 < wb && yb0 >= 0)
          *(dst + hb + o1 * nb) += od0 * xd0 * yd1 * v;
        if(xb0 >= 0 && yb0 + 1 < hb)
          *(dst + 1 + o1 * nb) += od0 * xd1 * yd0 * v;
        if(xb0 + 1 < wb && yb0 + 1 < hb)
          *(dst + hb + 1 + o1 * nb) += od0 * xd0 * yd0 * v;
      }
    }
  }

  delete [] G;
  delete [] O;

  for(int o = 0; o < oBin; o++)
  {
    double *src = hist + o * nb, *dst = norm, *end = norm + nb;
    while(dst < end)
    {
      *(dst++) += (*src) * (*src);
      src++;
    }
  }

  // compute normalized values (4 different normalizations per block)
  const int out[3] = { std::max(hb - 2, 0), std::max(wb - 2, 0), oBin * 4 };
  const int outp = out[0] * out[1];

  // array instantiated and all elements initialized to 0;
  for(int i = 0; i < out[0] * out[1] * out[2]; i++)
  {
    H[i]=0;
  }

  for(int x = 0; x < out[1]; x++)
  {
    for(int y = 0; y < out[0]; y++)
    {
      int dst = x * out[0] + y;
      double *src, *p, n;
      for(int x1 = 1; x1 >= 0; x1--)
      {
        for(int y1 = 1; y1 >= 0; y1--)
        {
          p = norm + (x + x1) * hb + (y + y1);
          n = 1.0 / std::sqrt(*p + *(p + 1) + *(p + hb) + *(p + hb + 1) + 0.0001);
          src = hist + (x + 1) * hb + (y + 1);
          for(int o = 0; o < oBin; o++)
          {
            H[dst] = std::min(*src * n, 0.2);
            dst += outp;
            src += nb;
          }
        }
      }
    }
  }

  delete []  norm;
  delete  hist;
}
#endif /* PCL_PEOPLE_HOG_HPP_ */
