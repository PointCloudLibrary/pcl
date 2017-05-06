/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Xuedong Wang
 *
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 */

#include "../include/scene_recognition.h"

#define SAMPLINGSTEP 1
#define CV_SURF_EXTENDED 1
#define FEATURE_LOWE_COLOR CV_RGB(255, 0, 255)
#define SURF_KDTREE_BBF_MAX_NN_CHKS 400
#define SURF_NN_SQ_DIST_RATIO_THR 0.6  /* the threshold value of distance ratio */
#define MINPQ_INIT_NALLOCD 512
#define MAXDIM 9
#define CLUSTERINGNUM 2
#define DANGLE 4
#define MINN 17
#define DISMAX 0.1
#define MIN_AMOUNT 4

template <typename T> int
cvSign (T t)
{
  return ((t < 0) ? -1 : 1);
}

static __inline int
parent (int i)
{
  return (i - 1) / 2;
}

/* Returns the array index of element */
static __inline int
right (int i)
{
  return 2 * i + 2;
}

/* Returns the array index of element */
static __inline int
left (int i)
{
  return 2 * i + 1;
}


SceneRecognition::SceneRecognition ()
{
  wrap = 0;
}

SceneRecognition::~SceneRecognition ()
{
}


bool
SceneRecognition::extractGlobalFeature (std::vector< std::vector<pcl::PointXYZ> > &points, GlobalFeature* global_feature)
{
  // Calculate the global area and duty cycle of scene
  if (points.size () > 0)
  {
    int r = points.size ();
    int k_r = 35;       /* Test row */
    double area = 0;    /* Area assessment */
    double dis = 0;

    for (int i = 0; i < (int) points[k_r].size () - 1; i++)
    {
      dis = sqrt ((points[k_r][i].x * points[k_r][i].x) + (points[k_r][i].y * points[k_r][i].y) +
                  (points[k_r][i].z * points[k_r][i].z));
      area += 0.5 * (0.5 / 360) * 2 * PI * dis * dis;
    }
    global_feature->duty_cycle = area;

    // The whole spatial volume assessment
    for (int i = k_r - 1; i < r - 1; i++)
    {
      double w_a = 0;
      for (int j = 0; j < (int) points[i].size () - 1; j++)
      {
        dis = sqrt ((points[i][j].x * points[i][j].x) + (points[i][j].y * points[i][j].y) +
                    (points[i][j].z * points[i][j].z));
        w_a += 0.5 * (0.5 / 360) * 2 * PI * dis * dis;
      }

      if (w_a > area)
      {
        area = w_a;
      }
    }
    global_feature->area = area;
    return true;
  }
  else
  {
    QMessageBox::critical (NULL, "Critical", "Extract global spatial features failed!\n"
                                 "NO available data, please load data at first!",
                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    return false;
  }
}

void
SceneRecognition::cvSURFInitialize ()
{
  wrap = cvCreateMat (2, 3, CV_32FC1);
  int k = 0;
  for (int i = 0; i < SCANOCTAVE; i++)
  {
    for (int j = 0; j < FILTERSCALE; j++)
    {
      double scal = ((j * 6 + 9) << i) * 1.2 / 9.;
      regions_cache[k] = cvCreateImage (cvSize (cvRound (21 * scal), cvRound (21 * scal)), 8, 1);
      k++;
    }
  }

  region_cache = cvCreateImage (cvSize (21, 21), 8, 1);
  dx_cache = cvCreateMat (20, 20, CV_64FC1);
  dy_cache = cvCreateMat (20, 20, CV_64FC1);
  gauss_kernel_cache = cvCreateMat (20, 20, CV_64FC1);
  cvSURFGaussian (gauss_kernel_cache, 3.3);

  for (int i = 0; i < 3600; i++)
  {
    cos_cache[i] = cos (i * 0.001745329);
    sin_cache[i] = sin (i * 0.001745329);
  }
}

void
SceneRecognition::cvSURFGaussian (CvMat* mat, double s)
{
  int w = mat->cols;
  int h = mat->rows;
  double x, y;
  double c2 = 1. / (s * s * 2);
  double over_exp = 1. / (3.14159 * 2 * s * s);
  for (int i = 0; i < w; i++)
    for (int j = 0; j < h; j++)
    {
      x = i - w / 2.;
      y = j - h / 2.;
      cvmSet (mat, j, i, exp (-(x * x + y * y) * c2) * over_exp);
    }
}

CvSeq*
SceneRecognition::cvSURFDescriptor (const CvArr* _img, CvMemStorage* storage, double quality, int flags)
{
  IplImage* img = (IplImage*) _img;
  CvMat* sum = 0;
  sum = cvCreateMat (img->height + 1, img->width + 1, CV_32SC1);
  cvIntegral (img, sum);
  CvMemStorage* point_storage = cvCreateChildMemStorage (storage);
  CvSeq* points = cvFastHessianDetector (sum, point_storage, quality);

  CvSeq* descriptors = cvCreateSeq (0, sizeof (CvSeq), sizeof (SURFDescriptor), storage);
  int dx_s[] = { 2, 0, 0, 2, 4, 0, -1, 2, 0, 4, 4, 0, 1 };
  int dy_s[] = { 2, 0, 0, 4, 2, 0, -1, 0, 2, 4, 4, 0, 1 };
  int dx_t[] = { 2, 0, 0, 2, 4, 0, -1, 2, 0, 4, 4, 0, 1 };
  int dy_t[] = { 2, 0, 0, 4, 2, 0, -1, 0, 2, 4, 4, 0, 1 };
  double x[81], *iter_x;
  double y[81], *iter_y;
  double angle[81], *iter_angle;
  double sumx, sumy;
  double temp_mod;
  int angle_n;
  for (int k = 0; k < points->total; k++)
  {
    SURFPoint* point = (SURFPoint*) cvGetSeqElem (points, k);
    SURFDescriptor descriptor;
    descriptor.feature_data = point;
    descriptor.x = cvRound (point->x);
    descriptor.y = cvRound (point->y);
    descriptor.laplacian = point->laplacian;
    int size = point->size;
    int layer = point->octave * FILTERSCALE + point->scale;
    descriptor.s = size * 1.2 / 9.;
    descriptor.mod = 0;

    /* Repeatable orientation */
    iter_x = x;
    iter_y = y;
    iter_angle = angle;
    angle_n = 0;
    cvResizeHaarPattern (dx_s, dx_t, 9, size);
    cvResizeHaarPattern (dy_s, dy_t, 9, size);
    int* sum_ptr = (int*) sum->data.ptr;
    double c2 = 1. / (descriptor.s * descriptor.s * 2.5 * 2.5 * 2);
    double over_exp = 1. / (3.14159 * 2 * descriptor.s * descriptor.s * 2.5 * 2.5);
    for (int j = -6; j <= 2; j++)
    {
      int y = descriptor.y + j * size / 9;
      if ((y >= 0) && (y < sum->rows - size))
      {
        double ry = j + 2;
        for (int i = -6; i <= 2; i++)
        {
          int x = descriptor.x + i * size / 9;
          if ((x >= 0) && (x < sum->cols - size))
          {
            double rx = j + 2;
            double radius = rx * rx + ry * ry;
            if (radius <= 16)
            {
              rx *= descriptor.s;
              ry *= descriptor.s;
              *iter_x = cvCalHaarPattern (sum_ptr + x + y * sum->cols, dx_t, sum->cols) * exp (-radius * c2) * over_exp;
              *iter_y = cvCalHaarPattern (sum_ptr + x + y * sum->cols, dy_t, sum->cols) * exp (-radius * c2) * over_exp;
              *iter_angle = cvFastArctan (*iter_y, *iter_x);
              iter_x++;
              iter_y++;
              iter_angle++;
              angle_n++;
            }
          }
        }
      }
    }
    double bestx = 0;
    double besty = 0;
    for (int i = 0; i < 360; i += 5)
    {
      sumx = 0;
      sumy = 0;
      iter_x = x;
      iter_y = y;
      iter_angle = angle;
      for (int j = 0; j < angle_n; j++)
      {
        if (((*iter_angle < i + 60) && (*iter_angle > i)) ||
           (((*iter_angle + 360) < i + 60) && ((*iter_angle + 360) > i)))
        {
          sumx += *iter_x;
          sumy += *iter_y;
        }
        iter_x++;
        iter_y++;
        iter_angle++;
      }
      temp_mod = sumx * sumx + sumy * sumy;
      if (temp_mod > descriptor.mod)
      {
        descriptor.mod = temp_mod;
        bestx = sumx;
        besty = sumy;
      }
    }
    descriptor.dir = cvFastArctan (besty, bestx);

    /* Get sub-region (CV_INTER_AREA approximately retain the information of total image for
       haar feature while reduce the time consuming */
    double cos_dir = cos_cache[MAX (cvRound (descriptor.dir * 10) + 3600, 0) % 3600];
    double sin_dir = sin_cache[MAX (cvRound (descriptor.dir * 10) + 3600, 0) % 3600];
    cvmSet (wrap, 0, 0, cos_dir);
    cvmSet (wrap, 0, 1, -sin_dir);
    cvmSet (wrap, 0, 2, descriptor.x);
    cvmSet (wrap, 1, 0, sin_dir);
    cvmSet (wrap, 1, 1, cos_dir);
    cvmSet (wrap, 1, 2, descriptor.y);

    cvGetQuadrangleSubPix (img, regions_cache[layer], wrap);
    cvResize (regions_cache[layer], region_cache, CV_INTER_AREA);
    uchar* region_d;
    int region_step;
    cvGetImageRawData (region_cache, &region_d, &region_step);
    uchar* region_x = region_d + 1;
    uchar* region_y = region_d + region_step;
    uchar* region_xy = region_d + 1 + region_step;
    region_step -= 20;
    double* iter_dx = (double*) dx_cache->data.ptr;
    double* iter_dy = (double*) dy_cache->data.ptr;
    for (int i = 0; i < 20; i++)
    {
      for (int j = 0; j < 20; j++)
      {
        *iter_dx = *region_y - *region_d - *region_x + *region_xy;
        *iter_dy = *region_x - *region_d - *region_y + *region_xy;
        iter_dx++;
        iter_dy++;
        region_d++;
        region_x++;
        region_y++;
        region_xy++;
      }
      region_d += region_step;
      region_x += region_step;
      region_y += region_step;
      region_xy += region_step;
    }
    cvMul (gauss_kernel_cache, dx_cache, dx_cache);
    cvMul (gauss_kernel_cache, dy_cache, dy_cache);

    double tx, ty;
    double* iter_vector = descriptor.vector;
    if (flags & CV_SURF_EXTENDED)
    {
      /* 128-bin descriptor */
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
          iter_vector[0] = 0;
          iter_vector[1] = 0;
          iter_vector[2] = 0;
          iter_vector[3] = 0;
          iter_vector[4] = 0;
          iter_vector[5] = 0;
          iter_vector[6] = 0;
          iter_vector[7] = 0;
          for (int x = i * 5; x < i * 5 + 5; x++)
          {
            for (int y = j * 5; y < j * 5 + 5; y++)
            {
              tx = cvGetReal2D (dx_cache, x, y);
              ty = cvGetReal2D (dy_cache, x, y);
              if (ty >= 0)
              {
                iter_vector[0] += tx;
                iter_vector[1] += fabs (tx);
              }
              else
              {
                iter_vector[2] += tx;
                iter_vector[3] += fabs (tx);
              }
              if (tx >= 0)
              {
                iter_vector[4] += ty;
                iter_vector[5] += fabs (ty);
              }
              else
              {
                iter_vector[6] += ty;
                iter_vector[7] += fabs (ty);
              }
            }
          }
          /* Unit vector is essential for contrast invariant */
          double normalize = 0;
          for (int k = 0; k < 8; k++)
            normalize += iter_vector[k] * iter_vector[k];
          normalize = sqrt (normalize);
          for (int k = 0; k < 8; k++)
            iter_vector[k] = iter_vector[k] / normalize;
          iter_vector += 8;
        }
    }
    else
    {
      /* 64-bin descriptor */
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
          iter_vector[0] = 0;
          iter_vector[1] = 0;
          iter_vector[2] = 0;
          iter_vector[3] = 0;
          for (int x = i * 5; x < i * 5 + 5; x++)
          {
            for (int y = j * 5; y < j * 5 + 5; y++)
            {
              tx = cvGetReal2D (dx_cache, x, y);
              ty = cvGetReal2D (dy_cache, x, y);
              iter_vector[0] += tx;
              iter_vector[1] += ty;
              iter_vector[2] += fabs (tx);
              iter_vector[3] += fabs (ty);
            }
          }
          double normalize = 0;
          for (int k = 0; k < 4; k++)
            normalize += iter_vector[k] * iter_vector[k];
          normalize = sqrt (normalize);
          for (int k = 0; k < 4; k++)
            iter_vector[k] = iter_vector[k] / normalize;
          iter_vector += 4;
        }
    }

    cvSeqPush (descriptors, &descriptor);
  }
  cvReleaseMemStorage (&point_storage);
  cvReleaseMat (&sum);

  return descriptors;
}

CvSeq*
SceneRecognition::cvFastHessianDetector (const CvMat* sum, CvMemStorage* storage, double quality)
{
  CvSeq* points = cvCreateSeq (0, sizeof (CvSeq), sizeof (SURFPoint), storage);
  CvMat* hessians[SCANOCTAVE * (FILTERSCALE + 2)];
  CvMat* traces[SCANOCTAVE * (FILTERSCALE + 2)];
  int size, size_cache[SCANOCTAVE * (FILTERSCALE + 2)];
  int scale, scale_cache[SCANOCTAVE * (FILTERSCALE + 2)];
  double *hessian_ptr, *hessian_ptr_cache[SCANOCTAVE * (FILTERSCALE + 2)];
  double *trace_ptr, *trace_ptr_cache[SCANOCTAVE * (FILTERSCALE + 2)];
  int dx_s[] = { 3, 0, 2, 3, 7, 0, 1, 3, 2, 6, 7, 0, -2, 6, 2, 9, 7, 0, 1 };
  int dy_s[] = { 3, 2, 0, 7, 3, 0, 1, 2, 3, 7, 6, 0, -2, 2, 6, 7, 9, 0, 1 };
  int dxy_s[] = { 4, 1, 1, 4, 4, 0, 1, 5, 1, 8, 4, 0, -1, 1, 5, 4, 8, 0, -1, 5, 5, 8, 8, 0, 1 };
  int dx_t[] = { 3, 0, 2, 3, 7, 0, 1, 3, 2, 6, 7, 0, -2, 6, 2, 9, 7, 0, 1 };
  int dy_t[] = { 3, 2, 0, 7, 3, 0, 1, 2, 3, 7, 6, 0, -2, 2, 6, 7, 9, 0, 1 };
  int dxy_t[] = { 4, 1, 1, 4, 4, 0, 1, 5, 1, 8, 4, 0, -1, 1, 5, 4, 8, 0, -1, 5, 5, 8, 8, 0, 1 };
  double dx = 0, dy = 0, dxy = 0;
  int k = 0;
  int hessian_rows, hessian_rows_cache[SCANOCTAVE * (FILTERSCALE + 2)];
  int hessian_cols, hessian_cols_cache[SCANOCTAVE * (FILTERSCALE + 2)];

  /* Hessian detector */
  for (int o = 0; o < SCANOCTAVE; o++)
  {
    for (int s = -1; s < FILTERSCALE + 1; s++)
    {
      if (s < 0)
        size_cache[k] = size = 7 << o; // Gaussian scale 1.0
      else
        size_cache[k] = size = (s * 6 + 9) << o; // Gaussian scale size*1.2/9.

      scale_cache[k] = scale = MAX (size, 9) * SAMPLINGSTEP;
      hessian_rows_cache[k] = hessian_rows = (sum->rows) * 9 / scale;
      hessian_cols_cache[k] = hessian_cols = (sum->cols) * 9 / scale;
      hessians[k] = cvCreateMat (hessian_rows, hessian_cols, CV_64FC1);
      traces[k] = cvCreateMat (hessian_rows, hessian_cols, CV_64FC1);
      int* sum_ptr = (int*) sum->data.ptr;
      cvResizeHaarPattern (dx_s, dx_t, 9, size);
      cvResizeHaarPattern (dy_s, dy_t, 9, size);
      cvResizeHaarPattern (dxy_s, dxy_t, 9, size);
      hessian_ptr_cache[k] = hessian_ptr = (double*) hessians[k]->data.ptr;
      trace_ptr_cache[k] = trace_ptr = (double*) traces[k]->data.ptr;
      hessian_ptr += 4 / SAMPLINGSTEP + (4 / SAMPLINGSTEP) * hessian_cols;
      trace_ptr += 4 / SAMPLINGSTEP + (4 / SAMPLINGSTEP) * hessian_cols;
      int oy = 0, y = 0;
      for (int j = 0; j < hessian_rows - 9 / SAMPLINGSTEP; j++)
      {
        int* sum_line_ptr = sum_ptr;
        double* trace_line_ptr = trace_ptr;
        double* hessian_line_ptr = hessian_ptr;
        int ox = 0, x = 0;
        for (int i = 0; i < hessian_cols - 9 / SAMPLINGSTEP; i++)
        {
          dx = cvCalHaarPattern (sum_line_ptr, dx_t, sum->cols);
          dy = cvCalHaarPattern (sum_line_ptr, dy_t, sum->cols);
          dxy = cvCalHaarPattern (sum_line_ptr, dxy_t, sum->cols);
          *hessian_line_ptr = (dx * dy - dxy * dxy * 0.81);
          *trace_line_ptr = dx + dy;
          x = (i + 1) * scale / 9;
          sum_line_ptr += x - ox;
          ox = x;
          trace_line_ptr++;
          hessian_line_ptr++;
        }
        y = (j + 1) * scale / 9;
        sum_ptr += (y - oy) * sum->cols;
        oy = y;
        trace_ptr += hessian_cols;
        hessian_ptr += hessian_cols;
      }
      k++;
    }
  }
  double min_accept = quality * 300;

  k = 0;
  for (int o = 0; o < SCANOCTAVE; o++)
  {
    k++;
    for (int s = 0; s < FILTERSCALE; s++)
    {
      size = size_cache[k];
      scale = scale_cache[k];
      hessian_rows = hessian_rows_cache[k];
      hessian_cols = hessian_cols_cache[k];
      int margin = (5 / SAMPLINGSTEP) * scale_cache[k + 1] / scale;
      hessian_ptr = hessian_ptr_cache[k] + margin + margin * hessian_cols;
      trace_ptr = trace_ptr_cache[k];
      for (int j = margin; j < hessian_rows - margin; j++)
      {
        double* hessian_line_ptr = hessian_ptr;
        for (int i = margin; i < hessian_cols - margin; i++)
        {
          if (*hessian_line_ptr > min_accept)
          {
            bool suppressed = false;
            /* Non-maxima suppression */
            for (int z = k - 1; z < k + 2; z++)
            {
              double* temp_hessian_ptr = hessian_ptr_cache[z] + i * scale / scale_cache[z] - 1 +
                                         (j * scale / scale_cache[z] - 1) * hessian_cols_cache[z];
              for (int y = 0; y < 3; y++)
              {
                double* temp_hessian_line_ptr = temp_hessian_ptr;
                for (int x = 0; x < 3; x++)
                {
                  if (((z != k) || (y != 1) || (x != 1)) && (*temp_hessian_line_ptr > *hessian_line_ptr))
                  {
                    suppressed = true;
                    break;
                  }
                  temp_hessian_line_ptr++;
                }
                if (suppressed)
                break;
                temp_hessian_ptr += hessian_cols_cache[z];
              }
              if (suppressed)
              break;
            }
            if (!suppressed)
            {
              SURFPoint point = cvSURFPoint (i * scale / 9, j * scale / 9, cvSign (trace_ptr[i + j * hessian_cols]),
                                             size_cache[k], o, s);
              cvSeqPush (points, &point);
            }
          }
          hessian_line_ptr++;
        }
        hessian_ptr += hessian_cols;
      }
      k++;
    }
    k++;
  }

  k = 0;
  for (int o = 0; o < SCANOCTAVE; o++)
    for (int s = -1; s < FILTERSCALE + 1; s++)
    {
      cvReleaseMat (&hessians[k]);
      cvReleaseMat (&traces[k]);
      k++;
    }

  return points;
}

void
SceneRecognition::cvResizeHaarPattern (int* t_s, int* t_d, int OldSize, int NewSize)
{
  int n = t_d[0] = t_s[0];
  for (int k = 0; k < n; k++)
  {
    t_d[1] = t_s[1] * NewSize / OldSize;
    t_d[2] = t_s[2] * NewSize / OldSize;
    t_d[3] = t_s[3] * NewSize / OldSize;
    t_d[4] = t_s[4] * NewSize / OldSize;
    t_d[5] = (t_d[3] - t_d[1] + 1) * (t_d[4] - t_d[2] + 1);
    t_d[6] = t_s[6];
    t_d += 6;
    t_s += 6;
  }
}

double
SceneRecognition::cvCalHaarPattern (int* origin, int* t, int widthStep)
{
  double d = 0;
  int *p0 = 0, *p1 = 0, *p2 = 0, *p3 = 0;
  int n = t[0];
  for (int k = 0; k < n; k++)
  {
    p0 = origin + t[1] + t[2] * widthStep;
    p1 = origin + t[1] + t[4] * widthStep;
    p2 = origin + t[3] + t[2] * widthStep;
    p3 = origin + t[3] + t[4] * widthStep;
    d += (double) ((*p3 - *p2 - *p1 + *p0) * t[6]) / (double) (t[5]);
    t += 6;
  }
  return d;
}

SURFPoint
SceneRecognition::cvSURFPoint (int x, int y, int laplacian, int size, int octave, int scale)
{
  SURFPoint p;
  p.x = x;
  p.y = y;
  p.laplacian = laplacian;
  p.size = size;
  p.octave = octave;
  p.scale = scale;
  return p;
}

bool
SceneRecognition::drawSURFFeatures (IplImage* img, CvSeq* feat, int n)
{
  if (n <= 0 || !feat)
  {
    QMessageBox::critical (NULL, "Critical", "No corresponding features!\n" 
                                 "Please replace another scene to continue!",
                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    return false;
  }
  else
  {
    drawSURFLoweFeatures (img, feat, n);
    return true;
  }
}

void
SceneRecognition::drawSURFLoweFeatures (IplImage* img, CvSeq* feat, int n)
{
  CvScalar color = CV_RGB (255, 0, 255);

  if (img->nChannels > 1)
    color = FEATURE_LOWE_COLOR;

  for (int i = 0; i < n; i++)
  {
    SURFDescriptor* feat1 = (SURFDescriptor*) cvGetSeqElem (feat, i);
    drawSURFLoweFeature (img, feat1, color);
  }
}

void
SceneRecognition::drawSURFLoweFeature (IplImage* img, SURFDescriptor* feat, CvScalar color)
{
  int len, hlen, blen, start_x, start_y, end_x, end_y, h1_x, h1_y, h2_x, h2_y;
  double scl, ori;
  double scale = 5.0;
  double hscale = 0.75;
  CvPoint start, end, h1, h2;

  /* Compute points for an arrow scaled and rotated by feat's scl and ori */
  start_x = cvRound (feat->x);
  start_y = cvRound (feat->y);
  scl = feat->s;
  ori = feat->dir;
  len = cvRound (scl * scale);
  hlen = cvRound (scl * hscale);
  blen = len - hlen;
  end_x = cvRound (len * cos (ori)) + start_x;
  end_y = cvRound (len * -sin (ori)) + start_y;
  h1_x = cvRound (blen * cos (ori + CV_PI / 18.0)) + start_x;
  h1_y = cvRound (blen * -sin (ori + CV_PI / 18.0)) + start_y;
  h2_x = cvRound (blen * cos (ori - CV_PI / 18.0)) + start_x;
  h2_y = cvRound (blen * -sin (ori - CV_PI / 18.0)) + start_y;
  start = cvPoint (start_x, start_y);
  end = cvPoint (end_x, end_y);
  h1 = cvPoint (h1_x, h1_y);
  h2 = cvPoint (h2_x, h2_y);

  cvLine (img, start, end, color, 1, 8, 0);
  cvLine (img, end, h1, color, 1, 8, 0);
  cvLine (img, end, h2, color, 1, 8, 0);
}


void
SceneRecognition::readFeaturesFromFile ()
{
  surf_descriptors.clear ();
  global_descriptors.clear ();

  files = QFileDialog::getOpenFileNames (0, "Load database files", ".", "TXT Files (*.txt)");

  if (!files.isEmpty ())
  {
    int file_number = files.size ();

    BOOST_FOREACH (QString s, files)
    {
      QFile file (s);
      if (file.open (QIODevice::ReadOnly | QIODevice::Text))
      {
        GlobalFeature global_feature;
        SURFDescriptor tmp_feature;
        std::vector< SURFDescriptor > tmp_surf_descriptors;
        tmp_surf_descriptors.clear ();

        // Read global features
        QByteArray line = file.readLine ();
        char* sep_str = line.data ();
        char sepr[] = " \t\n";

        char* str = strtok (sep_str, sepr);
        if (str != NULL)
          global_feature.area = atof (str);
        str = strtok (NULL, sepr);
        if (str != NULL)
          global_feature.duty_cycle = atof (str);
        global_descriptors.push_back (global_feature);

        // Read SURF features
        while (!file.atEnd ())
        {
          line = file.readLine ();
          sep_str = line.data ();

          //Read the pixel position(x,y)
          str = strtok (sep_str, sepr);
          if (str != NULL)
            tmp_feature.x = atof (str);
          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.y = atof (str);

          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.laplacian = atof (str);

          // Read the scale
          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.s = atof (str);

          // Read the principal direction
          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.dir = atof (str);

          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.mod = atof (str);

          // Read the corresponding laser point for this feature
          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.cor_point.x = atof (str);
          str = strtok (NULL, sepr);
          if (str != NULL)
            tmp_feature.cor_point.y = atof (str);
          str = strtok (NULL, sepr);
          if (str != NULL)
          tmp_feature.cor_point.z = atof (str);

          for (int i = 0; i < 128; i++)
          {
            str = strtok (NULL, sepr);
            if (str != NULL)
            tmp_feature.vector[i] = atof (str);
          }

          tmp_surf_descriptors.push_back (tmp_feature);
        }

        surf_descriptors.push_back (tmp_surf_descriptors);
        file.close ();
      }
      else
      {
        switch (QMessageBox::critical (NULL, "Critical", "Failed to read one database file!\n"
                "Would you like to read database files again, ignore it to continue or abort this option?",
                QMessageBox::Retry | QMessageBox::Ignore | QMessageBox::Abort, QMessageBox::Retry))
        {
          case QMessageBox::Retry:
            readFeaturesFromFile ();
            break;

          case QMessageBox::Ignore:
            break;

          case QMessageBox::Abort:
            return;
        }
      }
    }
  }
  else
  {
    switch (QMessageBox::critical (NULL, "Critical", "Failed to load database files!\n"
                                         "Would you like to reload them or abort this option?",
                                         QMessageBox::Retry | QMessageBox::Abort, QMessageBox::Retry))
    {
      case QMessageBox::Retry:
        readFeaturesFromFile ();
        break;

      case QMessageBox::Abort:
        break;
    }
  }
}


CvSeq*
SceneRecognition::findSURFMatchPairs (CvSeq* sep, int n, std::vector<SURFDescriptor> sfeat, CvMemStorage* storage)
{
  SURFDescriptor* feat = 0;
  SURFDescriptor** nbrs;
  SURFKDNode* kd_root;

  CvSeq* correspond = cvCreateSeq (0, sizeof (CvSeq), sizeof (SURFDescriptor), storage);

  // Build KD-Tree
  kd_root = buildKDTree (&sfeat[0], (int) (sfeat.size ()), 1);

  int match_num = 0;
  // Matching based on BBF
  for (int i = 0; i < n; i++)
  {
    // Take feature in the scene
    feat = (SURFDescriptor*) cvGetSeqElem (sep, i);
    // Use BBF to search matching pairs
    int k = KDTreeBbfKnn (kd_root, feat, 2, &nbrs, SURF_KDTREE_BBF_MAX_NN_CHKS, 1);
    if (k == 2)
    {
      double d0 = (double) descrDistSq (feat, nbrs[0], 1);
      double d1 = (double) descrDistSq (feat, nbrs[1], 1);
      if (d0 < d1 * SURF_NN_SQ_DIST_RATIO_THR)
      {
        cvSeqPush (correspond, feat);
        cvSeqPush (correspond, nbrs[0]);
        match_num++;
      }
    }
    free (nbrs);
  }

  // Filter the match results
  return correspond;
}


/******************************** KD-Tree ********************************/

SURFKDNode*
SceneRecognition::buildKDTree (SURFDescriptor* features, int n, int flags)
{
  SURFKDNode* kd_root;

  if (!features || n <= 0)
  {
    fprintf (stderr, "Warning: kdtree_build(): no features, %s, line %d\n", __FILE__, __LINE__);
    return NULL;
  }

  kd_root = initKDNode (features, n);
  expandKDNodeSubtree (kd_root, flags);

  return kd_root;
}

SURFKDNode*
SceneRecognition::initKDNode (SURFDescriptor* features, int n)
{
  SURFKDNode* kd_node;
  kd_node = (SURFKDNode*) malloc (sizeof (SURFKDNode));
  memset (kd_node, 0, sizeof (SURFKDNode));
  kd_node->ki = -1;
  kd_node->features = features;
  kd_node->n = n;

  return kd_node;
}

void
SceneRecognition::expandKDNodeSubtree (SURFKDNode* kd_node, int flags)
{
  /* Base case: leaf node */
  if (kd_node->n == 1 || kd_node->n == 0)
  {
    kd_node->leaf = 1;
    return;
  }

  assignPartKey (kd_node, flags);
  partitionFeatures (kd_node);

  if (kd_node->kd_left)
    expandKDNodeSubtree (kd_node->kd_left, flags);
  if (kd_node->kd_right)
    expandKDNodeSubtree (kd_node->kd_right, flags);
}

void
SceneRecognition::assignPartKey (SURFKDNode* kd_node, int flags)
{
  SURFDescriptor* features;
  double kv, x, mean, var, var_max = 0;
  double* tmp;
  int d, n, i, j, ki = 0;

  features = kd_node->features;
  n = kd_node->n;
  d = (flags & CV_SURF_EXTENDED) ? 128 : 64;

  /* Partition key index is that along which descriptors have most variance */
  for (j = 0; j < d; j++)
  {
    mean = var = 0;
    for (i = 0; i < n; i++)
      mean += features[i].vector[j];
    mean /= n;
    for (i = 0; i < n; i++)
    {
      x = features[i].vector[j] - mean;
      var += x * x;
    }
    var /= n;

    if (var > var_max)
    {
      ki = j;
      var_max = var;
    }
  }

  /* Partition key value is median of descriptor values at ki */
  tmp = reinterpret_cast < double* >(calloc (n, sizeof (double)));
  for (i = 0; i < n; i++)
  tmp[i] = features[i].vector[ki];
  kv = selectMedian (tmp, n);
  free (tmp);

  kd_node->ki = ki;
  kd_node->kv = kv;
}

double
SceneRecognition::selectMedian (double* array, int n)
{
  return selectRank (array, n, (n - 1) / 2);
}

double
SceneRecognition::selectRank (double* array, int n, int r)
{
  double *tmp, med;
  int gr_5, gr_tot, rem_elts, i, j;

  /* Base case */
  if (n == 1)
    return array[0];

  /* Divide array into groups of 5 and sort them */
  gr_5 = n / 5;
  gr_tot = cvCeil (n / 5.0);
  rem_elts = n % 5;
  tmp = array;
  for (i = 0; i < gr_5; i++)
  {
    sortInsertion (tmp, 5);
    tmp += 5;
  }
  sortInsertion (tmp, rem_elts);

  /* Recursively find the median of the medians of the groups of 5 */
  tmp = reinterpret_cast < double* >(calloc (gr_tot, sizeof (double)));
  for (i = 0, j = 2; i < gr_5; i++, j += 5)
    tmp[i] = array[j];
  if (rem_elts)
    tmp[i++] = array[n - 1 - rem_elts / 2];
  med = selectRank (tmp, i, (i - 1) / 2);
  free (tmp);

  /* Partition around median of medians and recursively select if necessary */
  j = partitionArray (array, n, med);
  if (r == j)
    return med;
  else if (r < j)
    return selectRank (array, j, r);
  else
  {
    array += j + 1;
    return selectRank (array, (n - j - 1), (r - j - 1));
  }
}

void
SceneRecognition::sortInsertion (double* array, int n)
{
  double k;
  int i, j;

  for (i = 1; i < n; i++)
  {
    k = array[i];
    j = i - 1;
    while (j >= 0 && array[j] > k)
    {
      array[j + 1] = array[j];
      j -= 1;
    }
    array[j + 1] = k;
  }
}

int
SceneRecognition::partitionArray (double* array, int n, double pivot)
{
  double tmp;
  int p, i, j;

  i = -1;
  for (j = 0; j < n; j++)
    if (array[j] <= pivot)
    {
      tmp = array[++i];
      array[i] = array[j];
      array[j] = tmp;
      if (array[i] == pivot)
        p = i;
    }

  array[p] = array[i];
  array[i] = pivot;

  return i;
}

void
SceneRecognition::partitionFeatures (SURFKDNode* kd_node)
{
  SURFDescriptor* features, tmp;
  double kv;
  int n, ki, p, i, j = -1;

  features = kd_node->features;
  n = kd_node->n;
  ki = kd_node->ki;
  kv = kd_node->kv;
  for (i = 0; i < n; i++)
  if (features[i].vector[ki] <= kv)
  {
    tmp = features[++j];
    features[j] = features[i];
    features[i] = tmp;
    if (features[j].vector[ki] == kv)
      p = j;
  }
  tmp = features[p];
  features[p] = features[j];
  features[j] = tmp;

  /* If all records fall on same side of partition, make node a leaf */
  if (j == n - 1)
  {
    kd_node->leaf = 1;
    return;
  }

  kd_node->kd_left = initKDNode (features, j + 1);
  
  kd_node->kd_right = initKDNode (features + (j + 1), (n - j - 1));
}

/********************************** End **********************************/


int
SceneRecognition::KDTreeBbfKnn (SURFKDNode* kd_root, SURFDescriptor* feat, int k,
                                SURFDescriptor*** nbrs, int max_nn_chks, int flags)
{
  SURFKDNode* expl;
  MinPq* min_pq;
  SURFDescriptor* tree_feat, **_nbrs;
  BbfData* bbf_data;
  int i, t = 0, n = 0;

  if (!nbrs || !feat || !kd_root)
  {
    fprintf (stderr, "Warning: NULL pointer error, %s, line %d\n", __FILE__, __LINE__);
    return -1;
  }

  _nbrs = reinterpret_cast < SURFDescriptor** >(calloc (k, sizeof (SURFDescriptor*)));
  min_pq = initMinPq ();
  insertMinPq (min_pq, kd_root, 0);
  while (min_pq->n > 0 && t < max_nn_chks)
  {
    expl = (SURFKDNode*) minPqExtractMin (min_pq);
    if (!expl)
    {
      fprintf (stderr, "Warning: PQ unexpectedly empty, %s line %d\n", __FILE__, __LINE__);
      goto fail;
    }

    expl = exploreToLeaf (expl, feat, min_pq, flags);
    if (!expl)
    {
      fprintf (stderr, "Warning: PQ unexpectedly empty, %s line %d\n", __FILE__, __LINE__);
      goto fail;
    }

    for (i = 0; i < expl->n; i++)
    {
      tree_feat = &expl->features[i];
      bbf_data = (BbfData*) malloc (sizeof (BbfData));
      if (!bbf_data)
      {
        fprintf (stderr, "Warning: unable to allocate memory, %s line %d\n", __FILE__, __LINE__);
        goto fail;
      }

      bbf_data->old_data = tree_feat->feature_data;
      bbf_data->d = descrDistSq (feat, tree_feat, flags);
      tree_feat->feature_data = bbf_data;
      n += insertIntoNbrArray (tree_feat, _nbrs, n, k);
    }
    t++;
  }

  releaseMinPq (&min_pq);
  for (i = 0; i < n; i++)
  {
    bbf_data = (BbfData*) _nbrs[i]->feature_data;
    _nbrs[i]->feature_data = bbf_data->old_data;
    free (bbf_data);
  }
  *nbrs = _nbrs;
  return n;

  fail:
  releaseMinPq (&min_pq);
  for (i = 0; i < n; i++)
  {
    bbf_data = (BbfData*) _nbrs[i]->feature_data;
    _nbrs[i]->feature_data = bbf_data->old_data;
    free (bbf_data);
  }
  free (_nbrs);
  *nbrs = NULL;
  return -1;
}

MinPq*
SceneRecognition::initMinPq ()
{
  MinPq* min_pq;
  min_pq = (MinPq*) malloc (sizeof (MinPq));
  min_pq->pq_array = (PqNode*) calloc (MINPQ_INIT_NALLOCD, sizeof (PqNode));
  min_pq->nallocd = MINPQ_INIT_NALLOCD;
  min_pq->n = 0;

  return min_pq;
}

int
SceneRecognition::insertMinPq (MinPq* min_pq, void* data, int key)
{
  int n = min_pq->n;

  /* Double array allocation if necessary */
  if (min_pq->nallocd == n)
  {
    min_pq->nallocd = doubleArray ((void**) &min_pq->pq_array, min_pq->nallocd, sizeof (PqNode));
    if (!min_pq->nallocd)
    {
      fprintf (stderr, "Warning: unable to allocate memory, %s, line %d\n", __FILE__, __LINE__);
      return 1;
    }
  }

  min_pq->pq_array[n].data = data;
  min_pq->pq_array[n].key = INT_MAX;
  decreasePqNodeKey (min_pq->pq_array, min_pq->n, key);
  min_pq->n++;

  return 0;
}


void
SceneRecognition::decreasePqNodeKey (PqNode* pq_array, int i, int key)
{
  PqNode tmp;

  if (key > pq_array[i].key)
    return;

  pq_array[i].key = key;
  while (i > 0 && pq_array[i].key < pq_array[parent (i)].key)
  {
    tmp = pq_array[parent (i)];
    pq_array[parent (i)] = pq_array[i];
    pq_array[i] = tmp;
    i = parent (i);
  }
}


/*
* Recursively restores correct priority queue order to a minimizing pq array
*  @param pq_array a minimizing priority queue array
*  @param i index at which to start reordering
*  @param n number of elements in \a pq_array
*/
void
SceneRecognition::restoreMinPqOrder (PqNode* pq_array, int i, int n)
{
  PqNode tmp;
  int l, r, min = i;

  l = left (i);
  r = right (i);
  if (l < n)
    if (pq_array[l].key < pq_array[i].key)
      min = l;
  if (r < n)
    if (pq_array[r].key < pq_array[min].key)
      min = r;

  if (min != i)
  {
    tmp = pq_array[min];
    pq_array[min] = pq_array[i];
    pq_array[i] = tmp;
    restoreMinPqOrder (pq_array, min, n);
  }
}

void*
SceneRecognition::minPqExtractMin (MinPq* min_pq)
{
  void* data;

  if (min_pq->n < 1)
  {
    fprintf (stderr, "Warning: PQ empty, %s line %d\n", __FILE__, __LINE__);
    return NULL;
  }
  data = min_pq->pq_array[0].data;
  min_pq->n--;
  min_pq->pq_array[0] = min_pq->pq_array[min_pq->n];
  restoreMinPqOrder (min_pq->pq_array, 0, min_pq->n);

  return data;
}

SURFKDNode*
SceneRecognition::exploreToLeaf (SURFKDNode* kd_node, SURFDescriptor* feat, MinPq* min_pq, int flags)
{
  SURFKDNode* unexpl, *expl = kd_node;
  double kv;
  int ki;
  int d = (flags & CV_SURF_EXTENDED) ? 128 : 64;

  while (expl && !expl->leaf)
  {
    ki = expl->ki;
    kv = expl->kv;

    if (ki >= d)
    {
      fprintf (stderr, "Warning: comparing imcompatible descriptors, %s line %d\n", __FILE__, __LINE__);
      return NULL;
    }
    if (feat->vector[ki] <= kv)
    {
      unexpl = expl->kd_right;
      expl = expl->kd_left;
    }
    else
    {
      unexpl = expl->kd_left;
      expl = expl->kd_right;
    }

    if (insertMinPq (min_pq, unexpl, (int) abs (kv - feat->vector[ki])))
    {
      fprintf (stderr, "Warning: unable to insert into PQ, %s, line %d\n", __FILE__, __LINE__);
      return NULL;
    }
  }

  return expl;
}

double
SceneRecognition::descrDistSq (SURFDescriptor* f1, SURFDescriptor* f2, int flags)
{
  double diff, dsq = 0;
  double *descr1, *descr2;
  int i;
  int d = (flags & CV_SURF_EXTENDED) ? 128 : 64;

  descr1 = f1->vector;
  descr2 = f2->vector;

  for (i = 0; i < d; i++)
  {
    diff = descr1[i] - descr2[i];
    dsq += diff * diff;
  }

  return dsq;
}

int
SceneRecognition::insertIntoNbrArray (SURFDescriptor* feat, SURFDescriptor** nbrs, int n, int k)
{
  BbfData *fdata, *ndata;
  double dn, df;
  int i, ret = 0;

  if (n == 0)
  {
    nbrs[0] = feat;
    return 1;
  }

  /* Check at end of array */
  fdata = (BbfData*) feat->feature_data;
  df = fdata->d;
  ndata = (BbfData*) nbrs[n - 1]->feature_data;
  dn = ndata->d;
  if (df >= dn)
  {
    if (n == k)
      {
        feat->feature_data = fdata->old_data;
        free (fdata);
        return 0;
      }
      nbrs[n] = feat;
      return 1;
  }

  /* Find the right place in the array */
  if (n < k)
  {
    nbrs[n] = nbrs[n - 1];
    ret = 1;
  }
  else
  {
    nbrs[n - 1]->feature_data = ndata->old_data;
    free (ndata);
  }
  i = n - 2;
  while (i >= 0)
  {
    ndata = (BbfData*) nbrs[i]->feature_data;
    dn = ndata->d;
    if (dn <= df)
      break;
    nbrs[i + 1] = nbrs[i];
    i--;
  }
  i++;
  nbrs[i] = feat;

  return ret;
}

void
SceneRecognition::releaseMinPq (MinPq** min_pq)
{
  if (!min_pq)
  {
    fprintf (stderr, "Warning: NULL pointer error, %s line %d\n", __FILE__, __LINE__);
    return;
  }
  if (*min_pq && (*min_pq)->pq_array)
  {
    free ((*min_pq)->pq_array);
    free (*min_pq);
    *min_pq = NULL;
  }
}

int
SceneRecognition::doubleArray (void** array, int n, int size)
{
  void* tmp;

  tmp = realloc (*array, 2 * n * size);
  if (!tmp)
  {
    fprintf (stderr, "Warning: unable to allocate memory in array_double(), %s line %d\n", __FILE__, __LINE__);
    if (*array)
      free (*array);
    *array = NULL;
    return 0;
  }
  *array = tmp;
  return n * 2;
}

double
SceneRecognition::getMatchDegree (CvSeq* seq)
{
  // Calculate the ratation translation matrix, excluding mismatching
  execute (seq);

  /***************** calculate error *****************/
  int n = seq->total / 2;
  if_false_pairs.resize (n);
  for (int eff = 0; eff < n; eff++)
  {
    if_false_pairs[eff] = false;
  }
  eff_match_pairs = 0;
  if ((int) RMatrixPlist.size () <= 0)
    for (int eff = 0; eff < n; eff++)
    {
      if_false_pairs[eff] = true;
    }

  double RMatrixt[9];
  double TVectort[3];
  double max_match = -111, match_deg;

  for (int i = 0; i < (int) RMatrixPlist.size (); i++)
  {
    for (int j = 0; j < 9; j++)
    {
      RMatrixt[j] = RMatrixPlist[i][j];
    }
    for (int j = 0; j < 3; j++)
      TVectort[j] = TVectorPlist[i][j];

    match_deg = calculateErr (seq, RMatrixt, TVectort);

    if (match_deg > max_match)
    {
      eff_match_pairs = 0;
      for (int number = 0; number < n; number++)
      {
        if (if_false_pairs[number] == false)
          eff_match_pairs++;
      }
      if (eff_match_pairs >= MIN_AMOUNT)
      {
        max_match = match_deg;
      }
    }
  }

  return (max_match);
}

void
SceneRecognition::execute (CvSeq* seq)
{
  transMatrix (seq);
  initSelect (seq);
  do
  {
    cutFalseMatrix ();
  } while ((int) RMatrixPlist.size () > 10);
}

void
SceneRecognition::transMatrix (CvSeq* seq)
{
  RMatrixPlist.clear ();
  TVectorPlist.clear ();
  int surfpairmum = (seq->total / 2);
  if ((seq->total / 2) < 4)
    return;

  for (int n = 3; n < surfpairmum; n++)
  {
    if ((int) RMatrixPlist.size () >= 1000)
    {
      break;
    }
    for (int m = 2; m < n; m++)
    {
      if ((int) RMatrixPlist.size () >= 1000)
      {
        break;
      }
      for (int j = 1; j < m; j++)
      {
        if ((int) RMatrixPlist.size () >= 1000)
        {
          break;
        }
        for (int i = 0; i < j; i++)
        {
          SURFDescriptor * tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * i);
          sce_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * j);
          sce_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * m);
          sce_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * n);
          sce_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * i + 1);
          lib_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * j + 1);
          lib_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * m + 1);
          lib_points.push_back (tmp->cor_point);
          tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * n + 1);
          lib_points.push_back (tmp->cor_point);
          Matrix ();
          SVD ();
          double* RMatrixp;
          double* TVectorp;
          RMatrixp = new double[9];
          TVectorp = new double[3];
          for (int i = 0; i < 9; i++)
          {
            RMatrixp[i] = RMatrix[i];
          }
          for (int j = 0; j < 3; j++)
          {
            TVectorp[j] = TVector[j];
          }
          RMatrixPlist.push_back (RMatrixp);
          TVectorPlist.push_back (TVectorp);
          sce_points.clear ();
          lib_points.clear ();
          if ((int) RMatrixPlist.size () >= 1000)
          {
            break;
          }
        }
      }
    }
  }
}

/* Calculate A*AH and AH*A matrix */
void
SceneRecognition::Matrix ()
{
  std::vector < pcl::PointXYZ >::iterator IT1;
  std::vector < pcl::PointXYZ >::iterator IT2;
  centroid1.x = 0;
  centroid1.y = 0;
  centroid1.z = 0;
  centroid2.x = 0;
  centroid2.y = 0;
  centroid2.z = 0;

  for (IT1 = lib_points.begin (); IT1 != lib_points.end (); IT1++)
  {
    centroid1.x = centroid1.x + (*IT1).x;
    centroid1.y = centroid1.y + (*IT1).y;
    centroid1.z = centroid1.z + (*IT1).z;
  }
  centroid1.x = centroid1.x / lib_points.size ();
  centroid1.y = centroid1.y / lib_points.size ();
  centroid1.z = centroid1.z / lib_points.size ();
  for (IT1 = lib_points.begin (); IT1 != lib_points.end (); IT1++)
  {
    (*IT1).x = (*IT1).x - centroid1.x;
    (*IT1).y = (*IT1).y - centroid1.y;
    (*IT1).z = (*IT1).z - centroid1.z;
  }

  for (IT2 = sce_points.begin (); IT2 != sce_points.end (); IT2++)
  {
    centroid2.x = centroid2.x + (*IT2).x;
    centroid2.y = centroid2.y + (*IT2).y;
    centroid2.z = centroid2.z + (*IT2).z;
  }
  centroid2.x = centroid2.x / sce_points.size ();
  centroid2.y = centroid2.y / sce_points.size ();
  centroid2.z = centroid2.z / sce_points.size ();
  for (IT2 = sce_points.begin (); IT2 != sce_points.end (); IT2++)
  {
    (*IT2).x = (*IT2).x - centroid2.x;
    (*IT2).y = (*IT2).y - centroid2.y;
    (*IT2).z = (*IT2).z - centroid2.z;
  }

  /* Initialization */
  A[0] = 0;
  A[1] = 0;
  A[2] = 0;
  A[3] = 0;
  A[4] = 0;
  A[5] = 0;
  A[6] = 0;
  A[7] = 0;
  A[8] = 0;

  for (IT1 = lib_points.begin (), IT2 = sce_points.begin (); IT1 != lib_points.end (); IT1++, IT2++)
  {
    /* A */
    A[0] = A[0] + (*IT2).x * (*IT1).x;
    A[1] = A[1] + (*IT2).x * (*IT1).y;
    A[2] = A[2] + (*IT2).x * (*IT1).z;
    A[3] = A[3] + (*IT2).y * (*IT1).x;
    A[4] = A[4] + (*IT2).y * (*IT1).y;
    A[5] = A[5] + (*IT2).y * (*IT1).z;
    A[6] = A[6] + (*IT2).z * (*IT1).x;
    A[7] = A[7] + (*IT2).z * (*IT1).y;
    A[8] = A[8] + (*IT2).z * (*IT1).z;
  }

  /* A*AH */
  A_AH[0] = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
  A_AH[1] = A[0] * A[3] + A[1] * A[4] + A[2] * A[5];
  A_AH[2] = A[0] * A[6] + A[1] * A[7] + A[2] * A[8];
  A_AH[3] = A[3] * A[0] + A[4] * A[1] + A[5] * A[2];
  A_AH[4] = A[3] * A[3] + A[4] * A[4] + A[5] * A[5];
  A_AH[5] = A[3] * A[6] + A[4] * A[7] + A[5] * A[8];
  A_AH[6] = A[6] * A[0] + A[7] * A[1] + A[8] * A[2];
  A_AH[7] = A[6] * A[3] + A[7] * A[4] + A[8] * A[5];
  A_AH[8] = A[6] * A[6] + A[7] * A[7] + A[8] * A[8];

  /* AH*A */
  AH_A[0] = A[0] * A[0] + A[3] * A[3] + A[6] * A[6];
  AH_A[1] = A[0] * A[1] + A[3] * A[4] + A[6] * A[7];
  AH_A[2] = A[0] * A[2] + A[3] * A[5] + A[6] * A[8];
  AH_A[3] = A[1] * A[0] + A[4] * A[3] + A[7] * A[6];
  AH_A[4] = A[1] * A[1] + A[4] * A[4] + A[7] * A[7];
  AH_A[5] = A[1] * A[2] + A[4] * A[5] + A[7] * A[8];
  AH_A[6] = A[2] * A[0] + A[5] * A[3] + A[8] * A[6];
  AH_A[7] = A[2] * A[1] + A[5] * A[4] + A[8] * A[7];
  AH_A[8] = A[2] * A[2] + A[5] * A[5] + A[8] * A[8];
}

/* Calculate rotation matrix R and translation vector T */
void
SceneRecognition::SVD ()
{
  int SUC1 = Jcbi (A_AH, U); /* Calculate eigenvector and eigenvalue of A*AH */
  int SUC2 = Jcbi (AH_A, V); /* Calculate eigenvector and eigenvalue of AH*A */

  RMatrix[0] = V[0] * U[0] + V[1] * U[1] + V[2] * U[2];
  RMatrix[1] = V[0] * U[3] + V[1] * U[4] + V[2] * U[5];
  RMatrix[2] = V[0] * U[6] + V[1] * U[7] + V[2] * U[8];
  RMatrix[3] = V[3] * U[0] + V[4] * U[1] + V[5] * U[2];
  RMatrix[4] = V[3] * U[3] + V[4] * U[4] + V[5] * U[5];
  RMatrix[5] = V[3] * U[6] + V[4] * U[7] + V[5] * U[8];
  RMatrix[6] = V[6] * U[0] + V[7] * U[1] + V[8] * U[2];
  RMatrix[7] = V[6] * U[3] + V[7] * U[4] + V[8] * U[5];
  RMatrix[8] = V[6] * U[6] + V[7] * U[7] + V[8] * U[8];

  TVector[0] = centroid1.x - RMatrix[0] * centroid2.x - RMatrix[1] * centroid2.y - RMatrix[2] * centroid2.z;
  TVector[1] = centroid1.y - RMatrix[3] * centroid2.x - RMatrix[4] * centroid2.y - RMatrix[5] * centroid2.z;
  TVector[2] = centroid1.z - RMatrix[6] * centroid2.x - RMatrix[7] * centroid2.y - RMatrix[8] * centroid2.z;
}

/* Calculate eigenvalue and eigenvector */
int 
SceneRecognition::Jcbi (double A[], double V[])
{
  int i, j, p, q, u, w, t, s, l;
  double fm, cn, sn, omega, x, y, d;
  l = 1;
  double eps = 0.1;
  int jt = 10;
  int n = 3;
  for (i = 0; i <= n - 1; i++)
  {
    V[i * n + i] = 1.0;
    for (j = 0; j <= n - 1; j++)
      if (i != j)
        V[i * n + j] = 0.0;
  }
  while (1 == 1)
  {
    fm = 0.0;
    for (i = 1; i <= n - 1; i++)
      for (j = 0; j <= i - 1; j++)
        {
          d = fabs (A[i * n + j]);
          if ((i != j) && (d > fm))
          {
            fm = d;
            p = i;
            q = j;
          }
        }

    if (fm < eps)
      return (1);
    if (l > jt)
      return (-1);
    l = l + 1;
    u = p * n + q;
    w = p * n + p;
    t = q * n + p;
    s = q * n + q;
    x = -A[u];
    y = (A[s] - A[w]) / 2.0;
    omega = x / sqrt (x * x + y * y);
    if (y < 0.0)
      omega = -omega;
    sn = 1.0 + sqrt (1.0 - omega * omega);
    sn = omega / sqrt (2.0 * sn);
    cn = sqrt (1.0 - sn * sn);
    fm = A[w];
    A[w] = fm * cn * cn + A[s] * sn * sn + A[u] * omega;
    A[s] = fm * sn * sn + A[s] * cn * cn - A[u] * omega;
    A[u] = 0.0;
    A[t] = 0.0;
    for (j = 0; j <= n - 1; j++)
    {
      if ((j != p) && (j != q))
      {
        u = p * n + j;
        w = q * n + j;
        fm = A[u];
        A[u] = fm * cn + A[w] * sn;
        A[w] = -fm * sn + A[w] * cn;
      }
    }

    for (i = 0; i <= n - 1; i++)
    {
      if ((i != p) && (i != q))
      {
        u = i * n + p;
        w = i * n + q;
        fm = A[u];
        A[u] = fm * cn + A[w] * sn;
        A[w] = -fm * sn + A[w] * cn;
      }
    }

    for (i = 0; i <= n - 1; i++)
    {
      u = i * n + p;
      w = i * n + q;
      fm = V[u];
      V[u] = fm * cn + V[w] * sn;
      V[w] = -fm * sn + V[w] * cn;
    }
  }

  return (1);
}

void
SceneRecognition::initSelect (CvSeq* seq)
{
  std::vector < double* >::iterator Riter;
  std::vector < double* >::iterator Titer;
  for (Riter = RMatrixPlist.begin (), Titer = TVectorPlist.begin (); Riter < RMatrixPlist.end ();)
  {
    if (!Marix (seq, *Riter, *Titer))
    {
      delete[](*Riter);
      Riter = RMatrixPlist.erase (Riter);
      delete[](*Titer);
      Titer = TVectorPlist.erase (Titer);
    }
    else
    {
      Riter++;
      Titer++;
    }
  }
}

bool
SceneRecognition::Marix (CvSeq* seq, double* RMatrix, double* TVector)
{
  for (int i = 0; i < (seq->total / 2); i++)
  {
    pcl::PointXYZ tem_point;
    SURFDescriptor* tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * i);

    tem_point.x = RMatrix[0] * tmp->cor_point.x + RMatrix[1] * tmp->cor_point.y +
                  RMatrix[2] * tmp->cor_point.z + TVector[0];

    tem_point.y = RMatrix[3] * tmp->cor_point.x + RMatrix[4] * tmp->cor_point.y +
                  RMatrix[5] * tmp->cor_point.z + TVector[1];

    tem_point.z = RMatrix[6] * tmp->cor_point.x + RMatrix[7] * tmp->cor_point.y +
                  RMatrix[8] * tmp->cor_point.z + TVector[2];

    tmp = (SURFDescriptor*) cvGetSeqElem (seq, 2 * i + 1);
    double dist = sqrt ((tem_point.x - tmp->cor_point.x) * (tem_point.x - tmp->cor_point.x) +
                        (tem_point.y - tmp->cor_point.y) * (tem_point.y - tmp->cor_point.y) +
                        (tem_point.z - tmp->cor_point.z) * (tem_point.z - tmp->cor_point.z));

    if (dist <= DISMAX)
    {
      return true;
    }
  }

  return false;
}

/* Delete the false rotation matrix */
void 
SceneRecognition::cutFalseMatrix ()
{
  int rawDataNum = int (RMatrixPlist.size ());
  if (rawDataNum >= 2)
  {
    CvMat* rawData_matrix = cvCreateMat (rawDataNum, MAXDIM, CV_32FC1);
    CvMat* clusters = cvCreateMat (rawDataNum, 1, CV_32SC1);  /* After cluster */

    for (int i = 0; i < rawDataNum; i++)
    {
      rawData_matrix->data.fl[i * MAXDIM] = RMatrixPlist[i][0];
      rawData_matrix->data.fl[i * MAXDIM + 1] = RMatrixPlist[i][1];
      rawData_matrix->data.fl[i * MAXDIM + 2] = RMatrixPlist[i][2];
      rawData_matrix->data.fl[i * MAXDIM + 3] = RMatrixPlist[i][3];
      rawData_matrix->data.fl[i * MAXDIM + 4] = RMatrixPlist[i][4];
      rawData_matrix->data.fl[i * MAXDIM + 5] = RMatrixPlist[i][5];
      rawData_matrix->data.fl[i * MAXDIM + 6] = RMatrixPlist[i][6];
      rawData_matrix->data.fl[i * MAXDIM + 7] = RMatrixPlist[i][7];
      rawData_matrix->data.fl[i * MAXDIM + 8] = RMatrixPlist[i][8];
    }
    /* Use K-means clustering into two groups */
    cvKMeans2 (rawData_matrix, CLUSTERINGNUM, clusters,
               cvTermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.01));
    int Cluster1Num = 0;
    int Clsuter2Num = 0;
    for (int j = 0; j < rawDataNum; j++)
    {
      if (clusters->data.i[j] == 0)
      {
        Cluster1Num++;
      }
      else
      Clsuter2Num++;
    }
 
    int TrueTag;
    TrueTag = (int) (Cluster1Num > Clsuter2Num ? 0 : 1);
    std::vector < double* >::iterator Riter;
    std::vector < double* >::iterator Titer;
    int j = 0;
    for (Riter = RMatrixPlist.begin (), Titer = TVectorPlist.begin (); Riter < RMatrixPlist.end ();)
    {
      if (clusters->data.i[j] != TrueTag)
      {
        delete[](*Riter);
        Riter = RMatrixPlist.erase (Riter);
        delete[](*Titer);
        Titer = TVectorPlist.erase (Titer);
      }
      else
      {
        Riter++;
        Titer++;
      }
      j++;
    }
  }
  else
  {
    return;
  }
}

double
SceneRecognition::calculateErr (CvSeq* seq, double RMatrix[9], double TVector[3])
{
  /* All participating checkout */
  int n = seq->total;
  if_false_pairs.resize (n);
  for (int eff = 0; eff < n; eff++)
  {
    if_false_pairs[eff] = false;
  } 
  double sum_err = 0;
  int counter = n;
  for (int i = 0; i < n; i++)
  {
    SURFDescriptor* tmp1 = (SURFDescriptor*) cvGetSeqElem (seq, 2 * i);
    pcl::PointXYZ data = tmp1->cor_point;  /* Corresponding laser scanning data */
    pcl::PointXYZ data4;
    data4.x = RMatrix[0] * data.x + RMatrix[1] * data.y + RMatrix[2] * data.z + TVector[0];
    data4.y = RMatrix[3] * data.x + RMatrix[4] * data.y + RMatrix[5] * data.z + TVector[1];
    data4.z = RMatrix[6] * data.x + RMatrix[7] * data.y + RMatrix[8] * data.z + TVector[2];
    bool flag = false;
    double max = 0.0;
    tmp1 = (SURFDescriptor*) cvGetSeqElem (seq, 2 * i + 1);
    pcl::PointXYZ data3 = tmp1->cor_point;

    double d1 = sqrt ((data4.x - data3.x) * (data4.x - data3.x) +
                      (data4.y - data3.y) * (data4.y - data3.y) +
                      (data4.z - data3.z) * (data4.z - data3.z));
    double d2 = sqrt ((data4.x * data4.x) + (data4.y * data4.y) + (data4.z * data4.z));
    double d3 = sqrt ((data3.x * data3.x) + (data3.y * data3.y) + (data3.z * data3.z));

    double alph = acos ((d2 * d2 + d3 * d3 - d1 * d1) / (2 * d2 * d3));
    alph = alph * 180 / PI;
    if (alph < DANGLE)      /* A range of given angle */
    {
      flag = true;
      if (max < (1 - alph / DANGLE))
      max = (1 - alph / DANGLE);
    }

    if (flag == false)
    {
      if_false_pairs[i] = true;
      counter--;
    }
    sum_err += max;
  }

  double s = 0;
  if (counter > 0 && counter < MINN)
  {
    double t = -(counter - MINN) * (counter - MINN) / (2 * 3 * 3);
    s = (1.0 / counter) * (sum_err) * exp (t);
  }
  else if (counter <= 0)
    s = 0;
  else
    s = (1.0 / counter) * (sum_err);

  return s;
}