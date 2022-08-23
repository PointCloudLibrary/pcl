/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception , Inc.
 *  Copyright (C) 2011  The Autonomous Systems Lab (ASL), ETH Zurich,
 *                      Stefan Leutenegger, Simon Lynen and Margarita Chli.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */


#ifndef PCL_FEATURES_IMPL_BRISK_2D_HPP_
#define PCL_FEATURES_IMPL_BRISK_2D_HPP_


namespace pcl
{

template <typename PointInT, typename PointOutT, typename KeypointT, typename IntensityT>
BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>::BRISK2DEstimation ()
  : rotation_invariance_enabled_ (true)
  , scale_invariance_enabled_ (true)
  , pattern_scale_ (1.0f)
  , input_cloud_ (), keypoints_ (), scale_range_ (), pattern_points_ (), points_ ()
  , n_rot_ (1024), scale_list_ (nullptr), size_list_ (nullptr)
  , scales_ (64)
  , scalerange_ (30)
  , basic_size_ (12.0)
  , strings_ (0), d_max_ (0.0f), d_min_ (0.0f), short_pairs_ (), long_pairs_ ()
  , no_short_pairs_ (0), no_long_pairs_ (0)
  , intensity_ ()
  , name_ ("BRISK2Destimation")
{
  // Since we do not assume pattern_scale_ should be changed by the user, we
  // can initialize the kernel in the constructor
  std::vector<float> r_list;
  std::vector<int> n_list;

  // this is the standard pattern found to be suitable also
  r_list.resize (5);
  n_list.resize (5);
  const float f = 0.85f * pattern_scale_;

  r_list[0] = f * 0.0f;
  r_list[1] = f * 2.9f;
  r_list[2] = f * 4.9f;
  r_list[3] = f * 7.4f;
  r_list[4] = f * 10.8f;

  n_list[0] = 1;
  n_list[1] = 10;
  n_list[2] = 14;
  n_list[3] = 15;
  n_list[4] = 20;

  generateKernel (r_list, n_list, 5.85f * pattern_scale_, 8.2f * pattern_scale_);
}


template <typename PointInT, typename PointOutT, typename KeypointT, typename IntensityT>
BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>::~BRISK2DEstimation ()
{
  delete [] pattern_points_;
  delete [] short_pairs_;
  delete [] long_pairs_;
  delete [] scale_list_;
  delete [] size_list_;
}


template <typename PointInT, typename PointOutT, typename KeypointT, typename IntensityT> void
BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>::generateKernel (
    std::vector<float> &radius_list,
    std::vector<int> &number_list, float d_max, float d_min,
    std::vector<int> index_change)
{
  d_max_ = d_max;
  d_min_ = d_min;

  // get the total number of points
  const auto rings = radius_list.size ();
  assert (radius_list.size () != 0 && radius_list.size () == number_list.size ());
  points_ = 0; // remember the total number of points
  for (const auto number: number_list)
    points_ += number;

  // set up the patterns
  pattern_points_ = new BriskPatternPoint[points_*scales_*n_rot_];
  BriskPatternPoint* pattern_iterator = pattern_points_;

  // define the scale discretization:
  static const float lb_scale = std::log (scalerange_) / std::log (2.0);
  static const float lb_scale_step = lb_scale / (float (scales_));

  scale_list_ = new float[scales_];
  size_list_  = new unsigned int[scales_];

  const float sigma_scale = 1.3f;

  for (unsigned int scale = 0; scale < scales_; ++scale)
  {
    scale_list_[scale] = static_cast<float> (pow (double (2.0), static_cast<double> (float (scale) * lb_scale_step)));
    size_list_[scale]  = 0;

    // generate the pattern points look-up
    for (std::size_t rot = 0; rot < n_rot_; ++rot)
    {
      // this is the rotation of the feature
      double theta = double (rot) * 2 * M_PI / double (n_rot_);
      for (int ring = 0; ring < static_cast<int>(rings); ++ring)
      {
        for (int num = 0; num < number_list[ring]; ++num)
        {
          // the actual coordinates on the circle
          double alpha = double (num) * 2 * M_PI / double (number_list[ring]);

          // feature rotation plus angle of the point
          pattern_iterator->x = scale_list_[scale] * radius_list[ring] * static_cast<float> (std::cos (alpha + theta));
          pattern_iterator->y = scale_list_[scale] * radius_list[ring] * static_cast<float> (sin (alpha + theta));
          // and the gaussian kernel sigma
          if (ring == 0)
            pattern_iterator->sigma = sigma_scale * scale_list_[scale] * 0.5f;
          else
            pattern_iterator->sigma = static_cast<float> (sigma_scale * scale_list_[scale] * (double (radius_list[ring])) * sin (M_PI / double (number_list[ring])));

          // adapt the sizeList if necessary
          const auto size = static_cast<unsigned int> (std::ceil (((scale_list_[scale] * radius_list[ring]) + pattern_iterator->sigma)) + 1);

          if (size_list_[scale] < size)
            size_list_[scale] = size;

          // increment the iterator
          ++pattern_iterator;
        }
      }
    }
  }

  // now also generate pairings
  short_pairs_ = new BriskShortPair[points_ * (points_ - 1) / 2];
  long_pairs_ = new BriskLongPair[points_ * (points_ - 1) / 2];
  no_short_pairs_ = 0;
  no_long_pairs_  = 0;

  // fill index_change with 0..n if empty
  if (index_change.empty ())
  {
    index_change.resize (points_ * (points_ - 1) / 2);
  }
  std::iota(index_change.begin (), index_change.end (), 0);

  const float d_min_sq = d_min_ * d_min_;
  const float d_max_sq  = d_max_ * d_max_;
  for (unsigned int i = 1; i < points_; i++)
  {
    for (unsigned int j = 0; j < i; j++)
    { //(find all the pairs)
      // point pair distance:
      const float dx = pattern_points_[j].x - pattern_points_[i].x;
      const float dy = pattern_points_[j].y - pattern_points_[i].y;
      const float norm_sq = (dx*dx+dy*dy);
      if (norm_sq > d_min_sq)
      {
        // save to long pairs
        BriskLongPair& longPair = long_pairs_[no_long_pairs_];
        longPair.weighted_dx = int ((dx / (norm_sq)) * 2048.0 + 0.5);
        longPair.weighted_dy = int ((dy / (norm_sq)) * 2048.0 + 0.5);
        longPair.i = i;
        longPair.j = j;
        ++no_long_pairs_;
      }
      else if (norm_sq < d_max_sq)
      {
        // save to short pairs
        assert (no_short_pairs_ < index_change.size ()); // make sure the user passes something sensible
        BriskShortPair& shortPair = short_pairs_[index_change[no_short_pairs_]];
        shortPair.j = j;
        shortPair.i = i;
        ++no_short_pairs_;
      }
    }
  }

  // no bits:
  strings_ = int (std::ceil ((float (no_short_pairs_)) / 128.0)) * 4 * 4;
}


template <typename PointInT, typename PointOutT, typename KeypointT, typename IntensityT> inline int
BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>::smoothedIntensity (
    const std::vector<unsigned char> &image,
    int image_width, int,
    //const Stefan& integral,
    const std::vector<int> &integral_image,
    const float key_x, const float key_y, const unsigned int scale,
    const unsigned int rot, const unsigned int point) const
{
  // get the float position
  const BriskPatternPoint& brisk_point = pattern_points_[scale * n_rot_*points_ + rot * points_ + point];
  const float xf = brisk_point.x + key_x;
  const float yf = brisk_point.y + key_y;
  const int x = int (xf);
  const int y = int (yf);
  const int& imagecols = image_width;

  // get the sigma:
  const float sigma_half = brisk_point.sigma;
  const float area = 4.0f * sigma_half * sigma_half;

  // Get the point step

  // calculate output:
  int ret_val;
  if (sigma_half < 0.5)
  {
    // interpolation multipliers:
    const int r_x   = static_cast<int> ((xf - float (x)) * 1024);
    const int r_y   = static_cast<int> ((yf - float (y)) * 1024);
    const int r_x_1 = (1024 - r_x);
    const int r_y_1 = (1024 - r_y);

    //+const unsigned char* ptr = static_cast<const unsigned char*> (&image[0].r) + x + y * imagecols;
    const unsigned char* ptr = static_cast<const unsigned char*>(&image[0]) + x + y * imagecols;

    // just interpolate:
    ret_val = (r_x_1 * r_y_1 * int (*ptr));

    //+ptr += sizeof (PointInT);
    ptr++;

    ret_val += (r_x * r_y_1 * int (*ptr));

    //+ptr += (imagecols * sizeof (PointInT));
    ptr += imagecols;

    ret_val += (r_x * r_y * int (*ptr));

    //+ptr -= sizeof (PointInT);
    ptr--;

    ret_val += (r_x_1 * r_y * int (*ptr));
    return (ret_val + 512) / 1024;
  }

  // this is the standard case (simple, not speed optimized yet):

  // scaling:
  const int scaling  = static_cast<int> (4194304.0f / area);
  const int scaling2 = static_cast<int> (float (scaling) * area / 1024.0f);

  // the integral image is larger:
  const int integralcols = imagecols + 1;

  // calculate borders
  const float x_1 = xf - sigma_half;
  const float x1  = xf + sigma_half;
  const float y_1 = yf - sigma_half;
  const float y1  = yf + sigma_half;

  const int x_left   = int (x_1 + 0.5);
  const int y_top    = int (y_1 + 0.5);
  const int x_right  = int (x1 + 0.5);
  const int y_bottom = int (y1 + 0.5);

  // overlap area - multiplication factors:
  const float r_x_1 = float (x_left) - x_1  + 0.5f;
  const float r_y_1 = float (y_top)  - y_1  + 0.5f;
  const float r_x1  = x1 - float (x_right)  + 0.5f;
  const float r_y1  = y1 - float (y_bottom) + 0.5f;
  const int dx = x_right  - x_left - 1;
  const int dy = y_bottom - y_top  - 1;
  const int A = static_cast<int> ((r_x_1 * r_y_1) * float (scaling));
  const int B = static_cast<int> ((r_x1  * r_y_1) * float (scaling));
  const int C = static_cast<int> ((r_x1  * r_y1)  * float (scaling));
  const int D = static_cast<int> ((r_x_1 * r_y1)  * float (scaling));
  const int r_x_1_i = static_cast<int> (r_x_1 * float (scaling));
  const int r_y_1_i = static_cast<int> (r_y_1 * float (scaling));
  const int r_x1_i  = static_cast<int> (r_x1  * float (scaling));
  const int r_y1_i  = static_cast<int> (r_y1  * float (scaling));

  if (dx + dy > 2)
  {
    // now the calculation:

    //+const unsigned char* ptr = static_cast<const unsigned char*> (&image[0].r) + x_left + imagecols * y_top;
    const unsigned char* ptr = static_cast<const unsigned char*>(&image[0]) + x_left + imagecols * y_top;

    // first the corners:
    ret_val = A * int (*ptr);

    //+ptr += (dx + 1) * sizeof (PointInT);
    ptr += dx + 1;

    ret_val += B * int (*ptr);

    //+ptr += (dy * imagecols + 1) * sizeof (PointInT);
    ptr += dy * imagecols + 1;

    ret_val += C * int (*ptr);

    //+ptr -= (dx + 1) * sizeof (PointInT);
    ptr -= dx + 1;

    ret_val += D * int (*ptr);

    // next the edges:
    //+int* ptr_integral;// = static_cast<int*> (integral.data) + x_left + integralcols * y_top + 1;
    const int* ptr_integral = static_cast<const int*> (&integral_image[0]) + x_left + integralcols * y_top + 1;

    // find a simple path through the different surface corners
    const int tmp1 = (*ptr_integral);
    ptr_integral += dx;
    const int tmp2 = (*ptr_integral);
    ptr_integral += integralcols;
    const int tmp3 = (*ptr_integral);
    ptr_integral++;
    const int tmp4 = (*ptr_integral);
    ptr_integral += dy * integralcols;
    const int tmp5 = (*ptr_integral);
    ptr_integral--;
    const int tmp6 = (*ptr_integral);
    ptr_integral += integralcols;
    const int tmp7 = (*ptr_integral);
    ptr_integral -= dx;
    const int tmp8 = (*ptr_integral);
    ptr_integral -= integralcols;
    const int tmp9 = (*ptr_integral);
    ptr_integral--;
    const int tmp10 = (*ptr_integral);
    ptr_integral -= dy * integralcols;
    const int tmp11 = (*ptr_integral);
    ptr_integral++;
    const int tmp12 = (*ptr_integral);

    // assign the weighted surface integrals:
    const int upper  = (tmp3 -tmp2  +tmp1  -tmp12) * r_y_1_i;
    const int middle = (tmp6 -tmp3  +tmp12 -tmp9)  * scaling;
    const int left   = (tmp9 -tmp12 +tmp11 -tmp10) * r_x_1_i;
    const int right  = (tmp5 -tmp4  +tmp3  -tmp6)  * r_x1_i;
    const int bottom = (tmp7 -tmp6  +tmp9  -tmp8)  * r_y1_i;

    return (ret_val + upper + middle + left + right + bottom + scaling2 / 2) / scaling2;
  }

  // now the calculation:

  //const unsigned char* ptr = static_cast<const unsigned char*> (&image[0].r) + x_left + imagecols * y_top;
  const unsigned char* ptr = static_cast<const unsigned char*>(&image[0]) + x_left + imagecols * y_top;

  // first row:
  ret_val = A * int (*ptr);

  //+ptr += sizeof (PointInT);
  ptr++;

  //+const unsigned char* end1 = ptr + (dx * sizeof (PointInT));
  const unsigned char* end1 = ptr + dx;

  //+for (; ptr < end1; ptr += sizeof (PointInT))
  for (; ptr < end1; ptr++)
    ret_val += r_y_1_i * int (*ptr);
  ret_val += B * int (*ptr);

  // middle ones:
  //+ptr += (imagecols - dx - 1) * sizeof (PointInT);
  ptr += imagecols - dx - 1;

  //+const unsigned char* end_j = ptr + (dy * imagecols) * sizeof (PointInT);
  const unsigned char* end_j = ptr + dy * imagecols;

  //+for (; ptr < end_j; ptr += (imagecols - dx - 1) * sizeof (PointInT))
  for (; ptr < end_j; ptr += imagecols - dx - 1)
  {
    ret_val += r_x_1_i * int (*ptr);

    //+ptr += sizeof (PointInT);
    ptr++;

    //+const unsigned char* end2 = ptr + (dx * sizeof (PointInT));
    const unsigned char* end2 = ptr + dx;

    //+for (; ptr < end2; ptr += sizeof (PointInT))
    for (; ptr < end2; ptr++)
      ret_val += int (*ptr) * scaling;

    ret_val += r_x1_i * int (*ptr);
  }
  // last row:
  ret_val += D * int (*ptr);

  //+ptr += sizeof (PointInT);
  ptr++;

  //+const unsigned char* end3 = ptr + (dx * sizeof (PointInT));
  const unsigned char* end3 = ptr + dx;

  //+for (; ptr<end3; ptr += sizeof (PointInT))
  for (; ptr<end3; ptr++)
    ret_val += r_y1_i * int (*ptr);

  ret_val += C * int (*ptr);

  return (ret_val + scaling2 / 2) / scaling2;
}


template <typename PointInT, typename PointOutT, typename KeypointT, typename IntensityT> bool
BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>::RoiPredicate (
    const float min_x, const float min_y,
    const float max_x, const float max_y, const KeypointT& pt)
{
  return ((pt.x < min_x) || (pt.x >= max_x) || (pt.y < min_y) || (pt.y >= max_y));
}


template <typename PointInT, typename PointOutT, typename KeypointT, typename IntensityT> void
BRISK2DEstimation<PointInT, PointOutT, KeypointT, IntensityT>::compute (
    PointCloudOutT &output)
{
  if (!input_cloud_->isOrganized ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] %s doesn't support non organized clouds!\n", name_.c_str ());
    return;
  }

  // image size
  const auto width = static_cast<index_t>(input_cloud_->width);
  const auto height = static_cast<index_t>(input_cloud_->height);

  // destination for intensity data; will be forwarded to BRISK
  std::vector<unsigned char> image_data (width*height);

  for (std::size_t i = 0; i < image_data.size (); ++i)
    image_data[i] = static_cast<unsigned char> (intensity_ ((*input_cloud_)[i]));

  // Remove keypoints very close to the border
  auto ksize = keypoints_->size ();
  std::vector<int> kscales; // remember the scale per keypoint
  kscales.resize (ksize);

  // initialize constants
  static const float lb_scalerange = std::log2 (scalerange_);

  auto beginning = keypoints_->points.begin ();
  auto beginningkscales = kscales.begin ();

  static const float basic_size_06 = basic_size_ * 0.6f;
  unsigned int basicscale = 0;

  if (!scale_invariance_enabled_)
    basicscale = std::max (static_cast<int> (float (scales_) / lb_scalerange * (std::log2 (1.45f * basic_size_ / (basic_size_06))) + 0.5f), 0);

  for (std::size_t k = 0; k < ksize; k++)
  {
    unsigned int scale;
    if (scale_invariance_enabled_)
    {
      scale = std::max (static_cast<int> (float (scales_) / lb_scalerange * (std::log2 ((*keypoints_)[k].size / (basic_size_06))) + 0.5f), 0);
      // saturate
      if (scale >= scales_) scale = scales_ - 1;
      kscales[k] = scale;
    }
    else
    {
      scale = basicscale;
      kscales[k] = scale;
    }

    const int border   = size_list_[scale];
    const int border_x = width - border;
    const int border_y = height - border;

    if (RoiPredicate (float (border), float (border), float (border_x), float (border_y), (*keypoints_)[k]))
    {
      //std::cerr << "remove keypoint" << std::endl;
      keypoints_->points.erase (beginning + k);
      kscales.erase (beginningkscales + k);
      if (k == 0)
      {
        beginning = keypoints_->points.begin ();
        beginningkscales = kscales.begin ();
      }
      ksize--;
      k--;
    }
  }

  keypoints_->width = keypoints_->size ();
  keypoints_->height = 1;

  // first, calculate the integral image over the whole image:
  // current integral image
  std::vector<int> integral ((width+1)*(height+1), 0);    // the integral image

  for (index_t row_index = 1; row_index < height; ++row_index)
  {
    for (index_t col_index = 1; col_index < width; ++col_index)
    {
      const std::size_t index = row_index*width+col_index;
      const std::size_t index2 = (row_index)*(width+1)+(col_index);

      integral[index2] = static_cast<int> (image_data[index])
        - integral[index2-1-(width+1)]
        + integral[index2-(width+1)]
        + integral[index2-1];
    }
  }

  int* values = new int[points_]; // for temporary use

  // resize the descriptors:
  //output = zeros (ksize, strings_);

  // now do the extraction for all keypoints:

  // temporary variables containing gray values at sample points:
  int t1;
  int t2;

  // the feature orientation
  int direction0;
  int direction1;

  output.resize (ksize);
  //output.width = ksize;
  //output.height = 1;
  for (std::size_t k = 0; k < ksize; k++)
  {
    unsigned char* ptr = &output[k].descriptor[0];

    int theta;
    KeypointT &kp    = (*keypoints_)[k];
    const int& scale = kscales[k];
    int shifter = 0;
    int* pvalues = values;
    const float& x = float (kp.x);
    const float& y = float (kp.y);
    if (true) // kp.angle==-1
    {
      if (!rotation_invariance_enabled_)
        // don't compute the gradient direction, just assign a rotation of 0 degree
        theta = 0;
      else
      {
        // get the gray values in the unrotated pattern
        for (unsigned int i = 0; i < points_; i++)
          *(pvalues++) = smoothedIntensity (image_data, width, height, integral, x, y, scale, 0, i);

        direction0 = 0;
        direction1 = 0;
        // now iterate through the long pairings
        const BriskLongPair* max = long_pairs_ + no_long_pairs_;

        for (BriskLongPair* iter = long_pairs_; iter < max; ++iter)
        {
          t1 = *(values + iter->i);
          t2 = *(values + iter->j);
          const int delta_t = (t1 - t2);

          // update the direction:
          const int tmp0 = delta_t * (iter->weighted_dx) / 1024;
          const int tmp1 = delta_t * (iter->weighted_dy) / 1024;
          direction0 += tmp0;
          direction1 += tmp1;
        }
        kp.angle = std::atan2 (float (direction1), float (direction0)) / float (M_PI) * 180.0f;
        theta = static_cast<int> ((float (n_rot_) * kp.angle) / (360.0f) + 0.5f);
        if (theta < 0)
          theta += n_rot_;
        if (theta >= int (n_rot_))
          theta -= n_rot_;
      }
    }
    else
    {
      // figure out the direction:
      //int theta=rotationInvariance*round((_n_rot*std::atan2(direction.at<int>(0,0),direction.at<int>(1,0)))/(2*M_PI));
      if (!rotation_invariance_enabled_)
        theta = 0;
      else
      {
        theta = static_cast<int> (n_rot_ * (kp.angle / (360.0)) + 0.5);
        if (theta < 0)
          theta += n_rot_;
        if (theta >= int (n_rot_))
          theta -= n_rot_;
      }
    }

    // now also extract the stuff for the actual direction:
    // let us compute the smoothed values
    shifter = 0;

    //unsigned int mean=0;
    pvalues = values;
    // get the gray values in the rotated pattern
    for (unsigned int i = 0; i < points_; i++)
      *(pvalues++) = smoothedIntensity (image_data, width, height, integral, x, y, scale, theta, i);

#ifdef __GNUC__
      using UINT32_ALIAS = std::uint32_t;
#endif
#ifdef _MSC_VER
      // Todo: find the equivalent to may_alias
      #define UCHAR_ALIAS std::uint32_t //__declspec(noalias)
      #define UINT32_ALIAS std::uint32_t //__declspec(noalias)
#endif

    // now iterate through all the pairings
    auto* ptr2 = reinterpret_cast<UINT32_ALIAS*> (ptr);
    const BriskShortPair* max = short_pairs_ + no_short_pairs_;

    for (BriskShortPair* iter = short_pairs_; iter < max; ++iter)
    {
      t1 = *(values + iter->i);
      t2 = *(values + iter->j);

      if (t1 > t2)
        *ptr2 |= ((1) << shifter);

      // else already initialized with zero
      // take care of the iterators:
      ++shifter;

      if (shifter == 32)
      {
        shifter = 0;
        ++ptr2;
      }
    }

    //ptr += strings_;

    //// Account for the scale + orientation;
    //ptr += sizeof (output[0].scale);
    //ptr += sizeof (output[0].orientation);
  }

  // we do not change the denseness
  output.width = output.size ();
  output.height = 1;
  output.is_dense = true;

  // clean-up
  delete [] values;
}

} // namespace pcl

#endif  //#ifndef PCL_FEATURES_IMPL_BRISK_2D_HPP_

