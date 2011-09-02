
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
 * Author: Julius Kammerl (julius@kammerl.de), Nico Blodow (blodow@cs.tum.edu)
 */


#ifndef PCL_SEARCH_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_H_
#define PCL_SEARCH_SEARCH_IMPL_ORGANIZED_NEIGHBOR_SEARCH_H_

#include "math.h"
#include "pcl/search/organized_neighbor_search.h"
namespace pcl
{

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    int
    OrganizedNeighborSearch<PointT>::radiusSearch (const PointCloudConstPtr &cloud_arg, int index_arg,
                                                   double radius_arg, std::vector<int> &k_indices_arg,
                                                   std::vector<float> &k_sqr_distances_arg, int max_nn_arg) 
    {

      this->setInputCloud (cloud_arg);
        return radiusSearch (index_arg, radius_arg, k_indices_arg, k_sqr_distances_arg, max_nn_arg);

    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    int
    OrganizedNeighborSearch<PointT>::radiusSearch (int index_arg, const double radius_arg,
                                                   std::vector<int> &k_indices_arg,
                                                   std::vector<float> &k_sqr_distances_arg, int max_nn_arg) const
    {

      const PointT searchPoint = getPointByIndex (index_arg);
       
      return radiusSearch (searchPoint, radius_arg, k_indices_arg, k_sqr_distances_arg, max_nn_arg);

    }

  //////////////////////////////////////////////////////////////////////////////////////////////


  template<typename PointT>
    int
    OrganizedNeighborSearch<PointT>::radiusSearch (const PointT &p_q_arg, const double radius_arg,
                                                   std::vector<int> &k_indices_arg,
                                                   std::vector<float> &k_sqr_distances_arg, int max_nn_arg) const
    {

      if (input_->height == 1)
      {
        PCL_ERROR ("[pcl::%s::radiusSearch] Input dataset is not organized!", getName ().c_str ());
        return 0;
      }
      // search window
      int leftX, rightX, leftY, rightY;
      int x, y, idx;
      double squared_distance, squared_radius;
      int nnn;

      k_indices_arg.clear ();
      k_sqr_distances_arg.clear ();

      squared_radius = radius_arg * radius_arg;
   
      this->getProjectedRadiusSearchBox (p_q_arg, squared_radius, leftX, rightX, leftY, rightY);

      // iterate over search box
      nnn = 0;
      for (x = leftX; (x <= rightX) && (nnn < max_nn_arg); x++)
        for (y = leftY; (y <= rightY) && (nnn < max_nn_arg); y++)
        {
          idx = y * input_->width + x;
          const PointT& point = input_->points[idx];

          const double point_dist_x = point.x - p_q_arg.x;
          const double point_dist_y = point.y - p_q_arg.y;
          const double point_dist_z = point.z - p_q_arg.z;

          // calculate squared distance
          squared_distance = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z * point_dist_z);

          // check distance and add to results
          if (squared_distance <= squared_radius)
          {
            k_indices_arg.push_back (idx);
            k_sqr_distances_arg.push_back (squared_distance);
            nnn++;
          }
        }

      return nnn;
    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    int
    OrganizedNeighborSearch<PointT>::nearestKSearch (int index_arg, int k_arg, std::vector<int> &k_indices_arg,
                                                     std::vector<float> &k_sqr_distances_arg) 
    {

      const PointT searchPoint = getPointByIndex (index_arg);
      return nearestKSearch (searchPoint, k_arg, k_indices_arg, k_sqr_distances_arg);
    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    int
    OrganizedNeighborSearch<PointT>::nearestKSearch (const PointCloudConstPtr &cloud_arg, int index_arg, int k_arg,
                                                     std::vector<int> &k_indices_arg,
                                                     std::vector<float> &k_sqr_distances_arg)  
    {
      this->setInputCloud (cloud_arg);
      return (nearestKSearch (index_arg, k_arg, k_indices_arg, k_sqr_distances_arg));
    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    int
    OrganizedNeighborSearch<PointT>::nearestKSearch (const PointT &p_q_arg, int k_arg, std::vector<int> &k_indices_arg,
                                                     std::vector<float> &k_sqr_distances_arg) 
    {
      int x_pos, y_pos, x, y, idx;
      std::size_t i;
      int leftX, rightX, leftY, rightY;
      int radiusSearchPointCount;

      double squaredMaxSearchRadius;

      assert (k_arg > 0);

      if (input_->height == 1)
      {
        PCL_ERROR ("[pcl::%s::nearestKSearch] Input dataset is not organized!", getName ().c_str ());
        return 0;
      }

      squaredMaxSearchRadius = max_distance_ * max_distance_;

      // vector for nearest neighbor candidates
      std::vector<nearestNeighborCandidate> nearestNeighbors;

      // iterator for radius seach lookup table
      typename std::vector<radiusSearchLoopkupEntry>::const_iterator radiusSearchLookup_Iterator;
      radiusSearchLookup_Iterator = radiusSearchLookup_.begin ();

      nearestNeighbors.reserve (k_arg * 2);

      // project search point to plane
      pointPlaneProjection (p_q_arg, x_pos, y_pos);
      x_pos += (int)input_->width / 2;
      y_pos += (int)input_->height / 2;

      // initialize result vectors
      k_indices_arg.clear ();
      k_sqr_distances_arg.clear ();

      radiusSearchPointCount = 0;
      // search for k_arg nearest neighbor candidates using the radius lookup table
      while ((radiusSearchLookup_Iterator != radiusSearchLookup_.end ()) && ((int)nearestNeighbors.size () < k_arg))
      {
        // select point from organized pointcloud
        x = x_pos + (*radiusSearchLookup_Iterator).x_diff_;
        y = y_pos + (*radiusSearchLookup_Iterator).y_diff_;

        radiusSearchLookup_Iterator++;
        radiusSearchPointCount++;

        if ((x >= 0) && (y >= 0) && (x < (int)input_->width) && (y < (int)input_->height))
        {
          idx = y * (int)input_->width + x;
          const PointT& point = input_->points[idx];

          if ((point.x == point.x) && // check for NaNs
              (point.y == point.y) && (point.z == point.z))
          {
            double squared_distance;

            const double point_dist_x = point.x - p_q_arg.x;
            const double point_dist_y = point.y - p_q_arg.y;
            const double point_dist_z = point.z - p_q_arg.z;

            // calculate squared distance
            squared_distance
                = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z * point_dist_z);

            if (squared_distance <= squaredMaxSearchRadius)
            {
              // we have a new candidate -> add it
              nearestNeighborCandidate newCandidate;
              newCandidate.index_ = idx;
              newCandidate.squared_distance_ = squared_distance;

              nearestNeighbors.push_back (newCandidate);
            }

          }
        }
      }

      // sort candidate list
      std::sort (nearestNeighbors.begin (), nearestNeighbors.end ());

      // we found k_arg candidates -> do radius search
      if ((int)nearestNeighbors.size () == k_arg)
      {
        double squared_radius;

        squared_radius = std::min<double> (nearestNeighbors.back ().squared_distance_, squaredMaxSearchRadius);

        std::vector<int> k_radius_indices;
        std::vector<float> k_radius_distances;

        nearestNeighbors.clear ();

        this->getProjectedRadiusSearchBox (p_q_arg, squared_radius, leftX, rightX, leftY, rightY);

        // iterate over all children
        for (x = leftX; x <= rightX; x++)
          for (y = leftY; y <= rightY; y++)
          {
            double squared_distance;
            idx = y * input_->width + x;
            const PointT& point = input_->points[idx];

            const double point_dist_x = point.x - p_q_arg.x;
            const double point_dist_y = point.y - p_q_arg.y;
            const double point_dist_z = point.z - p_q_arg.z;

            // calculate squared distance
            squared_distance
                = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z * point_dist_z);

            // check distance and add to results
            if (squared_distance <= squared_radius)
            {
              nearestNeighborCandidate newCandidate;
              newCandidate.index_ = idx;
              newCandidate.squared_distance_ = squared_distance;

              nearestNeighbors.push_back (newCandidate);
            }
          }

        std::sort (nearestNeighbors.begin (), nearestNeighbors.end ());

        // truncate sorted nearest neighbor vector if we found more than k_arg candidates
        if (nearestNeighbors.size () > (size_t)k_arg)
        {
          nearestNeighbors.resize (k_arg);
        }

      }

      // copy results from nearestNeighbors vector to separate indices and distance vector
      k_indices_arg.resize (nearestNeighbors.size ());
      k_sqr_distances_arg.resize (nearestNeighbors.size ());

      for (i = 0; i < nearestNeighbors.size (); i++)
      {
        k_indices_arg[i] = nearestNeighbors[i].index_;
        k_sqr_distances_arg[i] = nearestNeighbors[i].squared_distance_;
      }

      return k_indices_arg.size ();

    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    void
    OrganizedNeighborSearch<PointT>::getProjectedRadiusSearchBox (const PointT& point_arg, double squared_radius_arg,
                                                                  int& minX_arg, int& maxX_arg, int& minY_arg,
                                                                  int& maxY_arg) const
    {
      double r_sqr, r_quadr, z_sqr;
      double sqrt_term_y, sqrt_term_x, norm;
      double x_times_z, y_times_z;
      double x1, x2, y1, y2;
      double term_x, term_y;

      // see http://www.wolframalpha.com/input/?i=solve+%5By%2Fsqrt%28f^2%2By^2%29*c-f%2Fsqrt%28f^2%2By^2%29*b%2Br%3D%3D0%2C+f%3D1%2C+y%5D
      // where b = p_q_arg.y, c = p_q_arg.z, r = radius_arg, f = oneOverFocalLength_

      r_sqr = squared_radius_arg;
      r_quadr = r_sqr * r_sqr;
      z_sqr = point_arg.z * point_arg.z;
      norm = 1.0 / (z_sqr - r_sqr);

      // radius sphere projection on X axis
      term_x = point_arg.x * point_arg.x * r_sqr + z_sqr * r_sqr - r_quadr;

      // radius sphere projection on Y axis
      term_y = point_arg.y * point_arg.y * r_sqr + z_sqr * r_sqr - r_quadr;

      if ((term_x > 0) && (term_y > 0))
      {
        sqrt_term_x = sqrt (term_x);

        x_times_z = point_arg.x * point_arg.z;

        x1 = (x_times_z - sqrt_term_x) * norm;
        x2 = (x_times_z + sqrt_term_x) * norm;

        // determine 2-D search window
        minX_arg = (int)floor ((double)input_->width / 2 + (x1 / oneOverFocalLength_));
        maxX_arg = (int)ceil ((double)input_->width / 2 + (x2 / oneOverFocalLength_));

        // make sure the coordinates fit to point cloud resolution
        minX_arg = std::max<int> (0, minX_arg);
        maxX_arg = std::min<int> ((int)input_->width - 1, maxX_arg);

        sqrt_term_y = sqrt (term_y);

        y_times_z = point_arg.y * point_arg.z;

        y1 = (y_times_z - sqrt_term_y) * norm;
        y2 = (y_times_z + sqrt_term_y) * norm;

        // determine 2-D search window
        minY_arg = (int)floor ((double)input_->height / 2 + (y1 / oneOverFocalLength_));
        maxY_arg = (int)ceil ((double)input_->height / 2 + (y2 / oneOverFocalLength_));

        // make sure the coordinates fit to point cloud resolution
        minY_arg = std::max<int> (0, minY_arg);
        maxY_arg = std::min<int> ((int)input_->height - 1, maxY_arg);
      }
      else
      {
        // search point lies within search sphere
        minX_arg = 0;
        maxX_arg = (int)input_->width - 1;

        minY_arg = 0;
        maxY_arg = (int)input_->height - 1;
      }

    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    void
    OrganizedNeighborSearch<PointT>::estimateFocalLengthFromInputCloud ()
    {
      size_t i, count;
      int x, y;

      oneOverFocalLength_ = 0;

      count = 0;
      for (y = 0; y < (int)input_->height; y++)
        for (x = 0; x < (int)input_->width; x++)
        {
          i = y * input_->width + x;
          if ((input_->points[i].x == input_->points[i].x) && // check for NaNs
              (input_->points[i].y == input_->points[i].y) && (input_->points[i].z == input_->points[i].z))
          {
            const PointT& point = input_->points[i];
            if ((double)(x - input_->width / 2) * (double)(y - input_->height / 2) * point.z != 0)
            {
              // estimate the focal length for point.x and point.y
              oneOverFocalLength_ += point.x / ((double)(x - (int)input_->width / 2) * point.z);
              oneOverFocalLength_ += point.y / ((double)(y - (int)input_->height / 2) * point.z);
              count += 2;
            }
          }
        }
      // calculate an average of the focalLength
      oneOverFocalLength_ /= (double)count;

    }

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
OrganizedNeighborSearch<PointT>::generateRadiusLookupTable (unsigned int width, unsigned int height)
{
  int x, y, c;

  //check if point cloud dimensions changed
  if ( (this->radiusLookupTableWidth_!=(int)width) || (this->radiusLookupTableHeight_!=(int)height) )
  {

    this->radiusLookupTableWidth_ = (int)width;
    this->radiusLookupTableHeight_= (int)height;

    radiusSearchLookup_.clear ();
    radiusSearchLookup_.resize ((2*width+1) * (2*height+1));

    c = 0;
    for (x = -(int)width; x < (int)width+1; x++)
    for (y = -(int)height; y <(int)height+1; y++)
    {
      radiusSearchLookup_[c++].defineShiftedSearchPoint(x, y);
    }

    std::sort (radiusSearchLookup_.begin (), radiusSearchLookup_.end ());
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
const PointT&
OrganizedNeighborSearch<PointT>::getPointByIndex (const unsigned int index_arg) const
{
  // retrieve point from input cloud
  assert (index_arg < (unsigned int)input_->points.size ());
  return this->input_->points[index_arg];

}


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
OrganizedNeighborSearch<PointT>::approxRadiusSearch (
    const PointCloudConstPtr &cloud, int index, double radius, std::vector<int> &k_indices, 
    std::vector<float> &k_distances, int max_nn) const
{

  k_indices.clear ();
  k_distances.clear ();

  if (cloud->height == 1)
  {
    PCL_ERROR ("[pcl::%s::nearestKSearch] Input dataset is not organized!\n", getName ().c_str ());
    return 0;
  }
  int data_size = cloud->points.size ();
  if (index >= data_size)
    return 0;

  // Get the cloud's dimensions width
  int width = cloud->width,
      height = cloud->height;

  int y=index/width, x=index-y*width;
  
  const std::vector<PointT, Eigen::aligned_allocator<PointT> >& points = cloud->points;
  const PointT& point = points[index];
  if (!pcl_isfinite(point.x))
    return 0;
  
  // Put point itself into results
  k_indices.push_back(index);
  k_distances.push_back(0.0f);
  
  float max_dist_squared = radius*radius;
  bool still_in_range = true,
       done = false;
  for (int radius=1;  !done;  ++radius) 
  {
    int x2=x-radius-1, y2=y-radius;  // Top left - 1
    still_in_range = false;
    for (int i=0; i<8*radius; ++i)
    {
      if (i<=2*radius) ++x2; else if (i<=4*radius) ++y2; else if (i<=6*radius) --x2; else --y2;
      if (x2<0 || x2>=width || y2<0 || y2>=height)
        continue;
      int neighbor_index = y2*width + x2;
      const PointT& neighbor = points[neighbor_index];
      if (!pcl_isfinite(neighbor.x))
        continue;
      float distance_squared = squaredEuclideanDistance(point, neighbor);
      if (distance_squared > max_dist_squared)
        continue;
      //cout << "Radius "<<radius<<": found "<<neighbor_index << " with distance "<<sqrtf(distance_squared)<<"\n";
      still_in_range = true;
      k_indices.push_back(neighbor_index);
      k_distances.push_back(distance_squared);
      if ((int)k_indices.size() >= max_nn)
      {
        done = true;
        break;
      }
    }
    if (!still_in_range)
      done = true;
  }
  
  return (int(k_indices.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int 
OrganizedNeighborSearch<PointT>::approxNearestKSearch (const PointCloudConstPtr &cloud, int index, int k, 
    std::vector<int> &k_indices, std::vector<float> &k_distances)
{
  k_indices.resize (k);
  if (cloud->height == 1)
  {
    PCL_ERROR ("[pcl::%s::nearestKSearch] Input dataset is not dense!\n", getName ().c_str ());
    return 0;
  }
  int data_size = cloud->points.size ();
  if (index >= data_size)
    return 0;

  // Get the cloud width
  int width = cloud->width;

  // Obtain the <u,v> pixel values
  int u = index / width;
  int v = index % width;

  int l = -1, idx, uwv = u * width + v, uwvx;

  // Save the query point as the first neighbor (*ANN compatibility)
  k_indices[++l] = index;

  if (horizontal_window_==0 || vertical_window_)
    setSearchWindowAsK (k);

  // Get all point neighbors in a H x V window

std::cout << horizontal_window_ << '\t' << vertical_window_ << '\t' << k << std::endl;
  for (int x = -horizontal_window_; x != horizontal_window_; ++x)
  {
    uwvx = uwv + x * width;     // Get the correct index

    for (int y = -vertical_window_; y != vertical_window_; ++y)
    {
      // idx = (u+x) * cloud.width + (v+y);
      idx = uwvx + y;

      // If the index is not in the point cloud, continue
      if (idx == index || idx < 0 || idx >= data_size)
        continue;

      if (max_distance_ != 0)
      {
        if (fabs (cloud->points[index].z - cloud->points[idx].z) < max_distance_)
          k_indices[++l] = idx;
      }
      else
        k_indices[++l] = idx;
    }
  }
  // We need at least min_pts_ nearest neighbors to do something useful with them
  if (l < min_pts_)
    return 0;
  return k;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
OrganizedNeighborSearch<PointT>::setSearchWindowAsK (int k)
{
  int hw = 0, vw = 0;
  while ( (2 * hw + 1 ) * (2 * vw + 1) < k)
  {
    ++hw; ++vw;
  }
  horizontal_window_ = hw - 1;
  vertical_window_ = vw - 1;

//std::cout << horizontal_window_ <<std::endl;
//getchar();

}


}

#define PCL_INSTANTIATE_OrganizedNeighborSearch(T) template class PCL_EXPORTS pcl::OrganizedNeighborSearch<T>;

#endif
