#ifndef POINTCLOUD_DEPTH_NEIGHBOR_SEARCH_HPP
#define POINTCLOUD_DEPTH_NEIGHBOR_SEARCH_HPP

#ifndef PI
  #define PI 3.14159
#endif

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

      squared_radius = radius_arg*radius_arg;

      this->getProjectedRadiusSearchBox(p_q_arg, squared_radius, leftX, rightX, leftY, rightY);




      // iterate over all children
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
    void
    OrganizedNeighborSearch<PointT>::getProjectedRadiusSearchBox (const PointT& point_arg, double squared_radius_arg, int& minX_arg, int& maxX_arg, int& minY_arg, int& maxY_arg ) const
  {
    double r_sqr, r_quadr, z_sqr;
    double sqrt_term_y, sqrt_term_x, norm;
    double x_times_z, y_times_z;
    double x1, x2, y1, y2;

    // see http://www.wolframalpha.com/input/?i=solve+%5By%2Fsqrt%28f^2%2By^2%29*c-f%2Fsqrt%28f^2%2By^2%29*b%2Br%3D%3D0%2C+f%3D1%2C+y%5D
    // where b = p_q_arg.y, c = p_q_arg.z, r = radius_arg, f = focalLength_

    r_sqr = squared_radius_arg;
    r_quadr = r_sqr * r_sqr;
    z_sqr = point_arg.z * point_arg.z;

    sqrt_term_y = sqrt (point_arg.y * point_arg.y * r_sqr + z_sqr * r_sqr - r_quadr);
    sqrt_term_x = sqrt (point_arg.x * point_arg.x * r_sqr + z_sqr * r_sqr - r_quadr);
    norm = 1.0 / (z_sqr - r_sqr);

    x_times_z = point_arg.x * point_arg.z;
    y_times_z = point_arg.y * point_arg.z;

    y1 = (y_times_z - sqrt_term_y) * norm;
    y2 = (y_times_z + sqrt_term_y) * norm;
    x1 = (x_times_z - sqrt_term_x) * norm;
    x2 = (x_times_z + sqrt_term_x) * norm;

    // determine 2-D search window
    minX_arg  = (int)floor((double)input_->width / 2 + (x1 / focalLength_));
    maxX_arg = (int)ceil((double)input_->width / 2 + (x2 / focalLength_));

    minY_arg  = (int)floor((double)input_->height / 2 + (y1 / focalLength_));
    maxY_arg = (int)ceil((double)input_->height / 2 + (y2 / focalLength_));

    // make sure the coordinates fit to point cloud resolution
    minX_arg = std::max<int> (0, minX_arg);
    maxX_arg = std::min<int> ((int)input_->width - 1, maxX_arg);

    minY_arg = std::max<int> (0, minY_arg);
    maxY_arg = std::min<int> ((int)input_->height - 1, maxY_arg);
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

      return nearestKSearch (index_arg, k_arg, k_indices_arg, k_sqr_distances_arg);
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

      int maxSearchDistance;
      double squaredMaxSearchRadius;

      assert (k_arg>0);

      if (input_->height == 1)
      {
        PCL_ERROR ("[pcl::%s::nearestKSearch] Input dataset is not organized!", getName ().c_str ());
        return 0;
      }

      squaredMaxSearchRadius = max_distance_*max_distance_;

      // vector for nearest neighbor candidates
      std::vector<nearestNeighborCandidate> nearestNeighbors;

      // iterator for radius seach lookup table
      typename std::vector<radiusSearchLoopkupEntry>::const_iterator radiusSearchLookup_Iterator;
      radiusSearchLookup_Iterator = radiusSearchLookup_.begin ();

      nearestNeighbors.reserve (k_arg * 2);

      // project search point to plane
      pointPlaneProjection (p_q_arg, x_pos, y_pos);
      x_pos += (int)input_->width/2;
      y_pos += (int)input_->height/2;

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
              (point.y == point.y) &&
              (point.z == point.z))
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
        unsigned int pointCountRadiusSearch;
        unsigned int pointCountCircleSearch;

        squared_radius = std::min<double>(nearestNeighbors.back ().squared_distance_, squaredMaxSearchRadius);

        this->getProjectedRadiusSearchBox(p_q_arg, squared_radius, leftX, rightX, leftY, rightY);

        leftX *=leftX;
        rightX *= rightX;
        leftY *=leftY;
        rightY *= rightY;

        pointCountRadiusSearch = (rightX-leftX)*(rightY-leftY);

        // search for maximum distance between search point to window borders in 2-D search window
        maxSearchDistance = 0;
        maxSearchDistance = std::max<int> (maxSearchDistance, leftX + leftY);
        maxSearchDistance = std::max<int> (maxSearchDistance, leftX + rightY);
        maxSearchDistance = std::max<int> (maxSearchDistance, rightX + leftY);
        maxSearchDistance = std::max<int> (maxSearchDistance, rightX + rightY);

        maxSearchDistance +=1;
        maxSearchDistance *=maxSearchDistance;

        pointCountCircleSearch= (int)(PI*(double)(maxSearchDistance*maxSearchDistance));

        if (1){//(pointCountCircleSearch<pointCountRadiusSearch) {

          // check for nearest neighbors within window
          while ((radiusSearchLookup_Iterator != radiusSearchLookup_.end ())
              && ((*radiusSearchLookup_Iterator).squared_distance_ <= maxSearchDistance))
          {
            // select point from organized point cloud
            x = x_pos + (*radiusSearchLookup_Iterator).x_diff_;
            y = y_pos + (*radiusSearchLookup_Iterator).y_diff_;
            radiusSearchLookup_Iterator++;

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
                squared_distance = (point_dist_x * point_dist_x + point_dist_y * point_dist_y + point_dist_z
                    * point_dist_z);

                if ( squared_distance<=squared_radius )
                {
                  // add candidate
                  nearestNeighborCandidate newCandidate;
                  newCandidate.index_ = idx;
                  newCandidate.squared_distance_ = squared_distance;

                  nearestNeighbors.push_back (newCandidate);
                }
              }
            }
          }
        } else {
          std::vector<int> k_radius_indices;
          std::vector<float> k_radius_distances;

          nearestNeighbors.clear();

          k_radius_indices.reserve (k_arg*2);
          k_radius_distances.reserve (k_arg*2);

          radiusSearch (p_q_arg, sqrt(squared_radius),k_radius_indices , k_radius_distances);

          std::cout << k_radius_indices.size () <<std::endl;

          for (i = 0; i < k_radius_indices.size (); i++)
          {
            nearestNeighborCandidate newCandidate;
            newCandidate.index_ = k_radius_indices[i];
            newCandidate.squared_distance_ = k_radius_distances[i];

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
    OrganizedNeighborSearch<PointT>::estimateFocalLengthFromInputCloud ()
    {
      size_t i, count;
      int x, y;

      focalLength_ = 0;

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
              focalLength_ += point.x / ((double)(x - (int)input_->width / 2) * point.z);
              focalLength_ += point.y / ((double)(y - (int)input_->height / 2) * point.z);
              count += 2;
            }
          }
        }
      // calculate an average of the focalLength
      focalLength_ /= (double)count;

    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    void
    OrganizedNeighborSearch<PointT>::generateRadiusLookupTable (unsigned int width, unsigned int height)
    {
      int x, y, c;

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

}


#define PCL_INSTANTIATE_OrganizedNeighborSearch(T) template class pcl::OrganizedNeighborSearch<T>;

#endif
