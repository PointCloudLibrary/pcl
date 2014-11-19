/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */

#ifndef PCL_KMEANS_H_
#define PCL_KMEANS_H_

#include <set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/common/io.h>

#include <pcl/pcl_base.h>

namespace pcl
{
  /** \brief @b K-means clustering.
    * \author Christian Potthast
    * \ingroup ML
    */
  //template <typename PointT>
  //class Kmeans : public PCLBase<PointT>
  class PCL_EXPORTS Kmeans
  {
/*
    typedef PCLBase<PointT> BasePCLBase;

    public:
      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;
*/

    public:

      typedef unsigned int PointId;    // the id of this point
      typedef unsigned int ClusterId;  // the id of this cluster


      //typedef std::vector<Coord> Point;    // a point (a centroid)

      typedef std::set<PointId> SetPoints; // set of points

      typedef std::vector<float> Point;

      // ClusterId -> (PointId, PointId, PointId, .... )
      typedef std::vector<SetPoints> ClustersToPoints;
      // PointId -> ClusterId
      typedef std::vector<ClusterId> PointsToClusters; 
      // coll of centroids
      typedef std::vector<Point> Centroids;


      /** \brief Empty constructor. */
      Kmeans (unsigned int num_points, unsigned int num_dimensions);

      /** \brief This destructor destroys
        * 
        */
      ~Kmeans ();

      /** \brief This method sets the k-means cluster size.
        * \param[in] k number of clusters
        */
      void
      setClusterSize (unsigned int k) {num_clusters_ = k;};

/*
      void
      setClusterField (std::string field_name) 
      {
        cluster_field_name_ = field_name;
      };
*/    

      //void
      //getClusterCentroids (PointT &out);

      //void
      //cluster (std::vector<PointIndices> &clusters);

      void
      kMeans ();
      
      void
      setInputData (std::vector<Point> &data)
      {
        if (num_points_ != data.size ())
          std::cout << "Data vector not the same" << std::endl;
        
        data_ = data;
      }

      void
      addDataPoint (Point &data_point)
      {
        if (num_dimensions_ != data_point.size ())
          std::cout << "Dimensions not the same" << std::endl;


        data_.push_back (data_point);
      }

    // Initial partition points among available clusters
    void initialClusterPoints();

      void 
      computeCentroids();

      // distance between two points
      float distance(const Point& x, const Point& y)
      {
        float total = 0.0;
        float diff;
    
        Point::const_iterator cpx=x.begin(); 
        Point::const_iterator cpy=y.begin();
        Point::const_iterator cpx_end=x.end();
        for(;cpx!=cpx_end;++cpx,++cpy){
          diff = *cpx - *cpy;
          total += (diff * diff); 
        }
        return total;  // no need to take sqrt, which is monotonic
      }


      Centroids get_centroids (){return centroids_;}


    protected:
      // Members derived from the base class
/*
      using BasePCLBase::input_;
      using BasePCLBase::indices_;
      using BasePCLBase::initCompute;
      using BasePCLBase::deinitCompute;
*/

      unsigned int num_points_;
      unsigned int num_dimensions_;
      

      /** \brief The number of clusters. */
      unsigned int num_clusters_;
      
      /** \brief The cluster centroids. */
      //std::vector

      //std::string cluster_field_name_;
      
      // one data point

      // all data points
      std::vector<Point> data_;

      ClustersToPoints clusters_to_points_;
      PointsToClusters points_to_clusters_;
      Centroids centroids_;

      
      

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
}

#endif
