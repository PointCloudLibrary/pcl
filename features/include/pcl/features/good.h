/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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


#ifndef PCL_FEATURES_GOOD_H_
#define PCL_FEATURES_GOOD_H_

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/feature.h>


namespace pcl
{
/** \brief GOOD: a Global Orthographic Object Descriptor for 3D object recognition and manipulation.
  * GOOD descriptor has been designed to be robust, descriptive and efficient to compute and use. 
  * It has two outstanding characteristics: 
  * 
  * (1) Providing a good trade-off among :
  *	- descriptiveness,
  *	- robustness,
  *	- computation time,
  *	- memory usage.
  * 
  * (2) Allowing concurrent object recognition and pose estimation for manipulation.
  * 
  * \note This is an implementation of the GOOD descriptor which has been presented in the following papers:
  * 
  *	[1] Kasaei, S. Hamidreza,  Ana Maria Tome, Luis Seabra Lopes, Miguel Oliveira 
  *	"GOOD: A global orthographic object descriptor for 3D object recognition and manipulation." 
  *	Pattern Recognition Letters 83 (2016): 312-320.http://dx.doi.org/10.1016/j.patrec.2016.07.006
  *
  *	[2] Kasaei, S. Hamidreza, Luis Seabra Lopes, Ana Maria Tome, Miguel Oliveira 
  * 	"An orthographic descriptor for 3D object learning and recognition." 
  *	2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, 
  *	pp. 4158-4163. doi: 10.1109/IROS.2016.7759612
  * 
  * Please adequately refer to this work any time this code is being used by citing above papers.
  * If you do publish a paper where GOOD descriptor helped your research, we encourage you to cite the above papers in your publications.
  * 
  * \author Hamidreza Kasaei (Seyed.Hamidreza[at]ua[dot]pt  Kasaei.Hamidreza[at]gmail[dot]com )
  */
  
  template <typename PointInT, int BinN>
  class PCL_EXPORTS GOODEstimation : public Feature<PointInT, Histogram<3*BinN*BinN> >
  {
    public:     
      typedef pcl::Histogram<3*BinN*BinN> Descriptor;     
      typedef typename pcl::PointCloud<PointInT> PointCloudIn; 
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudInPtr;      
      typedef typename Feature<PointInT, Descriptor>::PointCloudOut PointCloudOut;  
      
      using Feature<PointInT, Descriptor>::feature_name_;
      using Feature<PointInT, Descriptor>::k_;
      using PCLBase<PointInT>::input_;

      /** \brief Constructor.
        * \param[in] threshold_ threshold parameter is used in constructing local reference frame
        */
      GOODEstimation (const float threshold = 0.0015)
      : threshold_ (threshold)
      {
        feature_name_ = "GOODEstimation";
        k_ = 1;
      };

      /** \brief Returns the number_of_bins_ parameter. */
      inline static int 
      getNumberOfBins () { return BinN; }

      /** \brief Set the threshold param which is used for local reference frame construction 
        * \param[in] threshold threshold_  parameter
        */
      inline void
      setThreshold (const float threshold) 
      {
        threshold_ = threshold;
      }

      /** \brief Returns the threshold_ parameter. */
      inline float
      getThreshold () const {  return threshold_; }

      /** \brief get three orthographic projections of a set of points given by setInputCloud() 
        * \return the resultant vector of point clouds that contains three orthographic projections of the query point cloud
        */ 
      inline const std::vector<PointCloudInPtr>&
      getOrthographicProjections () const { return vector_of_projected_views_; }
      
      /** \brief get objec point cloud in local reference frame constructed by the GOOD descriptor 
        * \return the resultant point cloud of the object in local reference frame
        */ 
      inline const PointCloudInPtr&
      getTransformedObject () const { return transformed_point_cloud_; } 

      /** \brief get center of boundingbox of a set of points given by setInputCloud() in camera reference frame 
        * \return the resultant center of boundingbox
        */ 
      inline const pcl::PointXYZ&
      getCenterOfObjectBoundingBox () const { return center_of_bbox_; }

      /** \brief get dimensions of bounding box of a set of points given by setInputCloud()
        * \return the resultant boundingbox dimensions
        */ 
      inline const Eigen::Vector3f&
      getObjectBoundingBoxDimensions () const { return bbox_dimensions_; }

      /** \brief get the order of protection plans in constructing GOOD descriptor
        * \return the resultant of order of projections
        */  
      inline const std::string&
      getOrderOfProjectedPlanes () const { return order_of_projected_plane_str_; }
      

      /** \brief get the transformation matrix from camera reference frame to object local reference frame
        * \return the resultant transformation matrix
        */       
      inline const Eigen::Matrix4f&
      getTransformationMatrix () const {return transformation_;}
      
      /** \brief get the order of projection views programatically */
      enum Projection
      {
        XoY,
        XoZ,
        YoZ
       };
       Projection order_of_projected_plane_[3];
       
    protected:

      /** \brief Estimate the GOOD descriptor at a set of points given by setInputCloud() 
      * \param[out] output  the resultant GOOD descriptor representing the feature at the query point cloud
      */        
      virtual void
      computeFeature (PointCloudOut &output);
    

    private:
     
      enum Axis { X, Y, Z};
      
      /** \brief threshold parameter is used in constructing local reference frame. 
       * By default, the threshold_ is set to 0.0015.
       */
      float threshold_;

      /** \brief resultant of sign disambiguation can be either 1 or -1 */
      int8_t sign_;
            
      /** \brief transformed point cloud in LRF */
      PointCloudInPtr transformed_point_cloud_;  
      
      /** \brief get transformation matrix */
      Eigen::Matrix4f transformation_;
      
      /** \brief dimensions of boundingboxbox of given point cloud */
      Eigen::Vector3f bbox_dimensions_;
      
      /** \brief center of boundingboxbox of given point cloud */
      pcl::PointXYZ center_of_bbox_;
      
      /** \brief vector of three point clouds containing orthographic projection views */
      std::vector <PointCloudInPtr> vector_of_projected_views_;
      
      /** \brief get order of projection views in string format e.g. XoY-XoZ-YoZ */
      std::string order_of_projected_plane_str_;       

      /** \brief project point cloud to a plane
        * \param[in] pc_in pointer to a point cloud.
        * \param[in] coefficients pcl::ModelCoefficients
        * \param[out] pc_out the resultant projected point cloud
        */ 
      void
      projectPointCloudToPlane (const PointCloudInConstPtr &pc_in, const pcl::ModelCoefficients::Ptr &coefficients, PointCloudIn &pc_out); 

      /** \brief convert 2D histogram to 1D histogram
        * \param[in] histogram_2D a 2D vector of unsigned int 
        * \return the resultant 1D vector of unsigned int 
        */ 
      static std::vector<unsigned>
      convert2DHistogramTo1DHistogram (const std::vector<std::vector<unsigned> > &histogram_2D);
            
      /** \brief sign disambiguation
        * \param[in] projected_view pointer to a point cloud.
        * \param[in] threshold used to deal with the special case when a point is close to the other planes
        * \return the resultant sign (either 1 or -1) 
        */            
      int8_t
      signDisambiguation (const PointCloudIn &projected_view, const int8_t axis) const;

      /** \brief create a 2D histogram from a projection
        * \param[in] projected_view pointer to a point cloud.
        * \param[in] largest_side largest side of object bounding box.
        * \param[in] number_of_bins  number of bins along one dimension.
        * \param[in] sign either 1 or -1.
        * \param[out] histogram a 2D vector of unsigned int.
        */  
      void 
      create2DHistogramFromProjection (const PointCloudInPtr &projected_view, const float largest_side, 
                                       const int8_t axis_a, const int8_t axis_b,
                                       std::vector<std::vector<unsigned int> > &histogram) const;
             
      /** \brief normalizing a 1D histogram
        * \param[in] histogram 1D histogram (int).
        * \param[out] normalized_histogram the resultant normalized_histogram (float).
        */       
      static void
      normalizeHistogram (const std::vector <unsigned int> histogram, std::vector <float> &normalized_histogram);

      /** \brief compute viewpoint entropy used for concatenating projections
        * \param[in] normalized_histogram normalized_histogram (float).
        * \return the resultant view entropy.
        */   
      float
      viewpointEntropy (const std::vector<float> &normalized_histogram);

      /** \brief find max view point entropy used for concatenating projections
        * \param[in] view_point_entropy a vector of float contains three view entropies.
        * \return the resultant projection index.
        */  
      int
      findMaxViewPointEntropy (const std::vector<float> &view_point_entropy);

      /** \brief compute variance of a given histogram
        * \param[in] histogram normalized histogram of projection.
        * \return variance the resultant variance.
        */        
      float
      varianceOfHistogram (const std::vector<float> &histogram);

      /** \brief concatinating three orthographic projections
       * three histograms are obtained for the projections; afterwards, two statistic features including entropy and variance have been calculated for each distribution vector;
       * the histograms are consequently concatenated together using entropy and variance features, to form a single description for the given object. The ordering of the three
       * histograms is first by decreasing values of entropy. Afterwards the second and third vectors are sorted again by increasing values of variance.
       * \param[in] maximum_entropy_index index of orthographic projection that has maximum entropy.		
       * \param[in] normalized_projected_views a vector of vector of float contains three normalized histogram of projected views
       * \param[out] sorted_normalized_projected_views a vector of float representing the GOOD description for a set of points given by setInputCloud() 
       * \param[out] name_of_sorted_projected_plane an string represents the order of concatenating projections.
       */              
      void
      objectViewHistogram (int maximum_entropy_index, 
                           const std::vector<std::vector<float> > &normalized_projected_views,
                           std::vector<float> &sorted_normalized_projected_views,
                           std::string &name_of_sorted_projected_plane);

      /** \brief compute largest side of the computed boundingbox i.e. bbox_dimensions_
        * \return the resultant largest side.
        * It should be noted that a small value (i.e. 0.02) is added to the output to deal with the special cases, 
        * when a point is projected onto the upper bound of the projection area
        */           
      inline float 
      computeLargestSideOfBoundingBox () const { return (bbox_dimensions_.maxCoeff() + 0.02) ;}      
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/good.hpp>
#endif

#endif /* PCL_FEATURES_GOOD_H_ */
