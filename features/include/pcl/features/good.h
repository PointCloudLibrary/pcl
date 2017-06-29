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
  *	[1] Kasaei, S. Hamidreza,  Ana Maria Tomé, Luís Seabra Lopes, Miguel Oliveira 
  *	"GOOD: A global orthographic object descriptor for 3D object recognition and manipulation." 
  *	Pattern Recognition Letters 83 (2016): 312-320.http://dx.doi.org/10.1016/j.patrec.2016.07.006
  *
  *	[2] Kasaei, S. Hamidreza, Luís Seabra Lopes, Ana Maria Tomé, Miguel Oliveira 
  * 	"An orthographic descriptor for 3D object learning and recognition." 
  *	2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, 
  *	pp. 4158-4163. doi: 10.1109/IROS.2016.7759612
  * 
  * Please adequately refer to this work any time this code is being used by citing above papers.
  * If you do publish a paper where GOOD descriptor helped your research, we encourage you to cite the above papers in your publications.
  * 
  * \author Hamidreza Kasaei (Seyed.Hamidreza[at]ua[dot]pt  Kasaei.Hamidreza[at]gmail[dot]com )
  */
  
  template <typename PointInT>
  class PCL_EXPORTS GOODEstimation //: public Feature<PointInT, PointOutT>
  {
    public:     

      typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudIn;
      //using PCLBase<PointInT>::input_;	  
      
      /** \brief Constructor.
	* \param[in] number_of_bins_ number of bins along one dimension; each projection plane is divided into number_of_bins × number_of_bins square bins.
	* The three projection vectors will be concatenated producing a vector of dimension 3 × number_of_bins^2 which is the final object descriptor, GOOD. 
	* \param[in] threshold_ threshold parameter is used in constructing local reference frame
	*/     
      GOODEstimation (const unsigned int number_of_bins = 5, const float threshold = 0.0015)
      {
	number_of_bins_ = number_of_bins;
	threshold_ = threshold;
      };
      
           
      /** \brief Set the input cloud.
	* \param[in] cloud pointer to a point cloud.
	* \note: I will remove this function, whenever I can drive it from PCLBase and Feature
	*/      
      void
      setInputCloud (PointCloudIn cloud);  

      /** \brief Sets GOOD descriptor resolution.
	* \param[in] number_of_bins number of bins along one dimension; each projection plane is divided into number_of_bins × number_of_bins square bins.
	* The three projection vectors will be concatenated producing a vector of dimension 3 × number_of_bins^2 which is the final object descriptor, GOOD. 
	*/      
      inline void
      setNumberOfBins (const unsigned int number_of_bins) 
      {
	number_of_bins_ = number_of_bins;
      }

      /** \brief Returns the number_of_bins_ parameter. */
      inline unsigned int
      getNumberOfBins () 
      { 
	return number_of_bins_; 	
      }
           
      /** \brief Set the threshold param which is used for local reference frame construction 
       * \param[in] threshold threshold_  parameter
       */
      inline void
      setThreshold (const float threshold) 
      {
	threshold_ = threshold;
      }

      /** \brief Returns the number_of_bins_ parameter. */
      inline float
      getThreshold () 
      { 
	return threshold_; 	
      }
              
      /** \brief get three orthographic projections of a set of points given by setInputCloud() 
        * \param[out] vector_of_projected_views the resultant vector of point clouds that contains three orthographic projections of the query point cloud
        */ 
      inline void
      getOrthographicProjections (std::vector<PointCloudIn> &vector_of_projected_views) const
      {
	vector_of_projected_views = vector_of_projected_views_;  
      }

      /** \brief get objec point cloud in local reference frame constructed by the GOOD descriptor 
        * \param[out] transformed_point_cloud the resultant point cloud of the object in local reference frame
        */ 
      inline void
      getTransformedObject (PointCloudIn &transformed_point_cloud) const
      {
	transformed_point_cloud = transformed_point_cloud_;
      } 

      /** \brief get center of boundingbox of a set of points given by setInputCloud() in camera reference frame 
        * \param[out] center_of_bbox the resultant center of boundingbox
        */ 
      inline void
      getCenterOfObjectBoundingBox (pcl::PointXYZ &center_of_bbox) const
      {
	center_of_bbox = center_of_bbox_;
      }

      /** \brief get dimensions of bounding box of a set of points given by setInputCloud()
        * \param[out] bbox_dimensions the resultant boundingbox dimensions
        */ 
      inline void
      getObjectBoundingBoxDimensions(pcl::PointXYZ &bbox_dimensions) const
      {
	bbox_dimensions = bbox_dimensions_;
      }

      /** \brief get the order of protection plans in constructing GOOD descriptor
        * \param[out] order_of_projected_plane the resultant of order of projections
        */  
      inline void
      getOrderOfProjectedPlanes (std::string &order_of_projected_plane) const
      {
	order_of_projected_plane = order_of_projected_plane_;
      }

      /** \brief get the transformation matrix from camera reference frame to object local reference frame
        * \param[out] transformation the resultant transformation matrix
        */       
      inline void
      getTransformationMatrix (Eigen::Matrix4f &transformation) const
      {
	transformation = transformation_;
      } 
      
      /** \brief Estimate the GOOD descriptor at a set of points given by setInputCloud() 
        * \param[out] object_description  the resultant GOOD descriptor representing the feature at the query point cloud
        */
      void
      compute (std::vector<float> &object_description );
       
    private:

      /** \brief number of bins along one dimension; each projection plane is divided into number_of_bins × number_of_bins square bins. 
       * By default, the number_of_bins_ is set to 5.
       */
      unsigned int number_of_bins_; 
      
      /** \brief threshold parameter is used in constructing local reference frame. 
       * By default, the number_of_bins_ is set to 0.0015.
       */
      float threshold_;

      /** \brief resultant of sign disambiguation can be either 1 or -1 */
      int sign_;
      
      /** \brief given point cloud*/
      PointCloudIn input_ ;
      
      /** \brief transformed point cloud in LRF */
      PointCloudIn transformed_point_cloud_;  
      
      /** \brief get transformation matrix */
      Eigen::Matrix4f transformation_;
      
      /** \brief dimensions of boundingboxbox of given point cloud */
      pcl::PointXYZ bbox_dimensions_;
      
      /** \brief center of boundingboxbox of given point cloud */
      pcl::PointXYZ center_of_bbox_;
      
      /** \brief vector of three point clouds containing orthographic projection views */
      std::vector < PointCloudIn > vector_of_projected_views_;
      
      /** \brief get order of projection views e.g. XoY-XoZ-YoZ */
      std::string order_of_projected_plane_;    

      
      /** \brief get dimensions of bounding box of a given point cloud
 	* \param[in] pc pointer to a point cloud.
	* \param[out] dimensions the resultant boundingbox dimensions
        */ 
      void 
      computeBoundingBoxDimensions (PointCloudIn pc, pcl::PointXYZ &dimensions);

      /** \brief project point cloud to a plane
 	* \param[in] pc_in pointer to a point cloud.
	* \param[in] coefficients pcl::ModelCoefficients
	* \param[out] pc_out the resultant projected point cloud
        */ 
      void
      projectPointCloudToPlane (PointCloudIn pc_in, pcl::ModelCoefficients::Ptr coefficients, PointCloudIn pc_out);

      /** \brief convert 2D histogram to 1D histogram
 	* \param[in] histogram_2D a 2D vector of unsigned int 
	* \param[out] histogram the resultant 1D vector of unsigned int 
        */ 
      void 
      convert2DHistogramTo1DHistogram (std::vector<std::vector<unsigned int> > histogram_2D, std::vector<unsigned int>  &histogram);

      /** \brief sign disambiguation of axis X
 	* \param[in] XoZ_projected_view pointer to a point cloud.
	* \param[in] threshold used to deal with the special case when a point is close to the YoZ plane
	* \param[out] sign the resultant sign (either 1 or -1) 
        */       
      void
      signDisambiguationXAxis (PointCloudIn  XoZ_projected_view, float threshold, int &sign );
      
      /** \brief sign disambiguation of axis Y
 	* \param[in] YoZ_projected_view pointer to a point cloud.
	* \param[in] threshold used to deal with the special case when a point is close to the XoZ plane
	* \param[out] sign the resultant sign (either 1 or -1) 
        */            
      void 
      signDisambiguationYAxis (PointCloudIn  YoZ_projected_view, float threshold, int &sign );

      /** \brief create a 2D histogram from YOZ projection
 	* \param[in] YoZ_projected_view pointer to a point cloud.
	* \param[in] largest_side largest side of object bounding box
	* \param[in] number_of_bins  number of bins along one dimension;
	* \param[in] sign either 1 or -1
	* \param[out] YOZ_histogram a 2D vector of unsigned int
        */            
      void
      create2DHistogramFromYOZProjection (PointCloudIn  YOZ_projected_view, double largest_side, unsigned int number_of_bins, 
					   int sign, std::vector<std::vector<unsigned int> > &YOZ_histogram);

      /** \brief create a 2D histogram from XOZ projection
 	* \param[in] XoZ_projected_view pointer to a point cloud.
	* \param[in] largest_side largest side of object bounding box.
	* \param[in] number_of_bins  number of bins along one dimension.
	* \param[in] sign either 1 or -1.
	* \param[out] XOZ_histogram a 2D vector of unsigned int.
        */            
      void 
      create2DHistogramFromXOZProjection ( PointCloudIn  XOZ_projected_view, double largest_side, 
					    unsigned int number_of_bins, int sign, std::vector < std::vector<unsigned int> > &XOZ_histogram);
      
      /** \brief create a 2D histogram from XOY projection
 	* \param[in] XoY_projected_view pointer to a point cloud.
	* \param[in] largest_side largest side of object bounding box.
	* \param[in] number_of_bins  number of bins along one dimension.
	* \param[in] sign either 1 or -1.
	* \param[out] XOY_histogram a 2D vector of unsigned int.
        */  
      void 
      create2DHistogramFromXOYProjection (PointCloudIn  XOY_projected_view,	double largest_side,
					unsigned int number_of_bins, int sign, std::vector < std::vector<unsigned int> > &XOY_histogram);

      /** \brief normalizing a 1D histogram
 	* \param[in] histogram 1D histogram (int).
	* \param[out] normalized_histogram the resultant normalized_histogram (float).
        */       
      void
      normalizingHistogram (std::vector <unsigned int> histogram, std::vector <float> &normalized_histogram);

      /** \brief compute viewpoint entropy used for concatenating projections
 	* \param[in] normalized_histogram normalized_histogram (float).
	* \param[out] entropy the resultant view entropy.
        */   
      void
      viewpointEntropy (std::vector <float> normalized_histogram, float &entropy);

      /** \brief find max view point entropy used for concatenating projections
 	* \param[in] view_point_entropy a vector of float contains three view entropies.
	* \param[out] index the resultant projection index.
        */  
      void
      findMaxViewPointEntropy (std::vector<float> view_point_entropy, int &index);

      /** \brief compute average of view point histograms
	* \param[in] histogram1 normalized histogram of projection 1.
	* \param[in] histogram2 normalized histogram of projection 2
	* \param[in] histogram3 normalized histogram of projection 3
	* \param[out] average the resultant average of three histograms.
        */  
      void
      averageHistograms (std::vector<float> histogram1, std::vector<float> historam2, std::vector<float> historam3, std::vector<float> &average);

      /** \brief compute mean of a given histogram
	* \param[in] histogram normalized histogram of projection.
	* \param[out] mean the resultant mean.
        */        
      void
      meanOfHistogram (std::vector<float> histogram, float &mean);

      /** \brief compute variance of a given histogram
	* \param[in] histogram normalized histogram of projection.
	* \param[in] mean mean of the given histogram.
	* \param[out] mean the resultant mean.
        */        
      void
      varianceOfHistogram (std::vector<float> histogram, float mean, float &variance);

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
      objectViewHistogram (int maximum_entropy_index, std::vector<std::vector<float> > normalized_projected_views,
		      std::vector<float> &sorted_normalized_projected_views,
		      std::string &name_of_sorted_projected_plane /*debug*/);

      /** \brief compute largest side of boundingbox
	* \param[in] dimensions shows the dimention of boundingbox
	* \param[out] largest_side the resultant largest side.
        */     
      void
      computeLargestSideOfBoundingBox (pcl::PointXYZ dimensions, double &largest_side );

      /** \brief compute compute distance between two projections
	* \param[in] projection1 2D histogram
	* \param[in] projection2 2D histogram
	* \param[out] largest_side the resultant distance.
        */           
      void
      computeDistanceBetweenProjections (std::vector <std::vector<float> > projection1, std::vector <std::vector<float> > projection2, float &distance);

      /** \brief initializing a 2D histogram with 0; This function creates a 2D vector with the size of [number_of_bins × number_of_bins] and initializes all elements with 0 value.
	* \param[in] number_of_bins number of bin 
	* \param[out] a_2D_vector
        */           
      inline std::vector<std::vector<unsigned int> > initializing2DHistogram (unsigned int number_of_bins);
      
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/good.hpp>
#endif

#endif /* PCL_FEATURES_GOOD_H_ */
