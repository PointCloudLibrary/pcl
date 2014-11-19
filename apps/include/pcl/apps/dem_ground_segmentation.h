/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 */
#ifndef PCL_DEM_GROUND_SEGMENTATION_H_
#define PCL_DEM_GROUND_SEGMENTATION_H_

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>

#include <opengm/graphicalmodel/graphicalmodel.hxx>

namespace pcl
{
  /** \brief Segment sequences of the Digital Elavation Map for road and sidewalks.
    *
    * Example of usage:
    * 
    * \code
    *  
    * \endcode
    *
    * \author Timur Ibadov (ibadov.timur@gmail.com)
    * \ingroup stereo
    */
  class DEMGroundSegmentation
  {
  public:
    /** \brief Enum for returning labels. */
    enum DEMLabels
    {
      UNASSIGNED       = 0u,
      ROAD             = 1u,
      LEFT_SIDEWALK    = 2u,
      RIGHT_SIDEWALK   = 3u,
      NUMBER_OF_LABELS = 4u
    };

    typedef pcl::PointDEM PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;

    /** \brief DEMGroundSegmentation constructor. */
    DEMGroundSegmentation (void);
    /** \brief Empty destructor. */
    ~DEMGroundSegmentation (void);
  
    /** \brief Set the current DEM.
      * \param[in] cloud Pointer to the current DEM.
      */
    void
    setInputCloud (const PointCloudPtr &cloud);

    /** \brief Set parameters of the transformation from the current DEM to the previous DEM.
      * \param[in] rototranslation The transformation matrix.
      */
    inline void
    setTransformation (const Eigen::Matrix4f &rototranslation)
    {
      rototranslation_ = boost::shared_ptr<Eigen::Matrix4f> (
          new Eigen::Matrix4f);
      *rototranslation_ = rototranslation;
    }

    /** \brief Segment the current DEM.
      * \param[out] labels Labels of each cell of the current DEM.
      * \param[out] left_curb Parameters of the left curb.
      * \param[out] right_curb Parameters of the right curb.
      * \param[out] road Parameters of the road surface.
      */
    void
    segment (pcl::PointCloud<pcl::Label> &labels, 
        std::vector<double> &left_curb,
        std::vector<double> &right_curb,
        std::vector<double> &road);
  private:
    /** \brief Init starting labels (or get it from the previous frame). */
    void 
    initLabels();

    /** \brief Estimate parameters of the scene. */
    void
    estimateSceneParameters ();

    /** \brief Estimate labels for the corresponding parameters. */
    void
    estimateLabels ();

    /** \brief Check, if specified surface has differ height with the road along a curb.
      * \param sidewalk_type The type of the required sidewalk.
      * \return true, if the specified surface realy exist, or false, 
      * if it belongs to the road.
      */
    bool
    checkSidewalk (DEMLabels sidewalk_type);
    
    /** \brief Estimate parameters of the curb (left or right).
      * \param x Parameters.
      * \param grad Precomputed gradients.
      * \param data A pointer to std::pair<DEMGroundSegmentation*, DEMLabels> 
      * (first parameter - *this, to access to the private part of the class,
      * second - what labels should be used for estimation).
      * \return The value of the function.
      */
    static double 
    curbError (const std::vector<double> &x, std::vector<double> &grad, void *data);

    /** \brief Estimate parameters of the surface (the road, the left or right sidewalk).
      * \param x Parameters.
      * \param grad Precomputed gradients.
      * \param data A pointer to std::pair<DEMGroundSegmentation*, DEMLabels> 
      * (first parameter - *this, to access to the private part of the class,
      * second - what labels should be used for estimation).
      * \return The value of the function.
      */
    static double 
    surfaceError (const std::vector<double> &x, std::vector<double> &grad, void *data);

    /** \brief Construct the unary term for the graphical model.
      * \param point_idx Index of the point.
      * \param first_label Label of the point.
      * \return Resulting function.
      */
    double
    constructUnaryTerm (const unsigned point_idx, const DEMLabels label);

    /** \brief Construct the prior term for the graphical model.
      *
      * This term performs a connection of the graphical model to the previous frame.
      * It's a part of the unary term.
      *
      * \param point_idx Index of the point.
      * \param first_label Label of the point.
      * \return Resulting function.
      */
    double
    constructPriorTerm (const unsigned point_idx, const DEMLabels label);

    /** \brief Construct the binary term for the graphical model.
      * \param first_idx Index of the first point.
      * \param first_label Label of the first point.
      * \param second_idx Index of the second point.
      * \param second_label Label of the second point.
      * \return Resulting function.
      */
    double
    constructBinaryTerm (const unsigned first_idx, const DEMLabels first_label,
                         const unsigned second_idx, const DEMLabels second_label);

    /** \brief Compute height variance of the current DEM relative to the specified surface.
      * \param params The surface parameters.
      * \param label The interested label.
      *\ return Resulting variance.
      */
    double
    computeSurfaceVariance (const std::vector<double> &params, const DEMLabels label);

    /** \brief Compute heigh of the surface in the point.
      * \param params The surface parameters.
      * \param point The point.
      * \return Resulting height.
      */
    static double
    computeSurfaceHeight (const std::vector<double> &params, const pcl::PointDEM &point);

    /** \brief Compute heigh of the curb sigmoid in the point.
      * \param params The curb parameters.
      * \param point The point.
      * \param label Type of the curb (sigmoid should be reflected for the left curb).
      * \param stepness Stepness of sigmoid.
      * \return Resulting height.
      */
    static double
    computeCurbSigmoid (const std::vector<double> &params, const pcl::PointDEM &point, 
        const DEMLabels label, const double steepness);

    /** \brief Compute the Gaussian function, centeres at point.y, with standart deviation taken from the point.height_variance. 
      * \param point The point.
      * \param arg Argument of the function.
      * \return Resulting value.
      */
    double
    computeGaussian (const pcl::PointDEM &point, const double arg);

    /** \brief Maximum number of iteration of the algorithm. */
    const unsigned MAX_NUMBER_OF_ITERATION;
    /** \brief Number of parameters of the surfaces. */
    const unsigned NUMBER_OF_SURFACE_PARAMETERS;
    /** \brief Number of parameters of the curbs. */
    const unsigned NUMBER_OF_CURB_PARAMETERS;
    /**brief  Variable to avoid divizion by zero when height variance is used. */
    const double VARIANCE_ERROR;
    /** \brief Start steepness of the sigmoid in curb estimation. */
    const double STEEPNESS_START;
    /** \brief Step of the steepness of the sigmoid in curb estimation. */
    const double STEEPNESS_STEP;
    /** \brief Min height of the curb. */
    const double HEIGHT_THRESHOLD;
    /** \brief X-coordinate of the left curb in initial labeling. */
    const float LEFT_CURB_INITIAL_X;
    /** \brief X-coordinate of the right curb in initial labeling. */
    const float RIGHT_CURB_INITIAL_X;

    /** \brief Steepness of the sigmoid in curb estimation. */
    double steepness_of_sigmoid_;
    
    /** \brief The current DEM pointer. */
    PointCloudPtr dem_;
    /** \brief A pointer to the labels of the current DEM. */
    pcl::PointCloud<pcl::Label>::Ptr labels_;
    /** \brief The current transformation matrix. */
    boost::shared_ptr<Eigen::Matrix4f> rototranslation_;
    /** \brief The unary terms of the current step. */
    boost::shared_ptr<std::vector<double> > probabilities_;
    
    /** \brief Vector of the road parameters. */
    std::vector<double> road_;
    /** \brief Vector of the left sidewalk parameters. */
    std::vector<double> sidewalk_left_;
    /** \brief Vector of the right sidewalk parameters. */
    std::vector<double> sidewalk_right_;
    /** \brief Vector of the left curb parameters. */
    std::vector<double> curb_left_;
    /** \brief Vector of the right curb parameters. */
    std::vector<double> curb_right_;
    /** \brief Height variance of the current DEM relative to the road surface. */
    double road_variance_;
    /** \brief Height variance of the current DEM relative to the left sidewalk surface. */
    double sidewalk_left_variance_;
    /** \brief Height variance of the current DEM relative to the right sidewalk surface. */
    double sidewalk_right_variance_;

    /** \brief Is it necessary to estimate left curb and sidewalk. */
    bool estimate_left_;
    /** \brief Is it necessary to estimate right curb and sidewalk. */
    bool estimate_right_;

    /** \brief The previous DEM pointer. */
    PointCloudPtr previous_dem_;
    /** \brief A pointer to the labels of the previous DEM. */
    pcl::PointCloud<pcl::Label>::Ptr previous_labels_;
    /** \brief The unary terms of the previous step. */
    boost::shared_ptr<std::vector<double> > previous_probabilities_;
  };
}

#endif // PCL_DEM_GROUND_SEGMENTATION_H_
