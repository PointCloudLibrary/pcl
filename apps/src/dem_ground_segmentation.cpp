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

#include <pcl/apps/dem_ground_segmentation.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include <pcl/console/print.h>

#include <nlopt.hpp>

#include <opengm/opengm.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/operations/minimizer.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>

pcl::DEMGroundSegmentation::DEMGroundSegmentation(void) :
    MAX_NUMBER_OF_ITERATION (3),
    NUMBER_OF_SURFACE_PARAMETERS (6),
    NUMBER_OF_CURB_PARAMETERS (4),
    VARIANCE_ERROR (0.000),
    STEEPNESS_START (1.0),
    STEEPNESS_STEP (2.0),
    HEIGHT_THRESHOLD (0.03),
    LEFT_CURB_INITIAL_X (-0.7f),
    RIGHT_CURB_INITIAL_X (0.7f),
    steepness_of_sigmoid_ (STEEPNESS_START),
    road_ (NUMBER_OF_SURFACE_PARAMETERS, 0.0f),
    sidewalk_left_ (NUMBER_OF_SURFACE_PARAMETERS, 0.0f),
    sidewalk_right_ (NUMBER_OF_SURFACE_PARAMETERS, 0.0f),
    curb_left_ (NUMBER_OF_CURB_PARAMETERS, 0.0f),
    curb_right_ (NUMBER_OF_CURB_PARAMETERS, 0.0f)
{
}


pcl::DEMGroundSegmentation::~DEMGroundSegmentation(void)
{
}

void
pcl::DEMGroundSegmentation::setInputCloud (const PointCloudPtr &cloud)
{
  // Check, if the previous DEM is available.
  if (previous_dem_)
  {
    // Check, if its size doesn't correspond to the current DEM.
    if (previous_dem_->width != cloud->width || 
        previous_dem_->height != cloud->height)
    {
      PCL_WARN ("[pcl::DEMGroundSegmentation::setInputCloud] The size of the cloud doesn/t correspond to the previous one's.\n");
      previous_dem_.reset ();
      previous_labels_.reset ();
      previous_probabilities_.reset ();
      
      // This vector should be resized later.
      probabilities_.reset ();
    }
  }

  dem_ = cloud;
}

void
pcl::DEMGroundSegmentation::segment (pcl::PointCloud<pcl::Label> &labels,
    std::vector<double> &left_curb, std::vector<double> &right_curb,
    std::vector<double> &road)
{
  // Check availability of the data.
  if (!rototranslation_)
  {
    PCL_ERROR ("[pcl::DEMGroundSegmentation::segment] Transformation parameters was not set.\n");
    return;
  }
  if (!dem_)
  {
    PCL_ERROR ("[pcl::DEMGroundSegmentation::segment] Input was not set.\n");
    return;
  }

  try
  {
    // Init starting labels (or get it from the previous frame).
    initLabels ();
    // Number of the iteration.
    unsigned iteration_count = 0;
    // The loop condition.
    bool iterate;

    // It's necessary to compute both of the sidewalks.
    estimate_left_ = true;
    estimate_right_ = true;

    // Untill labels would not be good.
    do
    {
      // Estimate scene parameters.
      estimateSceneParameters ();

      // Compute labels for these parameters.
      estimateLabels ();

      // Increase the steepness of the sigmoid.
      steepness_of_sigmoid_ += STEEPNESS_STEP;

      // If iteration should be continued.
      iterate = (++iteration_count < MAX_NUMBER_OF_ITERATION);
    }
    while (iterate);
    
    // Check, if all sidewalks realy exist.
    estimate_left_ = checkSidewalk (LEFT_SIDEWALK);
    estimate_right_ = checkSidewalk (RIGHT_SIDEWALK);
  }

  catch (std::exception error)
  {
    PCL_ERROR ("[pcl::DEMGroundSegmentation::segment] %s.\n", error.what ());
    return;
  }
  // Return final labels and save the current data as the previous.
  labels = *labels_;
  if (estimate_left_)
  {
    left_curb = curb_left_;
  }
  if (estimate_right_)
  {
    right_curb = curb_right_;
  }
  road = road_;
  previous_dem_ = dem_;
  previous_labels_ = labels_;
  previous_probabilities_ = probabilities_;

  probabilities_ = boost::shared_ptr<std::vector<double> > ();
}

void 
pcl::DEMGroundSegmentation::initLabels()
{
  // Check, if the previous labels exist.
  if (previous_labels_)
  {
    labels_ = previous_labels_;
    steepness_of_sigmoid_ = STEEPNESS_START;

    // If one of the curbs was not found before, add it to initial labeling.
    for (unsigned row = 0; row < labels_->height; ++row)
    {
      for (unsigned column = 0; column < labels_->width; ++column)
      {
        pcl::PointDEM point = dem_->at (column, row);
        if (!estimate_left_)
        {
          if (point.x < LEFT_CURB_INITIAL_X)
          {
            labels_->at (column, row).label = LEFT_SIDEWALK;
          }
          else if (point.x <= 0)
          {
            labels_->at (column, row).label = ROAD;
          }
        }
        if (!estimate_right_)
        {
          if (point.x > RIGHT_CURB_INITIAL_X)
          {
            labels_->at (column, row).label = RIGHT_SIDEWALK;
          }
          else if (point.x > 0.0f)
          {
            labels_->at (column, row).label = ROAD;
          }
        }
      }
    }

    // Label NaN points as "UNASSIGNED".
    for (unsigned point_idx = 0; point_idx < dem_->size (); ++point_idx)
    {
      if (!pcl::isFinite (dem_->at (point_idx)))
      {
        labels_->at (point_idx).label = UNASSIGNED;
      }      
    }

    // If one of the curbs was not presented, delete previous_probabilities.
    if (!estimate_right_ || !estimate_left_)
    {
      previous_probabilities_.reset ();
    }
    
  }
  else
  {
    /* Create our own starting labels.
     * The DEM will be divided into 3 parts, left part will be marked as 
     * a left sidewalk, center part as a road and the right part as
     * a right sidewalk.
     */
    labels_ = pcl::PointCloud<pcl::Label>::Ptr (new
      pcl::PointCloud<pcl::Label>);

    unsigned width = dem_->width;
    unsigned height = dem_->height;
    // Init labels.
    labels_->resize (width * height);
    labels_->width = width;
    labels_->height = height;
    for (unsigned row = 0; row < height; ++row)
    {
      for (unsigned column = 0; column < width; ++column)
      {
        pcl::PointDEM point = dem_->at (column, row);
        if (point.x < LEFT_CURB_INITIAL_X)
        {
          labels_->at(column, row).label = LEFT_SIDEWALK;
        }
        else if (point.x > RIGHT_CURB_INITIAL_X)
        {
          labels_->at(column, row).label = RIGHT_SIDEWALK;
        }
        else
        {
          labels_->at(column, row).label = ROAD;
        }
      }
    }

    steepness_of_sigmoid_ = STEEPNESS_START;
  } // else
}

void
pcl::DEMGroundSegmentation::estimateSceneParameters ()
{
  double minf;
  double maxtime = 1.0;
  // Data to pass to the error functions.
  std::pair<DEMGroundSegmentation*, DEMLabels> data;
  data.first = this;

  // Init the curb optimizer.
  {
    nlopt::opt opt(nlopt::GN_CRS2_LM, NUMBER_OF_CURB_PARAMETERS);

    std::vector<double> lb(NUMBER_OF_CURB_PARAMETERS);
    lb[0] = -0.1; 
    lb[1] = -10.0; 
    lb[2] = -10.0; 
    lb[3] = -10.0; 
    opt.set_lower_bounds(lb);
    
    std::vector<double> ub(NUMBER_OF_CURB_PARAMETERS);
    ub[0] = 0.1; 
    ub[1] = 10.0; 
    ub[2] = 10.0; 
    ub[3] = 10.0; 
    opt.set_upper_bounds(ub);
    
    // Relative tolerance on optimization parameters.
    opt.set_xtol_rel(1e-4);
    opt.set_maxtime (maxtime);

    // Estimate the left curb.
    if (estimate_left_)
    {
      data.second = LEFT_SIDEWALK;
      opt.set_min_objective(curbError, &data);
      opt.optimize(curb_left_, minf);
    }
    // Estimate the right curb
    if (estimate_right_)
    {
      data.second = RIGHT_SIDEWALK;
      opt.set_min_objective(curbError, &data);
      opt.optimize(curb_right_, minf);
    }
  }
  // Init the surface optimizer.
  {
    nlopt::opt opt(nlopt::GN_CRS2_LM, NUMBER_OF_SURFACE_PARAMETERS);

    std::vector<double> lb(NUMBER_OF_SURFACE_PARAMETERS);
    lb[0] = -0.001; 
    lb[1] = -0.001; 
    lb[2] = -0.001; 
    lb[3] = -0.5; 
    lb[4] = -0.5; 
    lb[5] = -2.0; 
    opt.set_lower_bounds(lb);
    
    std::vector<double> ub(NUMBER_OF_SURFACE_PARAMETERS);
    ub[0] = 0.001; 
    ub[1] = 0.001; 
    ub[2] = 0.001; 
    ub[3] = 0.5; 
    ub[4] = 0.5; 
    ub[5] = 2.0; 
    opt.set_upper_bounds(ub);
    
    // Relative tolerance on optimization parameters.
    opt.set_xtol_rel (1e-4);
    opt.set_maxtime (maxtime);

    // Estimate the road.
    data.second = ROAD;
    opt.set_min_objective(surfaceError, &data);
    opt.optimize(road_, minf);
    // Estimate the left sidewalk.
    if (estimate_left_)
    {
      data.second = LEFT_SIDEWALK;
      opt.set_min_objective(surfaceError, &data);
      opt.optimize(sidewalk_left_, minf);
    }
    // Estimate the right sidewalk.
    if (estimate_right_)
    {
      data.second = RIGHT_SIDEWALK;
      opt.set_min_objective(surfaceError, &data);
      opt.optimize(sidewalk_right_, minf);
    }
  }

  // Compute surfaces variances.
  road_variance_ = computeSurfaceVariance (road_, ROAD);
  sidewalk_left_variance_ = computeSurfaceVariance (sidewalk_left_, LEFT_SIDEWALK);
  sidewalk_right_variance_ = computeSurfaceVariance (sidewalk_right_, RIGHT_SIDEWALK);
}

void
pcl::DEMGroundSegmentation::estimateLabels ()
{
  // Init the size of the graph.
  unsigned width = dem_->width;
  unsigned height = dem_->height;

  // Init the vector with the unary terms, if necessary.
  if (!probabilities_)
  {
    probabilities_ = boost::shared_ptr<std::vector<double> > (new 
        std::vector<double> (width * height * NUMBER_OF_LABELS));
  }

  // Init the graphical model.
  typedef opengm::ExplicitFunction<double> ExplicitFunction;
  typedef opengm::SimpleDiscreteSpace<unsigned, unsigned> Space;
  typedef opengm::GraphicalModel<double, opengm::Adder, 
      opengm::meta::TypeList<ExplicitFunction, opengm::meta::ListEnd>, 
      Space> Model;
  typedef Model::FunctionIdentifier FunctionIdentifier;
  Space space (width * height, NUMBER_OF_LABELS);
  Model model (space);

  // Construct a special unary function for points with NaN.
  const unsigned shape[] = {NUMBER_OF_LABELS};
  ExplicitFunction f (shape, shape + 1);
  f (static_cast<unsigned> (UNASSIGNED)) = 1.0;
  for(unsigned state = 1; state < NUMBER_OF_LABELS; ++state)
  {
    f(state) = 0.0;
  }
  FunctionIdentifier function_nan_idx = model.addFunction (f);

  // Init 1st order factors.
  for(unsigned variable = 0; variable < model.numberOfVariables (); ++variable)
  {
    if (!pcl::isFinite (dem_->at (variable)))
    {
      // This variable should always be UNASSIGNED. 
      unsigned variable_idx[] = {variable};
      model.addFactor (function_nan_idx, variable_idx, variable_idx + 1);
    }
    else
    {
      // Construct the function.
      const unsigned shape[] = {model.numberOfLabels (variable)};
      ExplicitFunction f (shape, shape + 1);
      for(unsigned state = 0; state < model.numberOfLabels (variable); ++state)
      {
        f(state) = constructUnaryTerm (variable, static_cast<DEMLabels>(state));
      }
      FunctionIdentifier function_idx = model.addFunction (f);
      
      // Add it to the model. 
      unsigned variable_idx[] = {variable};
      model.addFactor (function_idx, variable_idx, variable_idx + 1);
    }
  }

  // Init 2nd order factors.
  for (unsigned row = 0; row < height; ++row) 
  {
    for (unsigned column = 0; column < width; ++column)
    {
      unsigned first_index = column + row * width;
      // Avoid edges to NaN points.
      if (!pcl::isFinite (dem_->at (first_index)))
      {
        continue;
      }

      // Vector with indexes of points, that make an edge with the current point.
      std::vector<unsigned> second_indexes;
      // Check, if it's not last column.
      if (column + 1 < width)
      {
        unsigned second_index = column + 1 + row * width;
        if (pcl::isFinite (dem_->at (second_index)))
        {
          second_indexes.push_back (second_index);
        }
      }
      // Check, if it's not last row.
      if (row + 1 < height)
      {
        unsigned second_index = column + (row + 1) * width;
        if (pcl::isFinite (dem_->at (second_index)))
        {
          second_indexes.push_back (second_index);
        }
      }
      // Add edges.
      for (unsigned edge_idx = 0; edge_idx < second_indexes.size (); ++edge_idx)
      {
        unsigned second_index = second_indexes[edge_idx];
        // Construct the function.
        const unsigned shape[] = {model.numberOfLabels (first_index),
                                model.numberOfLabels (second_index)};
      
        ExplicitFunction f(shape, shape + 2);
        for (unsigned state1 = 0; state1 < model.numberOfLabels (first_index); ++state1)
        {
          for (unsigned state2 = 0; state2 < model.numberOfLabels (second_index); ++state2)
          {         
            f (state1, state2) = constructBinaryTerm (
                first_index, static_cast<DEMLabels>(state1), 
                second_index, static_cast<DEMLabels>(state2));
          }
        }
        FunctionIdentifier function_idx = model.addFunction (f);

        // Add it to the model. 
        unsigned variable_idx[] = {first_index, second_index};
        std::sort (variable_idx, variable_idx + 2);
        model.addFactor (function_idx, variable_idx, variable_idx + 2);
      }
    }
  }

  // Optimize.
  typedef opengm::BeliefPropagationUpdateRules<Model, opengm::Maximizer> UpdateRules;
  typedef opengm::MessagePassing<Model, opengm::Maximizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;
  const unsigned MAX_NUMBER_OF_ITERATIONS = 100;
  const double CONVERGENCE_BOUND = 1e-4;
  const double DAMPING = 0.5;
  BeliefPropagation::Parameter parameter (MAX_NUMBER_OF_ITERATIONS, CONVERGENCE_BOUND, DAMPING);
  BeliefPropagation bp (model, parameter);
  
  BeliefPropagation::EmptyVisitorType visitor; // Remove it.
  bp.infer (visitor);

  // Set the output.
  std::vector<unsigned int> labels;
  bp.arg (labels);
  for (unsigned label = 0; label < labels.size (); ++label)
  {
    labels_->at (label).label = labels[label];
  }
}

bool
pcl::DEMGroundSegmentation::checkSidewalk (DEMLabels sidewalk_type)
{
  std::vector<double> *curb;
  if (sidewalk_type == LEFT_SIDEWALK)
  {
    curb = &curb_left_;
  }
  else if (sidewalk_type == RIGHT_SIDEWALK)
  {
    curb = &curb_right_;
  }

  unsigned z_steps = dem_->height;

  // Vector for height difference.
  std::vector<float> heights;
  heights.reserve (z_steps);

  for (unsigned row = 0; row < z_steps; ++row)
  {
    float z_value = dem_->at (0, row).z;
    float x_value = curb->at (0) * z_value * z_value * z_value +
                    curb->at (1) * z_value * z_value +
                    curb->at (2) * z_value +
                    curb->at (3);
    int right_col = 1;
    for (unsigned col = 1; col < dem_->width - 1; ++col)
    {
      if (dem_->at (col, row).x > x_value)
      {
        right_col = col;
        break;
      }
    }

    float diff = dem_->at (right_col - 1, row).y - dem_->at (right_col, row).y;
    // Cneck, if difference is not NaN.
    if (diff == diff)
    {
      heights.push_back (abs (diff));
    }
  }

  // Check, if median difference is greater, then the threshold.
  if (heights.size() == 0)
  {
    return false;
  }
  std::sort (heights.begin (), heights.end ());
  float median_height = heights[heights.size () / 2];
  if (median_height <= HEIGHT_THRESHOLD)
  {
    return false;
  }
  return true;
}

double 
pcl::DEMGroundSegmentation::surfaceError (
    const std::vector<double> &x, std::vector<double> &grad, void *data)
{
  // Init data.
  DEMGroundSegmentation* segmentator = 
      static_cast <std::pair<DEMGroundSegmentation*, DEMLabels>*> (data)->first;
  pcl::PointCloud<pcl::PointDEM>::Ptr dem = segmentator->dem_;
  pcl::PointCloud<pcl::Label>::Ptr labels = segmentator->labels_;
  DEMLabels interested_label = 
      static_cast <std::pair<DEMGroundSegmentation*, DEMLabels>*> (data)->second;
  const double EPSILON = segmentator->VARIANCE_ERROR;

  // Init gradients, if it needs.
  if (grad.size () > 0)
  {
    for (unsigned var_idx = 0; var_idx < grad.size (); ++var_idx)
    {
      grad[var_idx] = 0.0;
    }
  }

  // Resulting value of the function.
  double result = 0.0;
  for (unsigned point_idx = 0; point_idx < dem->size (); ++point_idx)
  {
    pcl::PointDEM point = dem->at (point_idx);
    if (!pcl::isFinite (point) || 
        labels->at (point_idx).label != interested_label)
    {
      continue;
    }
    double surface_height = computeSurfaceHeight (x, point);
    double point_error = point.y - surface_height;
    result += point_error * point_error / (point.height_variance + EPSILON);

    // Compute the gradients.
    if (grad.size () > 0)
    {
      double grad_multiplier = -2.0 * point_error / (point.height_variance + EPSILON);
      grad[0] += grad_multiplier * point.x * point.x;
      grad[1] += grad_multiplier * point.z * point.z;
      grad[2] += grad_multiplier * 2.0 * point.x * point.z;
      grad[3] += grad_multiplier * point.x;
      grad[4] += grad_multiplier * point.z;
      grad[5] += grad_multiplier;
    }
  }

  return (result);
}

double 
pcl::DEMGroundSegmentation::curbError (
    const std::vector<double> &x, std::vector<double> &grad, void *data)
{
  // Init data.
  DEMGroundSegmentation* segmentator = 
      static_cast <std::pair<DEMGroundSegmentation*, DEMLabels>*> (data)->first;
  pcl::PointCloud<pcl::PointDEM>::Ptr dem = segmentator->dem_;
  pcl::PointCloud<pcl::Label>::Ptr labels = segmentator->labels_;
  DEMLabels interested_label = 
      static_cast <std::pair<DEMGroundSegmentation*, DEMLabels>*> (data)->second;

  // Init gradients, if it needs.
  if (grad.size () > 0)
  {
    for (unsigned var_idx = 0; var_idx < grad.size (); ++var_idx)
    {
      grad[var_idx] = 0.0;
    }
  }

  // Resulting value of the function.
  double result = 0.0;
  for (unsigned point_idx = 0; point_idx < dem->size (); ++point_idx)
  {
    pcl::PointDEM point = dem->at (point_idx);
    if (!pcl::isFinite (point) || 
       (labels->at (point_idx).label != interested_label &&
        labels->at (point_idx).label != pcl::DEMGroundSegmentation::ROAD))
    {
      continue;
    }

    double height;
    if (labels->at (point_idx).label == pcl::DEMGroundSegmentation::ROAD)
    {
      height = -1.0;
    }
    else
    {
      height = 1.0;
    }

    double sigmoid = computeCurbSigmoid (x, point, interested_label, segmentator->steepness_of_sigmoid_);
    double deviation = height - sigmoid;

    result += deviation * deviation;// * weight;

    // Compute the gradients.
    if (grad.size () > 0)
    {
      double point_error = x[0] * point.z * point.z * point.z +
                           x[1] * point.z * point.z +
                           x[2] * point.z +
                           x[3] - point.x;
      if (interested_label == pcl::DEMGroundSegmentation::LEFT_SIDEWALK)
      {
        point_error *= -1.0;
      }
      
      double exponent = exp (segmentator->steepness_of_sigmoid_ * point_error);
      double grad_multiplier = -4.0 * deviation /
          (exponent + 1.0) / (exponent + 1.0) *
          exponent * segmentator->steepness_of_sigmoid_;// * weight;
      grad[0] += grad_multiplier * point.z * point.z * point.z;
      grad[1] += grad_multiplier * point.z * point.z;
      grad[2] += grad_multiplier * point.z;
      grad[3] += grad_multiplier;
    }
  }

  return (result);
}

double
pcl::DEMGroundSegmentation::constructUnaryTerm (
    const unsigned point_idx, const DEMLabels label)
{
  pcl::PointDEM point = dem_->at (point_idx);
  // Compute and normalize the a priori terms.
  double road_apriori = (
      (1.0 - computeCurbSigmoid (curb_left_, point, LEFT_SIDEWALK, steepness_of_sigmoid_)) * 
      (1.0 - computeCurbSigmoid (curb_right_, point, RIGHT_SIDEWALK, steepness_of_sigmoid_))) / 
      2.0 / (point.height_variance + VARIANCE_ERROR);
  double sidewalk_left_apriori = 
      (1.0 + computeCurbSigmoid (curb_left_, point, LEFT_SIDEWALK, steepness_of_sigmoid_)) / 
      (point.height_variance + VARIANCE_ERROR);
  double sidewalk_right_apriori = 
      (1.0 + computeCurbSigmoid (curb_right_, point, RIGHT_SIDEWALK, steepness_of_sigmoid_)) / 
      (point.height_variance + VARIANCE_ERROR);
  double unassigned_apriori = 1.0 / (1.0 + road_apriori + sidewalk_left_apriori + sidewalk_right_apriori);
  road_apriori *= unassigned_apriori;
  sidewalk_left_apriori *= unassigned_apriori;
  sidewalk_right_apriori *= unassigned_apriori;

  // Compute the probability.
  double probability = 0.0;
  double gaussian = 0.0;
  switch (label)
  {
    case ROAD:
      gaussian = computeGaussian (point, computeSurfaceHeight (road_, point));
      probability = road_apriori * gaussian * gaussian * gaussian;
      break;
    case LEFT_SIDEWALK:
      gaussian = computeGaussian (point, computeSurfaceHeight (sidewalk_left_, point));
      probability = sidewalk_left_apriori * gaussian;
      break;
    case RIGHT_SIDEWALK:
      gaussian = computeGaussian (point, computeSurfaceHeight (sidewalk_right_, point));
      probability = sidewalk_right_apriori * gaussian;
      break;
    case UNASSIGNED:
      gaussian = computeGaussian (point, point.y +
          3.0 * std::sqrt (point.height_variance + VARIANCE_ERROR));
      probability = unassigned_apriori * gaussian;
      break;
    default:
      break;
  }

  probability = std::log (probability);

  probability += constructPriorTerm (point_idx, label);

  probabilities_->at (label + NUMBER_OF_LABELS * point_idx) = probability;

  return (probability);
}

double
pcl::DEMGroundSegmentation::constructPriorTerm (
    const unsigned point_idx, const DEMLabels label)
{
  if (!previous_labels_ || !previous_probabilities_)
  {// There is no information about the previous step.
    return (0.0);
  }
  else
  {
    // Transform point to the previous DEM's coordinates.
    PointDEM point = dem_->at (point_idx);
    Eigen::Vector4f point_vector (point.x,
                                  point.y,
                                  point.z,
                                  1.0f);
    Eigen::Vector4f point_vector_transformed = *rototranslation_ * point_vector;

    // Find the transformed point in the previous DEM.
    int row = 0; 
    int col = 0;
    while (row < previous_dem_->height)
    {
      if (point_vector_transformed.z() > previous_dem_->at (0, row).z)
      {
        break;
      }
      ++row;
    }
    if (row >= previous_dem_->height || row <= 0)
    {
      return (1.0);
    }

    while (col < previous_dem_->width)
    {
      if (point_vector_transformed.x() < previous_dem_->at (col, row).x)
      {
        break;
      }
      ++col;
    }
    if (col >= previous_dem_->width || col <= 0)
    {
      return (1.0);
    }

    // Interpolate the probability.
    Eigen::Vector2f point_vector_2d (
        point_vector_transformed.x (),
        point_vector_transformed.z ()
    );
    Eigen::Vector2f right_top_point (
        previous_dem_->at (col, row).x,
        previous_dem_->at (col, row).z
    );
    Eigen::Vector2f right_bot_point (
        previous_dem_->at (col - 1, row).x,
        previous_dem_->at (col - 1, row).z
    );
    Eigen::Vector2f left_top_point (
        previous_dem_->at (col, row - 1).x,
        previous_dem_->at (col, row - 1).z
    );
    Eigen::Vector2f left_bot_point (
        previous_dem_->at (col - 1, row - 1).x,
        previous_dem_->at (col - 1, row - 1).z
    );

    float right_top_dist = (point_vector_2d - right_top_point).norm();
    float right_bot_dist = (point_vector_2d - right_bot_point).norm();
    float left_top_dist = (point_vector_2d - left_top_point).norm();
    float left_bot_dist = (point_vector_2d - left_bot_point).norm();

    float sum_dist = right_top_dist + right_bot_dist + left_top_dist + left_bot_dist;

    float probability = 0.0f;
    float prev_probability = 0.0f;

    prev_probability = previous_probabilities_->at (label + NUMBER_OF_LABELS * (row * dem_->width + col));
    probability += (1.0f - right_top_dist / sum_dist) * prev_probability;
    
    prev_probability = previous_probabilities_->at (label + NUMBER_OF_LABELS * (row * dem_->width + col - 1));
    probability += (1.0f - right_bot_dist / sum_dist) * prev_probability;

    prev_probability = previous_probabilities_->at (label + NUMBER_OF_LABELS * ((row - 1) * dem_->width + col));
    probability += (1.0f - left_top_dist / sum_dist) * prev_probability;

    prev_probability = previous_probabilities_->at (label + NUMBER_OF_LABELS * ((row - 1) * dem_->width + col - 1));
    probability += (1.0f - left_bot_dist / sum_dist) * prev_probability;

    probability /= 3.0f;

    return (probability);
  }
}

double
pcl::DEMGroundSegmentation::constructBinaryTerm (
    const unsigned first_idx, const DEMLabels first_label,
    const unsigned second_idx, const DEMLabels second_label)
{
  pcl::PointDEM first_point = dem_->at (first_idx);
  pcl::PointDEM second_point = dem_->at (second_idx);
  double distance = std::abs (first_point.y - second_point.y);
  double sigma = std::sqrt (first_point.height_variance + second_point.height_variance + VARIANCE_ERROR);
  /* Compute the shifted sigmoid for the distance. 
   * Parameters selection is caused by the fact that
   * sigmoid (0) is near 1 and sigmoid (3 * sigma) is equal to 0.5. 
   */
  double sigmoid = 1.0 / (1.0 + std::exp (5.0 / 3.0 / sigma * distance - 5));

  // Check for NaN, infinity and zero.
  if (sigmoid != sigmoid || sigmoid == 0)
  {
    throw std::out_of_range ("Binary term is not finite");
  }

  if (first_label == second_label)
  {
    return std::log(sigmoid);
  }
  else
  {
    return std::log(1 - sigmoid);
  }
}

double
pcl::DEMGroundSegmentation::computeSurfaceVariance (
    const std::vector<double> &params, const DEMLabels label)
{
  double variances_sum = 0;
  unsigned number_of_interested_labels = 0;
  for (unsigned point_idx = 0; point_idx < dem_->size (); ++point_idx)
  {
    if (labels_->at (point_idx).label == label)
    {
      pcl::PointDEM point = dem_->at (point_idx);
      if (pcl::isFinite (point))
      {
        double difference = point.y - computeSurfaceHeight (params, point);
        variances_sum += difference * difference;
        ++number_of_interested_labels;
      }
    }
  }
  return (variances_sum / number_of_interested_labels);
}

double
pcl::DEMGroundSegmentation::computeSurfaceHeight (
    const std::vector<double> &params, const pcl::PointDEM &point)
{
  double surface_height = params[0] * point.x * point.x +
                          params[1] * point.z * point.z +
                          params[2] * 2.0 * point.x * point.z +
                          params[3] * point.x +
                          params[4] * point.z +
                          params[5];
  return (surface_height);
}

double
pcl::DEMGroundSegmentation::computeCurbSigmoid (const std::vector<double> &params, 
    const pcl::PointDEM &point, const DEMLabels label, const double steepness)
{
  double point_error = params[0] * point.z * point.z * point.z +
                       params[1] * point.z * point.z +
                       params[2] * point.z +
                       params[3] - point.x;
  if (label == pcl::DEMGroundSegmentation::LEFT_SIDEWALK)
  {
    point_error *= -1.0;
  }
  
  double exponent = exp (steepness * point_error);
  double sigmoid = 2.0 / (1.0 + exponent) - 1.0;
  
  return (sigmoid);
}

double
pcl::DEMGroundSegmentation::computeGaussian (const pcl::PointDEM &point, const double arg)
{
  double difference = arg - point.y;
  double variance = point.height_variance + VARIANCE_ERROR;
  double exponent = std::exp (-difference * difference / variance);
  double coeff = 1.0 / std::sqrt (variance * 
      2.0 * boost::math::constants::pi<double> ());

  return (coeff * exponent);
}
