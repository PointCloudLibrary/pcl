/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/common/common.h>
#include <pcl/ml/feature_handler.h>
#include <pcl/ml/multi_channel_2d_comparison_feature.h>
#include <pcl/ml/multi_channel_2d_data_set.h>
#include <pcl/ml/multiple_data_2d_example_index.h>
#include <pcl/ml/point_xy_32f.h>
#include <pcl/ml/point_xy_32i.h>

#include <istream>
#include <ostream>

namespace pcl {

/** Feature utility class that handles the creation and evaluation of RGBD
 *  comparison features. */
template <class DATA_TYPE, std::size_t NUM_OF_CHANNELS>
class PCL_EXPORTS MultiChannel2DComparisonFeatureHandler
: public pcl::FeatureHandler<pcl::MultiChannel2DComparisonFeature<pcl::PointXY32i>,
                             pcl::MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>,
                             pcl::MultipleData2DExampleIndex> {

public:
  /** Constructor. */
  MultiChannel2DComparisonFeatureHandler(const int feature_window_width,
                                         const int feature_window_height)
  : feature_window_width_(feature_window_width)
  , feature_window_height_(feature_window_height)
  {}

  /** Destructor. */
  virtual ~MultiChannel2DComparisonFeatureHandler() {}

  /** Sets the feature window size.
   *
   * \param[in] width the width of the feature window
   * \param[in] height the height of the feature window
   */
  inline void
  setFeatureWindowSize(int width, int height)
  {
    feature_window_width_ = width;
    feature_window_height_ = height;
  }

  /** Creates random features.
   *
   * \param[in] num_of_features the number of random features to create
   * \param[out] features the destination for the created random features
   */
  inline void
  createRandomFeatures(
      const std::size_t num_of_features,
      std::vector<MultiChannel2DComparisonFeature<PointXY32i>>& features)
  {
    features.resize(num_of_features);
    for (std::size_t feature_index = 0; feature_index < num_of_features;
         ++feature_index) {
      features[feature_index].p1 = PointXY32i::randomPoint(-feature_window_width_ / 2,
                                                           feature_window_width_ / 2,
                                                           -feature_window_height_ / 2,
                                                           feature_window_height_ / 2);
      features[feature_index].p2 = PointXY32i::randomPoint(-feature_window_width_ / 2,
                                                           feature_window_width_ / 2,
                                                           -feature_window_height_ / 2,
                                                           feature_window_height_ / 2);
      features[feature_index].channel = static_cast<unsigned char>(
          NUM_OF_CHANNELS * (static_cast<float>(rand()) / (RAND_MAX + 1)));
    }
  }

  /** Evaluates a feature for a set of examples on the specified data set.
   *
   * \param[in] feature the feature to evaluate
   * \param[in] data_set the data set the feature is evaluated on
   * \param[in] examples the examples the feature is evaluated for
   * \param[out] results the destination for the evaluation results
   * \param[out] flags the destination for the flags corresponding to the evaluation
   *             results
   */
  inline void
  evaluateFeature(const MultiChannel2DComparisonFeature<PointXY32i>& feature,
                  MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>& data_set,
                  std::vector<MultipleData2DExampleIndex>& examples,
                  std::vector<float>& results,
                  std::vector<unsigned char>& flags) const
  {
    results.resize(examples.size());
    flags.resize(examples.size());
    for (int example_index = 0; example_index < examples.size(); ++example_index) {
      const MultipleData2DExampleIndex& example = examples[example_index];

      evaluateFeature(
          feature, data_set, example, results[example_index], flags[example_index]);
    }
  }

  /** Evaluates a feature for one examples on the specified data set.
   *
   * \param[in] feature the feature to evaluate
   * \param[in] data_set the data set the feature is evaluated on
   * \param[in] example the example the feature is evaluated for
   * \param[out] result the destination for the evaluation result
   * \param[out] flag the destination for the flag corresponding to the evaluation
   *             result
   */
  inline void
  evaluateFeature(const MultiChannel2DComparisonFeature<PointXY32i>& feature,
                  MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>& data_set,
                  const MultipleData2DExampleIndex& example,
                  float& result,
                  unsigned char& flag) const
  {
    const int center_col_index = example.x;
    const int center_row_index = example.y;

    const std::size_t p1_col =
        static_cast<std::size_t>(feature.p1.x + center_col_index);
    const std::size_t p1_row =
        static_cast<std::size_t>(feature.p1.y + center_row_index);

    const std::size_t p2_col =
        static_cast<std::size_t>(feature.p2.x + center_col_index);
    const std::size_t p2_row =
        static_cast<std::size_t>(feature.p2.y + center_row_index);

    const unsigned char channel = feature.channel;

    const float value1 =
        static_cast<float>(data_set(example.data_set_id, p1_col, p1_row)[channel]);
    const float value2 =
        static_cast<float>(data_set(example.data_set_id, p2_col, p2_row)[channel]);

    result = value1 - value2;
    flag = (std::isfinite(value1) && std::isfinite(value2)) ? 0 : 1;
  }

  /** Generates code for feature evaluation.
   *
   * \param[in] feature the feature for which code is generated
   * \param[out] stream the destination for the generated code
   */
  void
  generateCodeForEvaluation(const MultiChannel2DComparisonFeature<PointXY32i>& feature,
                            std::ostream& stream) const
  {
    stream << "ERROR: RegressionVarianceStatsEstimator does not implement "
              "generateCodeForBranchIndex(...)";
    // stream << "const float value = ( (*dataSet)(dataSetId, centerY+" << feature.p1.y
    // << ", centerX+" << feature.p1.x << ")[" << static_cast<int>(feature.colorChannel)
    // << "]"
    //  << " - " << "(*dataSet)(dataSetId, centerY+" << feature.p2.y << ", centerX+" <<
    //  feature.p2.x << ")[" << static_cast<int>(feature.colorChannel) << "] );" <<
    //  ::std::endl;
  }

private:
  /** The width of the feature window. */
  int feature_window_width_;
  /** The height of the feature window. */
  int feature_window_height_;
};

/** Feature utility class that handles the creation and evaluation of RGBD
 *  comparison features. */
template <class DATA_TYPE,
          std::size_t NUM_OF_CHANNELS,
          std::size_t SCALE_CHANNEL,
          bool INVERT_SCALE>
class PCL_EXPORTS ScaledMultiChannel2DComparisonFeatureHandler
: public pcl::FeatureHandler<pcl::MultiChannel2DComparisonFeature<pcl::PointXY32f>,
                             pcl::MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>,
                             pcl::MultipleData2DExampleIndex> {

public:
  /** Constructor. */
  ScaledMultiChannel2DComparisonFeatureHandler(const int feature_window_width,
                                               const int feature_window_height)
  : feature_window_width_(feature_window_width)
  , feature_window_height_(feature_window_height)
  {}

  /** Destructor. */
  virtual ~ScaledMultiChannel2DComparisonFeatureHandler() {}

  /** Sets the feature window size.
   *
   * \param[in] width the width of the feature window
   * \param[in] height the height of the feature window
   */
  inline void
  setFeatureWindowSize(int width, int height)
  {
    feature_window_width_ = width;
    feature_window_height_ = height;
  }

  /** Creates random features.
   *
   * \param[in] num_of_features the number of random features to create
   * \param[out] features the destination for the created random features
   */
  inline void
  createRandomFeatures(
      const std::size_t num_of_features,
      std::vector<MultiChannel2DComparisonFeature<PointXY32f>>& features)
  {
    features.resize(num_of_features);
    for (std::size_t feature_index = 0; feature_index < num_of_features;
         ++feature_index) {
      features[feature_index].p1 = PointXY32f::randomPoint(-feature_window_width_ / 2,
                                                           feature_window_width_ / 2,
                                                           -feature_window_height_ / 2,
                                                           feature_window_height_ / 2);
      features[feature_index].p2 = PointXY32f::randomPoint(-feature_window_width_ / 2,
                                                           feature_window_width_ / 2,
                                                           -feature_window_height_ / 2,
                                                           feature_window_height_ / 2);
      features[feature_index].channel = static_cast<unsigned char>(
          NUM_OF_CHANNELS * (static_cast<float>(rand()) / (RAND_MAX + 1)));
    }
  }

  /** Evaluates a feature for a set of examples on the specified data set.
   *
   * \param[in] feature the feature to evaluate
   * \param[in] data_set the data set the feature is evaluated on
   * \param[in] examples the examples the feature is evaluated for
   * \param[out] results the destination for the evaluation results
   * \param[out] flags the destination for the flags corresponding to the evaluation
   *             results
   */
  inline void
  evaluateFeature(const MultiChannel2DComparisonFeature<PointXY32f>& feature,
                  MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>& data_set,
                  std::vector<MultipleData2DExampleIndex>& examples,
                  std::vector<float>& results,
                  std::vector<unsigned char>& flags) const
  {
    results.resize(examples.size());
    flags.resize(examples.size());
    for (int example_index = 0; example_index < examples.size(); ++example_index) {
      const MultipleData2DExampleIndex& example = examples[example_index];

      evaluateFeature(
          feature, data_set, example, results[example_index], flags[example_index]);
    }
  }

  /** Evaluates a feature for one examples on the specified data set.
   *
   * \param[in] feature the feature to evaluate
   * \param[in] data_set the data set the feature is evaluated on
   * \param[in] example the example the feature is evaluated for
   * \param[out] result the destination for the evaluation result
   * \param[out] flag the destination for the flag corresponding to the evaluation
   *             result
   */
  inline void
  evaluateFeature(const MultiChannel2DComparisonFeature<PointXY32f>& feature,
                  MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>& data_set,
                  const MultipleData2DExampleIndex& example,
                  float& result,
                  unsigned char& flag) const
  {
    const int center_col_index = example.x;
    const int center_row_index = example.y;

    float scale;
    if (INVERT_SCALE)
      scale = 1.0f / static_cast<float>(data_set(example.data_set_id,
                                                 center_col_index,
                                                 center_row_index)[SCALE_CHANNEL]);
    else
      scale = static_cast<float>(data_set(
          example.data_set_id, center_col_index, center_row_index)[SCALE_CHANNEL]);

    const std::size_t p1_col =
        static_cast<std::size_t>(scale * feature.p1.x + center_col_index);
    const std::size_t p1_row =
        static_cast<std::size_t>(scale * feature.p1.y + center_row_index);

    const std::size_t p2_col =
        static_cast<std::size_t>(scale * feature.p2.x + center_col_index);
    const std::size_t p2_row =
        static_cast<std::size_t>(scale * feature.p2.y + center_row_index);

    const unsigned char channel = feature.channel;

    const float value1 =
        static_cast<float>(data_set(example.data_set_id, p1_col, p1_row)[channel]);
    const float value2 =
        static_cast<float>(data_set(example.data_set_id, p2_col, p2_row)[channel]);

    result = value1 - value2;
    flag = (std::isfinite(value1) && std::isfinite(value2)) ? 0 : 1;
  }

  /** Generates code for feature evaluation.
   *
   * \param[in] feature the feature for which code is generated
   * \param[out] stream the destination for the generated code
   */
  void
  generateCodeForEvaluation(const MultiChannel2DComparisonFeature<PointXY32f>& feature,
                            std::ostream& stream) const
  {
    stream << "ERROR: ScaledMultiChannel2DComparisonFeatureHandler does not implement "
              "generateCodeForBranchIndex(...)"
           << std::endl;

    // pcl::PointXY32f p1 = feature.p1;
    // pcl::PointXY32f p2 = feature.p2;

    // stream << "const float eval_value = data_ptr + " << p1.x << " + " << p1.y << " *
    // width;

    // stream << "const float value = ( (*dataSet)(dataSetId, centerY+" << feature.p1.y
    // << ", centerX+" << feature.p1.x << ")[" << static_cast<int>(feature.colorChannel)
    // << "]"
    //  << " - " << "(*dataSet)(dataSetId, centerY+" << feature.p2.y << ", centerX+" <<
    //  feature.p2.x << ")[" << static_cast<int>(feature.colorChannel) << "] );" <<
    //  ::std::endl;
  }

private:
  /** The width of the feature window. */
  int feature_window_width_;
  /** The height of the feature window. */
  int feature_window_height_;
};

template <class DATA_TYPE,
          std::size_t NUM_OF_CHANNELS,
          std::size_t SCALE_CHANNEL,
          bool INVERT_SCALE>
class PCL_EXPORTS ScaledMultiChannel2DComparisonFeatureHandlerCCodeGenerator
: public pcl::FeatureHandlerCodeGenerator<
      pcl::MultiChannel2DComparisonFeature<pcl::PointXY32f>,
      pcl::MultiChannel2DDataSet<DATA_TYPE, NUM_OF_CHANNELS>,
      pcl::MultipleData2DExampleIndex> {
public:
  ScaledMultiChannel2DComparisonFeatureHandlerCCodeGenerator() {}
  virtual ~ScaledMultiChannel2DComparisonFeatureHandlerCCodeGenerator() {}

  void
  generateEvalFunctionCode(std::ostream& stream) const;

  void
  generateEvalCode(const MultiChannel2DComparisonFeature<PointXY32f>& feature,
                   std::ostream& stream) const;
};

template <class DATA_TYPE,
          std::size_t NUM_OF_CHANNELS,
          std::size_t SCALE_CHANNEL,
          bool INVERT_SCALE>
void
ScaledMultiChannel2DComparisonFeatureHandlerCCodeGenerator<
    DATA_TYPE,
    NUM_OF_CHANNELS,
    SCALE_CHANNEL,
    INVERT_SCALE>::generateEvalFunctionCode(std::ostream& stream) const
{
  if (NUM_OF_CHANNELS == 1 && SCALE_CHANNEL == 0 && INVERT_SCALE) {
    stream << "const float scale  = 1.0f / static_cast<float> (*data_ptr);"
           << std::endl;
    stream << "" << std::endl;
    stream << "struct LocalFeatureHandler" << std::endl;
    stream << "{" << std::endl;
    stream << "  static inline void eval (" << typeid(DATA_TYPE).name()
           << " * a_ptr, const float a_x1, const float a_y1, const float a_x2, const "
              "float a_y2, const float a_scale, const int a_width, float & a_result, "
              "unsigned char & a_flags)"
           << std::endl;
    stream << "  {" << std::endl;
    stream << "    a_result = *(a_ptr + static_cast<int> (a_scale*a_x1) + "
              "(static_cast<int> (a_scale*a_y1)*a_width)) - *(a_ptr + static_cast<int> "
              "(a_scale*a_x2) + (static_cast<int> (a_scale*a_y2)*a_width));"
           << std::endl;
    stream << "  }" << std::endl;
    stream << "};" << std::endl;
  }
  else {
    stream << "ERROR: generateEvalFunctionCode not implemented" << std::endl;
  }
}

template <class DATA_TYPE,
          std::size_t NUM_OF_CHANNELS,
          std::size_t SCALE_CHANNEL,
          bool INVERT_SCALE>
void
ScaledMultiChannel2DComparisonFeatureHandlerCCodeGenerator<DATA_TYPE,
                                                           NUM_OF_CHANNELS,
                                                           SCALE_CHANNEL,
                                                           INVERT_SCALE>::
    generateEvalCode(const MultiChannel2DComparisonFeature<PointXY32f>& feature,
                     std::ostream& stream) const
{
  stream << "LocalFeatureHandler::eval (data_ptr, " << feature.p1.x << ", "
         << feature.p1.y << ", " << feature.p2.x << ", " << feature.p2.y << ", "
         << "scale, width, result, flags);" << std::endl;
}

using Depth2DComparisonFeatureHandler =
    MultiChannel2DComparisonFeatureHandler<float, 1>;
using IntensityDepth2DComparisonFeatureHandler =
    MultiChannel2DComparisonFeatureHandler<float, 2>;
using RGB2DComparisonFeatureHandler = MultiChannel2DComparisonFeatureHandler<float, 3>;
using RGBD2DComparisonFeatureHandler = MultiChannel2DComparisonFeatureHandler<float, 4>;

using ScaledDepth2DComparisonFeatureHandler =
    ScaledMultiChannel2DComparisonFeatureHandler<float, 1, 0, true>;
using ScaledIntensityDepth2DComparisonFeatureHandler =
    ScaledMultiChannel2DComparisonFeatureHandler<float, 2, 1, true>;
using ScaledRGBD2DComparisonFeatureHandler =
    ScaledMultiChannel2DComparisonFeatureHandler<float, 4, 3, true>;

using ScaledDepth2DComparisonFeatureHandlerCCodeGenerator =
    ScaledMultiChannel2DComparisonFeatureHandlerCCodeGenerator<float, 1, 0, true>;

} // namespace pcl
