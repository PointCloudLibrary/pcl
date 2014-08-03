/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FILTER_IMPL_FIELD_VAL_CONDITION_H_
#define PCL_FILTER_IMPL_FIELD_VAL_CONDITION_H_

#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <pcl/filters/conditional_removal.h>

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::FieldComparison<PointT>::FieldComparison (
    std::string field_name, ComparisonOps::CompareOp op, double compare_val) 
  : ComparisonBase<PointT> ()
  , compare_val_ (compare_val), point_data_ (NULL)
{
  field_name_ = field_name;
  op_ = op;

  // Get all fields
  std::vector<pcl::PCLPointField> point_fields;
  // Use a dummy cloud to get the field types in a clever way
  PointCloud<PointT> dummyCloud;
  pcl::getFields (dummyCloud, point_fields);

  // Find field_name
  if (point_fields.empty ())
  {
    PCL_WARN ("[pcl::FieldComparison::FieldComparison] no fields found!\n");
    capable_ = false;
    return;
  }

  // Get the field index
  size_t d;
  for (d = 0; d < point_fields.size (); ++d)
  {
    if (point_fields[d].name == field_name) 
      break;
  }
  
  if (d == point_fields.size ())
  {
    PCL_WARN ("[pcl::FieldComparison::FieldComparison] field not found!\n");
    capable_ = false;
    return;
  }
  uint8_t datatype = point_fields[d].datatype;
  uint32_t offset = point_fields[d].offset;

  point_data_ = new PointDataAtOffset<PointT>(datatype, offset);
  capable_ = true;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::FieldComparison<PointT>::~FieldComparison () 
{
  if (point_data_ != NULL)
  {
    delete point_data_;
    point_data_ = NULL;
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::FieldComparison<PointT>::evaluate (const PointT &point) const
{
  if (!this->capable_)
  {
    PCL_WARN ("[pcl::FieldComparison::evaluate] invalid compariosn!\n");
    return (false);
  }

  // if p(data) > val then compare_result = 1
  // if p(data) == val then compare_result = 0
  // if p(data) <  ival then compare_result = -1
  int compare_result = point_data_->compare (point, compare_val_);
  
  switch (this->op_)
  {
    case pcl::ComparisonOps::GT :
      return (compare_result > 0);
    case pcl::ComparisonOps::GE :
      return (compare_result >= 0);
    case pcl::ComparisonOps::LT :
      return (compare_result < 0);
    case pcl::ComparisonOps::LE :
      return (compare_result <= 0);
    case pcl::ComparisonOps::EQ :
      return (compare_result == 0);
    default:
      PCL_WARN ("[pcl::FieldComparison::evaluate] unrecognized op_!\n");
      return (false);
  }
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PackedRGBComparison<PointT>::PackedRGBComparison (
    std::string component_name, ComparisonOps::CompareOp op, double compare_val) :
  component_name_ (component_name), component_offset_ (), compare_val_ (compare_val)
{
  // get all the fields
  std::vector<pcl::PCLPointField> point_fields;
  // Use a dummy cloud to get the field types in a clever way
  PointCloud<PointT> dummyCloud;
  pcl::getFields (dummyCloud, point_fields);

  // Locate the "rgb" field
  size_t d;
  for (d = 0; d < point_fields.size (); ++d)
  {
    if (point_fields[d].name == "rgb" || point_fields[d].name == "rgba")
      break;
  }
  if (d == point_fields.size ())
  {
    PCL_WARN ("[pcl::PackedRGBComparison::PackedRGBComparison] rgb field not found!\n");
    capable_ = false;
    return;
  }

  // Verify the datatype
  uint8_t datatype = point_fields[d].datatype;
  if (datatype != pcl::PCLPointField::FLOAT32 &&
      datatype != pcl::PCLPointField::UINT32 &&
      datatype != pcl::PCLPointField::INT32)
  {
    PCL_WARN ("[pcl::PackedRGBComparison::PackedRGBComparison] has unusable type!\n");
    capable_ = false;
    return;
  }

  // Verify the component name
  if (component_name == "r")
  {
    component_offset_ = point_fields[d].offset + 2;
  }
  else if (component_name == "g")
  {
    component_offset_ = point_fields[d].offset + 1;
  }
  else if (component_name == "b")
  {
    component_offset_ = point_fields[d].offset;
  }
  else
  {
    PCL_WARN ("[pcl::PackedRGBComparison::PackedRGBComparison] unrecognized component name!\n");
    capable_ = false;
    return;
  }

  // save the rest of the context
  capable_ = true;
  op_ = op;
}


//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PackedRGBComparison<PointT>::evaluate (const PointT &point) const
{
  // extract the component value
  const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&point);
  uint8_t my_val = *(pt_data + component_offset_);

  // now do the comparison
  switch (this->op_) 
  {
    case pcl::ComparisonOps::GT :
      return (my_val > this->compare_val_);
    case pcl::ComparisonOps::GE :
      return (my_val >= this->compare_val_);
    case pcl::ComparisonOps::LT :
      return (my_val < this->compare_val_);
    case pcl::ComparisonOps::LE :
      return (my_val <= this->compare_val_);
    case pcl::ComparisonOps::EQ :
      return (my_val == this->compare_val_);
    default:
      PCL_WARN ("[pcl::PackedRGBComparison::evaluate] unrecognized op_!\n");
      return (false);
  }
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::PackedHSIComparison<PointT>::PackedHSIComparison (
    std::string component_name, ComparisonOps::CompareOp op, double compare_val) : 
  component_name_ (component_name), component_id_ (), compare_val_ (compare_val), rgb_offset_ ()
{
  // Get all the fields
  std::vector<pcl::PCLPointField> point_fields;
  // Use a dummy cloud to get the field types in a clever way
  PointCloud<PointT> dummyCloud;
  pcl::getFields (dummyCloud, point_fields);

  // Locate the "rgb" field
  size_t d;
  for (d = 0; d < point_fields.size (); ++d)
    if (point_fields[d].name == "rgb" || point_fields[d].name == "rgba") 
      break;
  if (d == point_fields.size ())
  {
    PCL_WARN ("[pcl::PackedHSIComparison::PackedHSIComparison] rgb field not found!\n");
    capable_ = false;
    return;
  }

  // Verify the datatype
  uint8_t datatype = point_fields[d].datatype;
  if (datatype != pcl::PCLPointField::FLOAT32 &&
      datatype != pcl::PCLPointField::UINT32 &&
      datatype != pcl::PCLPointField::INT32)
  {
    PCL_WARN ("[pcl::PackedHSIComparison::PackedHSIComparison] has unusable type!\n");
    capable_ = false;
    return;
  }

  // verify the offset
  uint32_t offset = point_fields[d].offset;
  if (offset % 4 != 0)
  {
    PCL_WARN ("[pcl::PackedHSIComparison::PackedHSIComparison] rgb field is not 32 bit aligned!\n");
    capable_ = false;
    return;
  }
  rgb_offset_ = point_fields[d].offset;

  // verify the component name
  if (component_name == "h" ) 
  {
    component_id_ = H;
  } 
  else if (component_name == "s") 
  {
    component_id_ = S;
  } 
  else if (component_name == "i") 
  { 
    component_id_ = I;
  } 
  else 
  {
    PCL_WARN ("[pcl::PackedRGBComparison::PackedRGBComparison] unrecognized component name!\n");
    capable_ = false;
    return;
  }

  // Save the context
  capable_ = true;
  op_ = op;
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::PackedHSIComparison<PointT>::evaluate (const PointT &point) const
{
  // Since this is a const function, we can't make these data members because we change them here
  static uint32_t rgb_val_ = 0;
  static uint8_t r_ = 0;
  static uint8_t g_ = 0;
  static uint8_t b_ = 0;
  static int8_t h_ = 0;
  static uint8_t s_ = 0;
  static uint8_t i_ = 0;

  // We know that rgb data is 32 bit aligned (verified in the ctor) so...
  const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&point);
  const uint32_t* rgb_data = reinterpret_cast<const uint32_t*> (pt_data + rgb_offset_);
  uint32_t new_rgb_val = *rgb_data;

  if (rgb_val_ != new_rgb_val) 
  { // avoid having to redo this calc, if possible
    rgb_val_ = new_rgb_val;
    // extract r,g,b
    r_ = static_cast <uint8_t> (rgb_val_ >> 16); 
    g_ = static_cast <uint8_t> (rgb_val_ >> 8);
    b_ = static_cast <uint8_t> (rgb_val_);

    // definitions taken from http://en.wikipedia.org/wiki/HSL_and_HSI
    float hx = (2.0f * r_ - g_ - b_) / 4.0f;  // hue x component -127 to 127
    float hy = static_cast<float> (g_ - b_) * 111.0f / 255.0f; // hue y component -111 to 111
    h_ = static_cast<int8_t> (atan2(hy, hx) * 128.0f / M_PI);

    int32_t i = (r_+g_+b_)/3; // 0 to 255
    i_ = static_cast<uint8_t> (i);

    int32_t m;  // min(r,g,b)
    m = (r_ < g_) ? r_ : g_;
    m = (m < b_) ? m : b_;

    s_ = static_cast<uint8_t> ((i == 0) ? 0 : 255 - (m * 255) / i); // saturation 0 to 255
  }

  float my_val = 0;

  switch (component_id_) 
  {
    case H:
      my_val = static_cast <float> (h_);
      break;
    case S:
      my_val = static_cast <float> (s_);
      break;
    case I:
      my_val = static_cast <float> (i_);
      break;
    default:
      assert (false);
  }

  // now do the comparison
  switch (this->op_) 
  {
    case pcl::ComparisonOps::GT :
      return (my_val > this->compare_val_);
    case pcl::ComparisonOps::GE :
      return (my_val >= this->compare_val_);
    case pcl::ComparisonOps::LT :
      return (my_val < this->compare_val_);
    case pcl::ComparisonOps::LE :
      return (my_val <= this->compare_val_);
    case pcl::ComparisonOps::EQ :
      return (my_val == this->compare_val_);
    default:
      PCL_WARN ("[pcl::PackedHSIComparison::evaluate] unrecognized op_!\n");
      return (false);
  }
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::TfQuadraticXYZComparison<PointT>::TfQuadraticXYZComparison () :
  comp_matr_ (), comp_vect_ (), comp_scalar_ (0.0)
{
  // get all the fields
  std::vector<pcl::PCLPointField> point_fields;
  // Use a dummy cloud to get the field types in a clever way
  PointCloud<PointT> dummyCloud;
  pcl::getFields (dummyCloud, point_fields);

  // Locate the "x" field
  size_t dX;
  for (dX = 0; dX < point_fields.size (); ++dX)
  {
    if (point_fields[dX].name == "x")
      break;
  }
  if (dX == point_fields.size ())
  {
    PCL_WARN ("[pcl::TfQuadraticXYZComparison::TfQuadraticXYZComparison] x field not found!\n");
    capable_ = false;
    return;
  }

  // Locate the "y" field
  size_t dY;
  for (dY = 0; dY < point_fields.size (); ++dY)
  {
    if (point_fields[dY].name == "y")
      break;
  }
  if (dY == point_fields.size ())
  {
    PCL_WARN ("[pcl::TfQuadraticXYZComparison::TfQuadraticXYZComparison] y field not found!\n");
    capable_ = false;
    return;
  }

  // Locate the "z" field
  size_t dZ;
  for (dZ = 0; dZ < point_fields.size (); ++dZ)
  {
    if (point_fields[dZ].name == "z")
      break;
  }
  if (dZ == point_fields.size ())
  {
    PCL_WARN ("[pcl::TfQuadraticXYZComparison::TfQuadraticXYZComparison] z field not found!\n");
    capable_ = false;
    return;
  }

  comp_matr_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  comp_vect_ << 0.0, 0.0, 0.0, 1.0;
  tf_comp_matr_ = comp_matr_;
  tf_comp_vect_ = comp_vect_;
  op_ = pcl::ComparisonOps::EQ;
  capable_ = true;
}

//////////////////////////////////////////////////////////////////////////
template<typename PointT>
pcl::TfQuadraticXYZComparison<PointT>::TfQuadraticXYZComparison (const pcl::ComparisonOps::CompareOp op,
                                                                 const Eigen::Matrix3f &comparison_matrix,
                                                                 const Eigen::Vector3f &comparison_vector,
                                                                 const float &comparison_scalar,
                                                                 const Eigen::Affine3f &comparison_transform) :
  comp_matr_ (), comp_vect_ (), comp_scalar_ (comparison_scalar)
{
  // get all the fields
  std::vector<pcl::PCLPointField> point_fields;
  // Use a dummy cloud to get the field types in a clever way
  PointCloud<PointT> dummyCloud;
  pcl::getFields (dummyCloud, point_fields);

  // Locate the "x" field
  size_t dX;
  for (dX = 0; dX < point_fields.size (); ++dX)
  {
    if (point_fields[dX].name == "x")
      break;
  }
  if (dX == point_fields.size ())
  {
    PCL_WARN ("[pcl::TfQuadraticXYZComparison::TfQuadraticXYZComparison] x field not found!\n");
    capable_ = false;
    return;
  }

  // Locate the "y" field
  size_t dY;
  for (dY = 0; dY < point_fields.size (); ++dY)
  {
    if (point_fields[dY].name == "y")
      break;
  }
  if (dY == point_fields.size ())
  {
    PCL_WARN ("[pcl::TfQuadraticXYZComparison::TfQuadraticXYZComparison] y field not found!\n");
    capable_ = false;
    return;
  }

  // Locate the "z" field
  size_t dZ;
  for (dZ = 0; dZ < point_fields.size (); ++dZ)
  {
    if (point_fields[dZ].name == "z")
      break;
  }
  if (dZ == point_fields.size ())
  {
    PCL_WARN ("[pcl::TfQuadraticXYZComparison::TfQuadraticXYZComparison] z field not found!\n");
    capable_ = false;
    return;
  }

  capable_ = true;
  op_ = op;
  setComparisonMatrix (comparison_matrix);
  setComparisonVector (comparison_vector);
  if (!comparison_transform.matrix ().isIdentity ())
    transformComparison (comparison_transform);
}

//////////////////////////////////////////////////////////////////////////
template<typename PointT>
bool
pcl::TfQuadraticXYZComparison<PointT>::evaluate (const PointT &point) const
{
  Eigen::Vector4f pointAffine;
  pointAffine << point.x, point.y, point.z, 1; 
  
  float myVal = static_cast<float>(2.0f * tf_comp_vect_.transpose () * pointAffine) + static_cast<float>(pointAffine.transpose () * tf_comp_matr_ * pointAffine) + comp_scalar_ - 3.0f;
  
  // now do the comparison
  switch (this->op_)
  {
    case pcl::ComparisonOps::GT:
      return (myVal > 0);
    case pcl::ComparisonOps::GE:
      return (myVal >= 0);
    case pcl::ComparisonOps::LT:
      return (myVal < 0);
    case pcl::ComparisonOps::LE:
      return (myVal <= 0);
    case pcl::ComparisonOps::EQ:
      return (myVal == 0);
    default:
      PCL_WARN ("[pcl::transformableQuadricXYZComparison::evaluate] unrecognized op_!\n");
      return (false);
  }
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PointDataAtOffset<PointT>::compare (const PointT& p, const double& val) 
{
  // if p(data) > val return 1
  // if p(data) == val return 0
  // if p(data) < val return -1 
  
  const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&p);

  switch (datatype_) 
  {
    case pcl::PCLPointField::INT8 :
    {
      int8_t pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (int8_t));
      return (pt_val > static_cast<int8_t>(val)) - (pt_val < static_cast<int8_t> (val));
    }
    case pcl::PCLPointField::UINT8 :
    {
      uint8_t pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (uint8_t));
      return (pt_val > static_cast<uint8_t>(val)) - (pt_val < static_cast<uint8_t> (val));
    }
    case pcl::PCLPointField::INT16 :
    {
      int16_t pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (int16_t));
      return (pt_val > static_cast<int16_t>(val)) - (pt_val < static_cast<int16_t> (val));
    }
    case pcl::PCLPointField::UINT16 :
    {
      uint16_t pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (uint16_t));
      return (pt_val > static_cast<uint16_t> (val)) - (pt_val < static_cast<uint16_t> (val));
    }
    case pcl::PCLPointField::INT32 :
    {
      int32_t pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (int32_t));
      return (pt_val > static_cast<int32_t> (val)) - (pt_val < static_cast<int32_t> (val));
    }
    case pcl::PCLPointField::UINT32 :
    {
      uint32_t pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (uint32_t));
      return (pt_val > static_cast<uint32_t> (val)) - (pt_val < static_cast<uint32_t> (val));
    }
    case pcl::PCLPointField::FLOAT32 :
    {
      float pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (float));
      return (pt_val > static_cast<float> (val)) - (pt_val < static_cast<float> (val));
    }
    case pcl::PCLPointField::FLOAT64 :
    {
      double pt_val;
      memcpy (&pt_val, pt_data + this->offset_, sizeof (double));
      return (pt_val > val) - (pt_val < val);
    }
    default : 
      PCL_WARN ("[pcl::pcl::PointDataAtOffset::compare] unknown data_type!\n");
      return (0);
  }
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::ConditionBase<PointT>::addComparison (ComparisonBaseConstPtr comparison)
{
  if (!comparison->isCapable ())
    capable_ = false;
  comparisons_.push_back (comparison);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::ConditionBase<PointT>::addCondition (Ptr condition)
{
  if (!condition->isCapable ())
    capable_ = false;
  conditions_.push_back (condition);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::ConditionAnd<PointT>::evaluate (const PointT &point) const
{
  for (size_t i = 0; i < comparisons_.size (); ++i)
    if (!comparisons_[i]->evaluate (point))
      return (false);

  for (size_t i = 0; i < conditions_.size (); ++i)
    if (!conditions_[i]->evaluate (point))
      return (false);

  return (true);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool 
pcl::ConditionOr<PointT>::evaluate (const PointT &point) const
{
  if (comparisons_.empty () && conditions_.empty ()) 
    return (true);
  for (size_t i = 0; i < comparisons_.size (); ++i)
    if (comparisons_[i]->evaluate(point))
      return (true);

  for (size_t i = 0; i < conditions_.size (); ++i)
    if (conditions_[i]->evaluate (point))
      return (true);

  return (false);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::ConditionalRemoval<PointT>::setCondition (ConditionBasePtr condition)
{
  condition_ = condition;
  capable_ = condition_->isCapable ();
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::ConditionalRemoval<PointT>::applyFilter (PointCloud &output)
{
  if (capable_ == false)
  {
    PCL_WARN ("[pcl::%s::applyFilter] not capable!\n", getClassName ().c_str ());
    return;
  }
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    return;
  }

  if (condition_.get () == NULL) 
  {
    PCL_WARN ("[pcl::%s::applyFilter] No filtering condition given!\n", getClassName ().c_str ());
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.header       = input_->header;
  if (!keep_organized_)
  {
    output.height    = 1;   // filtering breaks the organized structure
    output.is_dense  = true;
  } 
  else 
  {
    output.height   = this->input_->height;
    output.width    = this->input_->width;
    output.is_dense = this->input_->is_dense;
  }
  output.points.resize (input_->points.size ());
  removed_indices_->resize (input_->points.size ());

  int nr_p = 0;
  int nr_removed_p = 0;

  if (!keep_organized_)
  {
    for (size_t cp = 0; cp < Filter<PointT>::indices_->size (); ++cp)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (input_->points[(*Filter < PointT > ::indices_)[cp]].x)
          || !pcl_isfinite (input_->points[(*Filter < PointT > ::indices_)[cp]].y)
          || !pcl_isfinite (input_->points[(*Filter < PointT > ::indices_)[cp]].z))
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = (*Filter<PointT>::indices_)[cp];
          nr_removed_p++;
        }
        continue;
      }

      if (condition_->evaluate (input_->points[(*Filter < PointT > ::indices_)[cp]]))
      {
        copyPoint (input_->points[(*Filter < PointT > ::indices_)[cp]], output.points[nr_p]);
        nr_p++;
      }
      else
      {
        if (extract_removed_indices_)
        {
          (*removed_indices_)[nr_removed_p] = (*Filter<PointT>::indices_)[cp];
          nr_removed_p++;
        }
      }
    }

    output.width = nr_p;
    output.points.resize (nr_p);
  }
  else
  {
    std::vector<int> indices = *Filter<PointT>::indices_;
    std::sort (indices.begin (), indices.end ());   //TODO: is this necessary or can we assume the indices to be sorted?
    bool removed_p = false;
    size_t ci = 0;
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      if (cp == static_cast<size_t> (indices[ci]))
      {
        if (ci < indices.size () - 1)
        {
          ci++;
          if (cp == static_cast<size_t> (indices[ci]))   //check whether the next index will have the same value. TODO: necessary?
            continue;
        }

        // copy all the fields
        copyPoint (input_->points[cp], output.points[cp]);

        if (!condition_->evaluate (input_->points[cp]))
        {
          output.points[cp].getVector4fMap ().setConstant (user_filter_value_);
          removed_p = true;

          if (extract_removed_indices_)
          {
            (*removed_indices_)[nr_removed_p] = static_cast<int> (cp);
            nr_removed_p++;
          }
        }
      }
      else
      {
        output.points[cp].getVector4fMap ().setConstant (user_filter_value_);
        removed_p = true;
        //as for !keep_organized_: removed points due to setIndices are not considered as removed_indices_
      }
    }

    if (removed_p && !pcl_isfinite (user_filter_value_))
      output.is_dense = false;
  }
  removed_indices_->resize (nr_removed_p);
}

#define PCL_INSTANTIATE_PointDataAtOffset(T) template class PCL_EXPORTS pcl::PointDataAtOffset<T>;
#define PCL_INSTANTIATE_ComparisonBase(T) template class PCL_EXPORTS pcl::ComparisonBase<T>;
#define PCL_INSTANTIATE_FieldComparison(T) template class PCL_EXPORTS pcl::FieldComparison<T>;
#define PCL_INSTANTIATE_PackedRGBComparison(T) template class PCL_EXPORTS pcl::PackedRGBComparison<T>;
#define PCL_INSTANTIATE_PackedHSIComparison(T) template class PCL_EXPORTS pcl::PackedHSIComparison<T>;
#define PCL_INSTANTIATE_TfQuadraticXYZComparison(T) template class PCL_EXPORTS pcl::TfQuadraticXYZComparison<T>;
#define PCL_INSTANTIATE_ConditionBase(T) template class PCL_EXPORTS pcl::ConditionBase<T>;
#define PCL_INSTANTIATE_ConditionAnd(T) template class PCL_EXPORTS pcl::ConditionAnd<T>;
#define PCL_INSTANTIATE_ConditionOr(T) template class PCL_EXPORTS pcl::ConditionOr<T>;
#define PCL_INSTANTIATE_ConditionalRemoval(T) template class PCL_EXPORTS pcl::ConditionalRemoval<T>;

#endif 
