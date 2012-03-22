/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file tree2.h
 * @brief This file contains the Tree2 structure and the inline <<-operator for it.
 **/

#ifndef LABELSKEL_TREE2_H
#define LABELSKEL_TREE2_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// Our header
#include <pcl/people/label_skeleton/common.h>
#include <pcl/people/label_skeleton/blob2.h>

namespace LabelSkel
{
/**
 * @brief This structure containts all parameters to describe the segmented tree
 */
struct Tree2 {
  //Inline constructor
  inline Tree2() {
    id = NO_CHILD;
    lid = NO_CHILD;
    nr_parts = 0;
    for(int i=0;i<NUM_PARTS;i++)
      parts_lid[i] = NO_CHILD;
  }

  inline Tree2( const Tree2& T) {
    *this = T;
  }

  Tree2& operator = ( const Tree2& T ) {
    id    = T.id;     // unique ID for every tree in the  frame
    label = T.label;  // the root part label of this tree
    lid   = T.lid;    // the label id, namely the number of blob of this label to uniquely identify a tree

    for(int i=0;i<3;++i)       mean(i) = T.mean(i);
    for(int i=0;i<9;++i)        cov(i) = T.cov(i);
    for(int i=0;i<3;++i)   eigenval(i) = T.eigenval(i);
    for(int i=0;i<9;++i)  eigenvect(i) = T.eigenvect(i);

    indices = T.indices;

    for(int i=0;i<3;++i)   min(i) = T.min(i);
    for(int i=0;i<3;++i)   max(i) = T.max(i);

    return *this;
  }

  int     id;                     // specific identification number of this tree
  part_t  label;                  // labels which part the root of this tree is
  int     lid;                    // label id, which number of this type of part is this
  int     nr_parts;               // the number of parts in this tree
  int     parts_lid[NUM_PARTS];   // Indicate the used parts
  float   total_dist_error;       // sum of all distance errors
  float   norm_dist_error;         // total_dist_error/nr_parts

  Eigen::Vector4f  mean;          // mean in xyz
  Eigen::Matrix3f  cov;           // covariance in 3x3 matrix
  Eigen::Vector3f  eigenval;      // eigenvalue of blob
  Eigen::Matrix3f  eigenvect;     // eigenvector of blob

  pcl::PointIndices indices;      // The indices of the pointcloud
  Eigen::Vector4f   min;          // The min of the bounding box
  Eigen::Vector4f   max;          // The max of the bounding box
};

inline std::ostream& operator << (std::ostream& os, const Tree2& t)
{
  os << " Tree2 id " << t.id << " label " << t.label << " lid " << t.lid << " nr_parts " << t.nr_parts << std::endl;
  os << " total_dist_error " << t.total_dist_error << " norm_dist_error " << t.norm_dist_error << std::endl;
  os << " mean " << t.mean(0) << " , " << t.mean(1) << " , " << t.mean(2) << " , " << t.mean(3) << std::endl;
  os << " cov " << std::endl << t.cov << std::endl;
  os << " eigenval " << t.eigenval(0) << " , " << t.eigenval(1) << " , " << t.eigenval(2) << std::endl;
  os << " eigenvect " << std::endl << t.eigenvect << std::endl;
  os << " min " << t.min(0) << " , " << t.min(1) << " , " << t.min(2) << " , " << t.min(3) << std::endl;
  os << " max " << t.max(0) << " , " << t.max(1) << " , " << t.max(2) << " , " << t.max(3) << std::endl;
  os << " indices length " << t.indices.indices.size() << std::endl;
  return (os);
}
} // end namespace LabelSkel

#endif
