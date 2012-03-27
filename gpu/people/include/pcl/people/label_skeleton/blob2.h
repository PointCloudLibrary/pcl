/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file blob2.h
 * @brief This file contains the Blob2 structure and the inline <<-operator for it
 **/

#ifndef PCL_PEOPLE_LABEL_SKELETON_BLOB2_H_
#define PCL_PEOPLE_LABEL_SKELETON_BLOB2_H_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// Our header
#include <pcl/people/label_skeleton/common.h>

namespace pcl
{
  namespace people
  {
    namespace label_skeleton
    {
      /**
       * @brief This structure containts all parameters to describe blobs and their parent/child relations
       * @todo: clean this out in the end, perhaps place the children in a separate struct
       */
      struct Blob2 {
        inline Blob2() {}

        inline Blob2( const Blob2& B) {
          *this = B;
        }

        Blob2& operator = ( const Blob2& B ) {
          id    = B.id;     // unique ID for every blob in the  frame
          label = B.label;  // the body part label of this blob
          lid   = B.lid;    // the label id, namingly the number of blob of this label

          for(int i=0;i<3;++i)       mean(i) = B.mean(i);
          for(int i=0;i<9;++i)        cov(i) = B.cov(i);
          for(int i=0;i<3;++i)   eigenval(i) = B.eigenval(i);
          for(int i=0;i<9;++i)  eigenvect(i) = B.eigenvect(i);

          for(int i=0;i<MAX_CHILD;++i) child_id[i]      = B.child_id[i];
          for(int i=0;i<MAX_CHILD;++i) child_lid[i]     = B.child_lid[i];
          for(int i=0;i<MAX_CHILD;++i) child_dist[i]    = B.child_dist[i];
          for(int i=0;i<MAX_CHILD;++i) child_label[i]   = B.child_label[i];

          indices = B.indices;

          for(int i=0;i<3;++i)   min(i) = B.min(i);
          for(int i=0;i<3;++i)   max(i) = B.max(i);

          return *this;
        }

        int    id;                      // specific identification number of this blob
        part_t label;                   // labels which part this blob is, defined in common.
        int    lid;                     // label id, which number of this type of part is this

        Eigen::Vector4f  mean;          // mean in xyz
        Eigen::Matrix3f  cov;           // covariance in 3x3 matrix
        Eigen::Vector3f  eigenval;      // eigenvalue of blob
        Eigen::Matrix3f  eigenvect;     // eigenvector of blob

        //These variables are added in order to be able to build trees with them
        int    child_id[MAX_CHILD];     // id of the best found child
        int    child_lid[MAX_CHILD];    // lid of the best found child
        float  child_dist[MAX_CHILD];   // result of evaluation function of this child
        char   child_label[MAX_CHILD];  // makes displaying the tree easier
      
        pcl::PointIndices indices;      // The indices of the pointcloud
        Eigen::Vector4f   min;          // The min of the bounding box
        Eigen::Vector4f   max;          // The max of the bounding box
      };
      inline std::ostream& operator << (std::ostream& os, const Blob2& b)
      {
        os << " Blob2 id " << b.id << " label " << b.label << " lid " << b.lid << std::endl;
        os << " mean " << b.mean(0) << " , " << b.mean(1) << " , " << b.mean(2) << " , " << b.mean(3) << std::endl;
        os << " cov " << std::endl << b.cov << std::endl;
        os << " eigenval " << b.eigenval(0) << " , " << b.eigenval(1) << " , " << b.eigenval(2) << std::endl;
        os << " eigenvect " << std::endl << b.eigenvect << std::endl;
        os << " min " << b.min(0) << " , " << b.min(1) << " , " << b.min(2) << " , " << b.min(3) << std::endl;
        os << " max " << b.max(0) << " , " << b.max(1) << " , " << b.max(2) << " , " << b.max(3) << std::endl;
        os << " indices length " << b.indices.indices.size() << std::endl;
        for(int i = 0; i < MAX_CHILD; i++)
        {
          os << " child " << i << " id " << b.child_id[i] << " lid " << b.child_lid[i] << " dist " << b.child_dist[i] << " label " << b.child_label[i] << std::endl;
        }
        return (os);
      }
    } // end namespace label_skeleton
  } // end namespace people
} // end namespace pcl

#endif
