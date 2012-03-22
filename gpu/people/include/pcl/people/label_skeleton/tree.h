/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file tree.h
 * @brief This file contains the function prototypes for the tree building functions.
 */
// our headers
#include "pcl/people/label_skeleton/blob2.h"   //this one defines the blob structure
#include "pcl/people/label_skeleton/tree2.h"   //this one defines the blob structure
#include "pcl/people/label_skeleton/common.h"  //this one defines the LUT's

namespace pcl
{
  namespace people
  {
    namespace label_skeleton
    {
      int buildRelations( std::vector<std::vector<Blob2> >& sorted);

      int leafBlobVector(   std::vector<std::vector<Blob2> >& sorted,
                            int                               label );

      int noChildBlobVector(  std::vector<std::vector<Blob2> >& sorted,
                              int                               label,
                              int                               child_number);

      bool hasThisLabelChildren ( std::vector<std::vector<Blob2> >& sorted,
                                  part_t                            label,
                                  int                               child_number);

      int evaluateBlobVector( std::vector<std::vector<Blob2> >& sorted,
                              unsigned int                      parent_label,
                              int                               child_label,
                              int                               child_number);

      float evaluateBlobs (Blob2& parent, Blob2& child, int child_nr);

      int buildTree ( std::vector<std::vector <Blob2> >&  sorted,
                      pcl::PointCloud<pcl::PointXYZRGB>&  cloud_in,
                      part_t                              part_label,
                      int                                 part_lid,
                      Tree2&                              tree);
    } //end namespace LabelSkel
  } //end namespace people
} // end namespace pcl
