/**
 * @copyright Copyright (2011) Willow Garage
 * @author Koen Buys
 * @file common.h
 * @brief This file contains all LUT for the library and all common defines
 **/
#ifndef PCL_PEOPLE_LABELSKELETON_COMMON_H_
#define PCL_PEOPLE_LABELSKELETON_COMMON_H_

// Some defines for the tree
#define ROOT        -1
#define LEAF        -2
#define NO_CHILD    -3
#define NUM_PARTS   25      // We have 25 body parts defined
#define MAX_CHILD   4       // a parent node has maximum 4 children

// Some defines for geometry
#define FOCAL     525       // Focal length of rgb camera in pixels
#define WIDTH     640
#define HEIGHT    480
#define RATIO     WIDTH/HEIGHT

// Some defines for part image
#define NO_DATA   255       // We have no depth data for this part of the image

// Other defines
#define DEBUG     1

enum part_t{ Lfoot = 0,
             Lleg = 1,
             Lknee = 2,
             Lthigh = 3,
             Rfoot = 4,
             Rleg = 5,
             Rknee = 6,
             Rthigh = 7,
             Rhips = 8,
             Lhips = 9,
             Neck = 10,
             Rarm = 11,
             Relbow = 12,
             Rforearm = 13,
             Rhand = 14,
             Larm = 15,
             Lelbow = 16,
             Lforearm = 17,
             Lhand = 18,
             FaceLB = 19,
             FaceRB = 20,
             FaceLT = 21,
             FaceRT = 22,
             Rchest = 23,
             Lchest = 24};
/** @todo get this to work
std::string part_k[NUM_PARTS] = {"Lfoot","Lleg", "Lknee","Lthigh",
            "Rfoot","Rleg","Rknee","Rthigh",
            "Rhips","Lhips","Neck",
            "Rarm","Relbow","Rforearm","Rhand",
            "Larm","Lelbow","Lforearm","Lhand",
            "FaceLB","FaceRB","FaceLT","FaceRT",
            "Rchest","Lchest"};

inline std::ostream& operator << (std::ostream& os, const part_t& p)
{
  os << part_k[(int) p];
  return (os);
}
**/

namespace pcl
{
  namespace people
  {
    namespace label_skeleton
    {

    /**
     * @brief This LUT contains the max primary eigenvalue for each part
     * @todo read this from XML file
     **/
    static const float LUT_max_part_size[] = {
    0.5,            // 0 Lfoot
    0.7,            // 1 Lleg
    0.6,            // 2 Lknee
    0.6,            // 3 Lthigh
    0.5,            // 4 Rfoot
    0.7,            // 5 Rleg
    0.6,            // 6 Rknee
    0.6,            // 7 Rthigh
    0.9,            // 8 Rhips
    0.9,            // 9 Lhips
    0.5,            // 10 Neck
    0.7,            // 11 Rarm
    0.5,            // 12 Relbow
    0.7,            // 13 Rforearm
    0.5,            // 14 Rhand
    0.7,            // 15 Larm
    0.5,            // 16 Lelbow
    0.7,            // 17 Lforearm
    0.5,            // 18 Lhand
    0.5,            // 19 FaceLB
    0.5,            // 20 FaceRB
    0.5,            // 21 FaceLT
    0.5,            // 22 FaceRT
    0.9,            // 23 Rchest
    0.9             // 24 Lchest
    };

    /**
     *  @brief This LUT contains the ideal lenght between this part and his children
     **/
    static const float LUT_ideal_length[][4] = {
    {-1.0, -1.0, -1.0, -1.0}, // 0 Lfoot
    { 0.2, -1.0, -1.0, -1.0}, // 1 Lleg
    { 0.2, -1.0, -1.0, -1.0}, // 2 Lknee
    { 0.3, -1.0, -1.0, -1.0}, // 3 Lthigh
    {-1.0, -1.0, -1.0, -1.0}, // 4 Rfoot
    { 0.2, -1.0, -1.0, -1.0}, // 5 Rleg
    { 0.2, -1.0, -1.0, -1.0}, // 6 Rknee
    { 0.3, -1.0, -1.0, -1.0}, // 7 Rthigh
    { 0.3, -1.0, -1.0, -1.0}, // 8 Rhips
    { 0.3, -1.0, -1.0, -1.0}, // 9 Lhips
    { 0.15,  0.15,  0.2,  0.2}, // 10 Neck
    { 0.15, -1.0, -1.0, -1.0}, // 11 Rarm
    { 0.1, -1.0, -1.0, -1.0}, // 12 Relbow
    { 0.15, -1.0, -1.0, -1.0}, // 13 Rforearm
    {-1.0, -1.0, -1.0, -1.0}, // 14 Rhand
    { 0.15, -1.0, -1.0, -1.0}, // 15 Larm
    { 0.1, -1.0, -1.0, -1.0}, // 16 Lelbow
    { 0.15, -1.0, -1.0, -1.0}, // 17 Lforearm
    {-1.0, -1.0, -1.0, -1.0}, // 18 Lhand
    { 0.15, -1.0, -1.0, -1.0}, // 19 FaceLB
    { 0.15, -1.0, -1.0, -1.0}, // 20 FaceRB
    {-1.0, -1.0, -1.0, -1.0}, // 21 FaceLT
    {-1.0, -1.0, -1.0, -1.0}, // 22 FaceRT
    { 0.3,  0.3, -1.0, -1.0}, // 23 Rchest
    { 0.3,  0.3, -1.0, -1.0}  // 24 Lchest
    };

    /**
     * @brief This LUT contains the max lenght between this part and his children
     **/
    static const float LUT_max_length_offset[][4] = {
    { 0.15,  0.15,  0.15,  0.15}, // 0 Lfoot
    { 0.15,  0.15,  0.15,  0.15}, // 1 Lleg
    { 0.15,  0.15,  0.15,  0.15}, // 2 Lknee
    { 0.15,  0.15,  0.15,  0.15}, // 3 Lthigh
    { 0.15,  0.15,  0.15,  0.15}, // 4 Rfoot
    { 0.15,  0.15,  0.15,  0.15}, // 5 Rleg
    { 0.15,  0.15,  0.15,  0.15}, // 6 Rknee
    { 0.15,  0.15,  0.15,  0.15}, // 7 Rthigh
    { 0.15,  0.15,  0.15,  0.15}, // 8 Rhips
    { 0.15,  0.15,  0.15,  0.15}, // 9 Lhips
    { 0.15,  0.15,  0.15,  0.15}, // 10 Neck
    { 0.15,  0.15,  0.15,  0.15}, // 11 Rarm
    { 0.15,  0.15,  0.15,  0.15}, // 12 Relbow
    { 0.15,  0.15,  0.15,  0.15}, // 13 Rforearm
    { 0.15,  0.15,  0.15,  0.15}, // 14 Rhand
    { 0.15,  0.15,  0.15,  0.15}, // 15 Larm
    { 0.15,  0.15,  0.15,  0.15}, // 16 Lelbow
    { 0.15,  0.15,  0.15,  0.15}, // 17 Lforearm
    { 0.15,  0.15,  0.15,  0.15}, // 18 Lhand
    { 0.15,  0.15,  0.15,  0.15}, // 19 FaceLB
    { 0.15,  0.15,  0.15,  0.15}, // 20 FaceRB
    { 0.3,  0.15,  0.15,  0.15}, // 21 FaceLT
    { 0.3,  0.15,  0.15,  0.15}, // 22 FaceRT
    { 0.15,  0.15,  0.15,  0.15}, // 23 Rchest
    { 0.15,  0.15,  0.15,  0.15} // 24 Lchest
    };

    /**
     * @brief This LUT contains the number of children for each parent
     **/
    static const unsigned int LUT_nr_children[] = {
    0,            // 0 Lfoot
    1,            // 1 Lleg
    1,            // 2 Lknee
    1,            // 3 Lthigh
    0,            // 4 Rfoot
    1,            // 5 Rleg
    1,            // 6 Rknee
    1,            // 7 Rthigh
    1,            // 8 Rhips
    1,            // 9 Lhips
    4,            // 10 Neck
    1,            // 11 Rarm
    1,            // 12 Relbow
    1,            // 13 Rforearm
    0,            // 14 Rhand
    1,            // 15 Larm
    1,            // 16 Lelbow
    1,            // 17 Lforearm
    0,            // 18 Lhand
    1,            // 19 FaceLB
    1,            // 20 FaceRB
    0,            // 21 FaceLT
    0,            // 22 FaceRT
    2,            // 23 Rchest
    2             // 24 Lchest
    };

    } // End namespace LabelSkel
  } // End namespace people
} // End namespace pcl
#endif
