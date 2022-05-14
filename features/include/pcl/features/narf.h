/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

namespace pcl
{
  // Forward declarations
  class RangeImage;
  struct InterestPoint;

#define NARF_DEFAULT_SURFACE_PATCH_PIXEL_SIZE 10

  /**
    * \brief NARF (Normal Aligned Radial Features) is a point feature descriptor type for 3D data.
    * Please refer to pcl/features/narf_descriptor.h if you want the class derived from pcl Feature.
    * See B. Steder, R. B. Rusu, K. Konolige, and W. Burgard
    *     Point Feature Extraction on 3D Range Scans Taking into Account Object Boundaries
    *     In Proc. of the IEEE Int. Conf. on Robotics &Automation (ICRA). 2011. 
    * \author Bastian Steder
    * \ingroup features
    */
  class PCL_EXPORTS Narf
  {
    public:
      // =====CONSTRUCTOR & DESTRUCTOR=====
      //! Constructor
      Narf();
      //! Copy Constructor
      Narf(const Narf& other);
      //! Destructor
      ~Narf();
      
      // =====Operators=====
      //! Assignment operator
      const Narf& operator=(const Narf& other);

      // =====STATIC=====
      /** The maximum number of openmp threads that can be used in this class */
      static int max_no_of_threads;

      /** Add features extracted at the given interest point and add them to the list */
      static void 
      extractFromRangeImageAndAddToList (const RangeImage& range_image, const Eigen::Vector3f& interest_point, int descriptor_size,
                                         float support_size, bool rotation_invariant, std::vector<Narf*>& feature_list);
      /** Same as above */
      static void 
      extractFromRangeImageAndAddToList (const RangeImage& range_image, float image_x, float image_y, int descriptor_size,
                                         float support_size, bool rotation_invariant, std::vector<Narf*>& feature_list);
      /** Get a list of features from the given interest points. */
      static void 
      extractForInterestPoints (const RangeImage& range_image, const PointCloud<InterestPoint>& interest_points,
                                int descriptor_size, float support_size, bool rotation_invariant, std::vector<Narf*>& feature_list);
      /** Extract an NARF for every point in the range image. */
      static void 
      extractForEveryRangeImagePointAndAddToList (const RangeImage& range_image, int descriptor_size, float support_size,
                                                  bool rotation_invariant, std::vector<Narf*>& feature_list);
      
      // =====PUBLIC METHODS=====
      /** Method to extract a NARF feature from a certain 3D point using a range image.
       *  pose determines the coordinate system of the feature, whereas it transforms a point from the world into the feature system.
       *  This means the interest point at which the feature is extracted will be the inverse application of pose onto (0,0,0).
       *  descriptor_size_ determines the size of the descriptor,
       *  support_size determines the support size of the feature, meaning the size in the world it covers */
      bool 
      extractFromRangeImage (const RangeImage& range_image, const Eigen::Affine3f& pose, int descriptor_size, float support_size,
                             int surface_patch_world_size=NARF_DEFAULT_SURFACE_PATCH_PIXEL_SIZE);
      
      //! Same as above, but determines the transformation from the surface in the range image
      bool 
      extractFromRangeImage (const RangeImage& range_image, float x, float y, int descriptor_size, float support_size);

      //! Same as above
      bool 
      extractFromRangeImage (const RangeImage& range_image, const InterestPoint& interest_point, int descriptor_size, float support_size);

      //! Same as above
      bool 
      extractFromRangeImage (const RangeImage& range_image, const Eigen::Vector3f& interest_point, int descriptor_size, float support_size);

      /** Same as above, but using the rotational invariant version by choosing the best extracted rotation around the normal.
       *  Use extractFromRangeImageAndAddToList if you want to enable the system to return multiple features with different rotations. */
      bool 
      extractFromRangeImageWithBestRotation (const RangeImage& range_image, const Eigen::Vector3f& interest_point,
                                             int descriptor_size, float support_size);
      
      /* Get the dominant rotations of the current descriptor
       * \param rotations the returned rotations
       * \param strength values describing how pronounced the corresponding rotations are
       */
      void 
      getRotations (std::vector<float>& rotations, std::vector<float>& strengths) const;
      
      /* Get the feature with a different rotation around the normal
       * You are responsible for deleting the new features!
       * \param range_image the source from which the feature is extracted
       * \param rotations list of angles (in radians)
       * \param rvps returned features
       */
      void 
      getRotatedVersions (const RangeImage& range_image, const std::vector<float>& rotations, std::vector<Narf*>& features) const;
      
      //! Calculate descriptor distance, value in [0,1] with 0 meaning identical and 1 every cell above maximum distance
      inline float 
      getDescriptorDistance (const Narf& other) const;
      
      //! How many points on each beam of the gradient star are used to calculate the descriptor?
      inline int 
      getNoOfBeamPoints () const { return (static_cast<int> (pcl_lrint (std::ceil (0.5f * float (surface_patch_pixel_size_))))); }
      
      //! Copy the descriptor and pose to the point struct Narf36
      inline void 
      copyToNarf36 (Narf36& narf36) const;
      
      /** Write to file */
      void 
      saveBinary (const std::string& filename) const;
      /** Write to output stream */
      void 
      saveBinary (std::ostream& file) const;
      
      /** Read from file */
      void 
      loadBinary (const std::string& filename);
      /** Read from input stream */
      void 
      loadBinary (std::istream& file);
      
      //! Create the descriptor from the already set other members
      bool 
      extractDescriptor (int descriptor_size);
      
      // =====GETTERS=====
      //! Getter (const) for the descriptor
      inline const float* 
      getDescriptor () const { return descriptor_;}
      //! Getter for the descriptor
      inline float* 
      getDescriptor () { return descriptor_;}
      //! Getter (const) for the descriptor length
      inline const int& 
      getDescriptorSize () const { return descriptor_size_;}
      //! Getter for the descriptor length
      inline int& 
      getDescriptorSize () { return descriptor_size_;}
      //! Getter (const) for the position
      inline const Eigen::Vector3f& 
      getPosition () const { return position_;}
      //! Getter for the position
      inline Eigen::Vector3f& 
      getPosition () { return position_;}
      //! Getter (const) for the 6DoF pose
      inline const Eigen::Affine3f& 
      getTransformation () const { return transformation_;}
      //! Getter for the 6DoF pose
      inline Eigen::Affine3f& 
      getTransformation () { return transformation_;}
      //! Getter (const) for the pixel size of the surface patch (only one dimension)
      inline const int& 
      getSurfacePatchPixelSize () const { return surface_patch_pixel_size_;}
      //! Getter for the pixel size of the surface patch (only one dimension)
      inline int& 
      getSurfacePatchPixelSize () { return surface_patch_pixel_size_;}
      //! Getter (const) for the world size of the surface patch
      inline const float& 
      getSurfacePatchWorldSize () const { return surface_patch_world_size_;}
      //! Getter for the world size of the surface patch
      inline float& 
      getSurfacePatchWorldSize () { return surface_patch_world_size_;}
      //! Getter (const) for the rotation of the surface patch
      inline const float& 
      getSurfacePatchRotation () const { return surface_patch_rotation_;}
      //! Getter for the rotation of the surface patch
      inline float& 
      getSurfacePatchRotation () { return surface_patch_rotation_;}
      //! Getter (const) for the surface patch
      inline const float* 
      getSurfacePatch () const { return surface_patch_;}
      //! Getter for the surface patch
      inline float* 
      getSurfacePatch () { return surface_patch_;}
      //! Method to erase the surface patch and free the memory
      inline void 
      freeSurfacePatch () { delete[] surface_patch_; surface_patch_=nullptr; surface_patch_pixel_size_=0; }
      
      // =====SETTERS=====
      //! Setter for the descriptor
      inline void 
      setDescriptor (float* descriptor) { descriptor_ = descriptor;}
      //! Setter for the surface patch
      inline void 
      setSurfacePatch (float* surface_patch) { surface_patch_ = surface_patch;}
      
      // =====PUBLIC MEMBER VARIABLES=====
      
      // =====PUBLIC STRUCTS=====
      struct FeaturePointRepresentation : public PointRepresentation<Narf*>
      {
        using PointT = Narf *;
        FeaturePointRepresentation(int nr_dimensions) { this->nr_dimensions_ = nr_dimensions; }
        /** \brief Empty destructor */
        ~FeaturePointRepresentation () {}
        void copyToFloatArray (const PointT& p, float* out) const override { memcpy(out, p->getDescriptor(), sizeof(*p->getDescriptor())*this->nr_dimensions_); }
      };
      
    protected:
      // =====PROTECTED METHODS=====
      //! Reset al members to default values and free allocated memory
      void 
      reset ();
      //! Create a deep copy of other
      void 
      deepCopy (const Narf& other);
      //! Get the surface patch with a blur on it
      float* 
      getBlurredSurfacePatch (int new_pixel_size, int blur_radius) const;
      
      /** Write header to output stream */
      void 
      saveHeader (std::ostream& file) const;
      /** Read header from input stream */
      int 
      loadHeader (std::istream& file) const;
      
      // =====PROTECTED STATIC METHODS=====
      static const std::string 
      getHeaderKeyword () { return "NARF"; }
      
      // =====PROTECTED STATIC VARIABLES=====
      const static int VERSION = 1;

      // =====PROTECTED MEMBER VARIABLES=====
      Eigen::Vector3f position_;
      Eigen::Affine3f transformation_;
      float* surface_patch_;
      int surface_patch_pixel_size_;
      float surface_patch_world_size_;
      float surface_patch_rotation_;
      float* descriptor_;
      int descriptor_size_;

      // =====STATIC PROTECTED=====
      
    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
#undef NARF_DEFAULT_SURFACE_PATCH_PIXEL_SIZE

}  // end namespace pcl

#include <pcl/features/impl/narf.hpp>
