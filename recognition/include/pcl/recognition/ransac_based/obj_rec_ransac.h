/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_RECOGNITION_OBJ_REC_RANSAC_H_
#define PCL_RECOGNITION_OBJ_REC_RANSAC_H_

#include <pcl/recognition/ransac_based/model_library.h>
#include <pcl/recognition/ransac_based/orr_octree.h>
#include <pcl/recognition/ransac_based/orr_octree_zprojection.h>
#include <pcl/recognition/ransac_based/auxiliary.h>
#include <pcl/pcl_exports.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <string>
#include <vector>
#include <list>

#define OBJ_REC_RANSAC_VERBOSE

namespace pcl
{
  namespace recognition
  {
    class ORRGraph;

    /** \brief This is a RANSAC-based 3D object recognition method. Do the following to use it: (i) call addModel() k times with k different models
      * representing the objects to be recognized and (ii) call recognize() with the 3D scene in which the objects should be recognized. Recognition means both
      * object identification and pose (position + orientation) estimation. Check the method descriptions for more details.
      *
      * \note If you use this code in any academic work, please cite:
      *
      *   - Chavdar Papazov, Sami Haddadin, Sven Parusel, Kai Krieger and Darius Burschka.
      *     Rigid 3D geometry matching for grasping of known objects in cluttered scenes.
      *     The International Journal of Robotics Research 2012. DOI: 10.1177/0278364911436019
      *
      *   - Chavdar Papazov and Darius Burschka.
      *     An Efficient RANSAC for 3D Object Recognition in Noisy and Occluded Scenes.
      *     In Proceedings of the 10th Asian Conference on Computer Vision (ACCV'10),
      *     November 2010.
      *
      *
      * \author Chavdar Papazov
      * \ingroup recognition
      */
    class PCL_EXPORTS ObjRecRANSAC
    {
      public:
        typedef ModelLibrary::PointCloudIn PointCloudIn;
        typedef ModelLibrary::PointCloudN PointCloudN;

        /** \brief This is an output item of the ObjRecRANSAC::recognize() method. It contains the recognized model, its name (the ones passed to
          * ObjRecRANSAC::addModel()), the rigid transform which aligns the model with the input scene and the match confidence which is a number
          * in the interval (0, 1] which gives the fraction of the model surface area matched to the scene. E.g., a match confidence of 0.3 means
          * that 30% of the object surface area was matched to the scene points. If the scene is represented by a single range image, the match
          * confidence can not be greater than 0.5 since the range scanner sees only one side of each object.
          */
        class Output
        {
          public:
            Output (const std::string& object_name, const float rigid_transform[12], float match_confidence) :
              object_name_ (object_name),
              match_confidence_ (match_confidence)
            {
              for ( int i = 0 ; i < 12 ; ++i )
                rigid_transform_[i] = rigid_transform[i];
            }
            virtual ~Output (){}

          public:
            std::string object_name_;
            float rigid_transform_[12], match_confidence_;
        };

    	class OrientedPointPair
    	{
    	  public:
            OrientedPointPair (const float *p1, const float *n1, const float *p2, const float *n2)
            {
              memcpy(p1_, p1, 3*sizeof (float));
              memcpy(n1_, n1, 3*sizeof (float));
              memcpy(p2_, p2, 3*sizeof (float));
              memcpy(n2_, n2, 3*sizeof (float));
            }
            virtual ~OrientedPointPair (){}
            float p1_[3], n1_[3], p2_[3], n2_[3];
    	};

    	class Hypothesis
    	{
          public:
            Hypothesis (ModelLibrary::Model* obj_model)
             : match_confidence_ (0.0f),
               obj_model_ (obj_model)
            {}
            virtual ~Hypothesis (){}

          public:
            float rigid_transform_[12], match_confidence_;
            ModelLibrary::Model* obj_model_;
            std::set<int> explained_pixels_;
    	};

      public:
        /** \brief Constructor with some important parameters which can not be changed once an instance of that class is created.
          *
          * \param[in] pair_width should be roughly half the extent of the visible object part. This means, for each object point p there should be (at least)
          * one point q (from the same object) such that ||p - q|| <= pair_width. Tradeoff: smaller values allow for detection in more occluded scenes but lead
          * to more imprecise alignment. Bigger values lead to better alignment but require large visible object parts (i.e., less occlusion).
          *
          * \param[in] voxel_size is the size of the leafs of the octree, i.e., the "size" of the discretization. Tradeoff: High values lead to less
          * computation time but ignore object details. Small values allow to better distinguish between objects, but will introduce more holes in the resulting
          * "voxel-surface" (especially for a sparsely sampled scene).
          *
          * \param[in] fraction_of_pairs_in_hash_table determines how many pairs (as fraction of the total number) will be kept in the hashtable. Use the default
          * value (check the papers above for more details). */
        ObjRecRANSAC (float pair_width, float voxel_size, float fraction_of_pairs_in_hash_table = 0.8f);
        virtual ~ObjRecRANSAC ()
        {
          this->clear();
        }

        /** \brief Removes all models from the model library and releases some memory dynamically allocated by this instance. */
        void
        inline clear()
        {
          model_library_.removeAllModels ();
          scene_octree_.clear ();
          scene_octree_proj_.clear ();
          sampled_oriented_point_pairs_.clear ();
        }

        /** \brief Add an object model to be recognized.
          *
          * \param[in] points are the object points.
          * \param[in] normals at each point.
          * \param[in] object_name is an identifier for the object. If that object is detected in the scene 'object_name' is returned by the recognition
          * method and you know which object has been detected. Note that 'object_name' has to be unique!
          *
          * The method returns true if the model was successfully added to the model library and false otherwise (e.g., if 'object_name' is already in use).
          */
        inline bool
        addModel (PointCloudIn* points, PointCloudN* normals, const std::string& object_name)
        {
          return (model_library_.addModel (points, normals, object_name));
        }

        /** \brief This method performs the recognition of the models loaded to the model library with the method addModel().
          *
          * \param[in]  scene is the 3d scene in which the object should be recognized.
          * \param[in]  normals are the scene normals.
          * \param[out] recognized_objects is the list of output items each one containing the recognized model instance, its name, the aligning rigid transform
          * and the match confidence (see ObjRecRANSAC::Output for further explanations).
          * \param[in]  success_probability is the user-defined probability of detecting all objects in the scene.
          */
        void
        recognize (const PointCloudIn* scene, const PointCloudN* normals, std::list<ObjRecRANSAC::Output>& recognized_objects, double success_probability = 0.99);

        inline void
        enterTestModeSampleOPP ()
        {
          rec_mode_ = ObjRecRANSAC::SAMPLE_OPP;
        }

        inline void
        leaveTestMode ()
        {
          rec_mode_ = ObjRecRANSAC::FULL_RECOGNITION;
        }

        /** \brief Returns the hash table in the model library. */
        inline const pcl::recognition::ModelLibrary::HashTable*
        getHashTable ()
        {
          return (model_library_.getHashTable ());
        }

        inline const ORROctree&
        getSceneOctree ()
        {
          return (scene_octree_);
        }

        /** \brief This function is useful for testing purposes. It returns the otiented point pairs which were sampled from the
          * scene during the recognition process. */
        inline const std::list<ObjRecRANSAC::OrientedPointPair>&
        getSampledOrientedPointPairs () const
        {
          return (sampled_oriented_point_pairs_);
        }

        inline float
        getPairWidth () const
        {
          return pair_width_;
        }

      protected:
        enum Recognition_Mode {SAMPLE_OPP = 0, /*GENERATE_HYPOTHESES, TEST_HYPOTHESES, BUILD_CONFLICT_GRAPH,*/ FULL_RECOGNITION};

        friend class ModelLibrary;

        inline int
        computeNumberOfIterations (double success_probability)
        {
          // 'p_obj' is the probability that given that the first sample point belongs to an object,
          // the second sample point will belong to the same object
          double p, p_obj = 0.25f;

          p = p_obj*relative_obj_size_*fraction_of_pairs_in_hash_table_;

          if ( 1.0 - p <= 0.0 )
            return 1;

          return static_cast<int> (log (1.0-success_probability)/log (1.0-p) + 1.0);
        }

        void
        sampleOrientedPointPairs (int num_iterations, std::vector<ORROctree::Node*>& full_scene_leaves, std::list<OrientedPointPair>& output);

        int
        generateHypotheses(const std::list<OrientedPointPair>& pairs, std::list<Hypothesis*>& out);

        void
        testHypotheses (std::list<Hypothesis*>& hypotheses, std::vector<Hypothesis*>& accepted_hypotheses);

        void
        buildConflictGraph (std::vector<Hypothesis*>& hypotheses, ORRGraph& graph);

        void
        filterWeakHypotheses (ORRGraph& graph, std::list<ObjRecRANSAC::Output>& recognized_objects);

    	/** \brief Computes the rigid transform that maps the line (a1, b1) to (a2, b2).
    	 * The computation is based on the corresponding points 'a1' <-> 'a2' and 'b1' <-> 'b2'
    	 * and the normals 'a1_n', 'b1_n', 'a2_n', and 'b2_n'. The result is saved in
    	 * 'rigid_transform' which is an array of length 12. The first 9 elements are the
    	 * rotational part (row major order) and the last 3 are the translation. */
        inline void
        computeRigidTransform(
          const float *a1, const float *a1_n, const float *b1, const float* b1_n,
          const float *a2, const float *a2_n, const float *b2, const float* b2_n,
          float* rigid_transform) const
        {
          // Some local variables
          float o1[3], o2[3], x1[3], x2[3], y1[3], y2[3], z1[3], z2[3], tmp1[3], tmp2[3], Ro1[3], invFrame1[3][3];

          // Compute the origins
          o1[0] = 0.5f*(a1[0] + b1[0]);
          o1[1] = 0.5f*(a1[1] + b1[1]);
          o1[2] = 0.5f*(a1[2] + b1[2]);

          o2[0] = 0.5f*(a2[0] + b2[0]);
          o2[1] = 0.5f*(a2[1] + b2[1]);
          o2[2] = 0.5f*(a2[2] + b2[2]);

          // Compute the x-axes
          aux::vecDiff3 (b1, a1, x1); aux::vecNormalize3 (x1);
          aux::vecDiff3 (b2, a2, x2); aux::vecNormalize3 (x2);
          // Compute the y-axes. First y-axis
          aux::projectOnPlane3 (a1_n, x1, tmp1); aux::vecNormalize3 (tmp1);
          aux::projectOnPlane3 (b1_n, x1, tmp2); aux::vecNormalize3 (tmp2);
          aux::vecSum3 (tmp1, tmp2, y1); aux::vecNormalize3 (y1);
          // Second y-axis
          aux::projectOnPlane3 (a2_n, x2, tmp1); aux::vecNormalize3 (tmp1);
          aux::projectOnPlane3 (b2_n, x2, tmp2); aux::vecNormalize3 (tmp2);
          aux::vecSum3 (tmp1, tmp2, y2); aux::vecNormalize3 (y2);
          // Compute the z-axes
          aux::vecCross3 (x1, y1, z1);
          aux::vecCross3 (x2, y2, z2);

          // 1. Invert the matrix [x1|y1|z1] (note that x1, y1, and z1 are treated as columns!)
          invFrame1[0][0] = x1[0]; invFrame1[0][1] = x1[1]; invFrame1[0][2] = x1[2];
          invFrame1[1][0] = y1[0]; invFrame1[1][1] = y1[1]; invFrame1[1][2] = y1[2];
          invFrame1[2][0] = z1[0]; invFrame1[2][1] = z1[1]; invFrame1[2][2] = z1[2];
          // 2. Compute the desired rotation as rigid_transform = [x2|y2|z2]*invFrame1
          aux::mult3x3 (x2, y2, z2, invFrame1, rigid_transform);

          // Construct the translation which is the difference between the rotated o1 and o2
          aux::mult3x3 (rigid_transform, o1, Ro1);
          rigid_transform[9]  = o2[0] - Ro1[0];
          rigid_transform[10] = o2[1] - Ro1[1];
          rigid_transform[11] = o2[2] - Ro1[2];
        }

        /** \brief Computes the signature of the oriented point pair ((p1, n1), (p2, n2)) consisting of the angles between
          * n1 and (p2-p1),
          * n2 and (p1-p2),
          * n1 and n2
          *
          * \param[out] signature is an array of three doubles saving the three angles in the order shown above. */
        static inline void
        compute_oriented_point_pair_signature (const float *p1, const float *n1, const float *p2, const float *n2, float signature[3])
        {
          // Get the line from p1 to p2
          float line[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
          aux::vecNormalize3 (line);

          signature[0] = static_cast<float> (acos (aux::vecDot3 (n1,line))); line[0] = -line[0]; line[1] = -line[1]; line[2] = -line[2];
          signature[1] = static_cast<float> (acos (aux::vecDot3 (n2,line)));
          signature[2] = static_cast<float> (acos (aux::vecDot3 (n1,n2)));
        }

      protected:
        float pair_width_, voxel_size_, fraction_of_pairs_in_hash_table_, relative_obj_size_;
        float abs_zdist_thresh_;
    	float visibility_, relative_num_of_illegal_pts_, intersection_fraction_;

        ModelLibrary model_library_;
        ORROctree scene_octree_;
        ORROctreeZProjection scene_octree_proj_;

        std::list<OrientedPointPair> sampled_oriented_point_pairs_;
        Recognition_Mode rec_mode_;
    };
  } // namespace recognition
} // namespace pcl

#endif // PCL_RECOGNITION_OBJ_REC_RANSAC_H_
