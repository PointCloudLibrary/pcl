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
            Hypothesis (ModelLibrary::Model* obj_model): obj_model_ (obj_model){}
            Hypothesis (const Hypothesis& src)
             : match_confidence_ (0.0f), obj_model_ (src.obj_model_)
            {
              for ( int i = 0 ; i < 12 ; ++i )
                this->rigid_transform_[i] = src.rigid_transform_[i];
            }
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
          * "voxel-surface" (especially for a sparsely sampled scene) and thus will make normal computation unreliable.
          *
          * \param[in] fraction_of_pairs_in_hash_table determines how many pairs (as fraction of the total number) will be kept in the hashtable. Use the default
          * value (check the papers above for more details). */
        ObjRecRANSAC (float pair_width, float voxel_size, float fraction_of_pairs_in_hash_table = 0.8f);
        virtual ~ObjRecRANSAC ()
        {
          this->clear();
        }

        /** \brief Deletes all models from the model library and releases the memory dynamically allocated by this instance. */
        void
        clear()
        {
          model_library_.clear();
          scene_octree_.clear ();
          scene_octree_proj_.clear ();
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
        bool
        addModel (PointCloudIn* points, PointCloudN* normals, const std::string& object_name)
        {
          return (model_library_.addModel (points, normals, object_name));
        }

        /** \brief This method performs the recognition of the models loaded to the model library with the method addModel().
          *
          * \param[in]  scene is the 3d scene in which the object should be recognized.
          * \param[in]  normals are the scene normals.
          * \param[out] recognized_objects is the list of output items each one containing the recognized model instance, its name, the aligning rigid transform
          * \param[in]  success_probability is the user-defined probability of detecting all objects in the scene.
          * and the match confidence (see ObjRecRANSAC::Output for further explanations).
          */
        void
        recognize (const PointCloudIn* scene, const PointCloudN* normals, std::list<ObjRecRANSAC::Output>& recognized_objects, double success_probability = 0.99);

        /** \brief Returns the hash table in the model library. */
        const pcl::recognition::ModelLibrary::HashTable*
        getHashTable()
        {
          return (model_library_.getHashTable ());
        }

      protected:
        int
        computeNumberOfIterations (double success_probability);

        void
        sampleOrientedPointPairs (int num_iterations, std::vector<ORROctree::Node*>& full_scene_leaves, std::list<OrientedPointPair>& output);

        int
        generateHypotheses(const std::list<OrientedPointPair>& pairs, std::list<Hypothesis>& out);

        void
        testHypotheses (std::list<Hypothesis>& hypotheses, std::vector<Hypothesis>& accepted_hypotheses);

        void
        buildConflictGraph (std::vector<Hypothesis>& hypotheses, ORRGraph& graph);

        void
        filterWeakHypotheses (ORRGraph& graph, std::list<ObjRecRANSAC::Output>& recognized_objects);

    	/** \brief Computes the rigid transform in that maps the line (a1, b1) to (a2, b2).
    	 * The computation is based on the corresponding points 'a1' <-> 'a2' and 'b1' <-> 'b2'
    	 * and	the normals 'a1_n', 'b1_n', 'a2_n', and 'b2_n'. The result is saved in
    	 * 'rigid_transform' which is an array of length 12. The first 9 elements are the
    	 * rotational part (row major order) and the last 3 are the translation. */
        void
        computeRigidTransform(
          const float *a1, const float *a1_n, const float *b1, const float* b1_n,
          const float *a2, const float *a2_n, const float *b2, const float* b2_n,
          float* rigid_transform) const;

        /** \brief Computes the signature of the oriented point pair ((p1, n1), (p2, n2)) consisting of the angles between
          * n1 and (p2-p1),
          * n2 and (p1-p2),
          * n1 and n2
          *
          * \param[out] signature is an array of three doubles saving the three angles in the order shown above. */
        static inline void
        compute_oriented_point_pair_signature (const float *p1, const float *n1, const float *p2, const float *n2, float signature[3]);

        friend class ModelLibrary;

      protected:
        float pair_width_, voxel_size_, fraction_of_pairs_in_hash_table_, relative_obj_size_;
        float abs_zdist_thresh_;
    	float visibility_, relative_num_of_illegal_pts_, intersection_fraction_;

        ModelLibrary model_library_;
        ORROctree scene_octree_;
        ORROctreeZProjection scene_octree_proj_;
    };

// === inline methods ===================================================================================================================================

    inline void
    ObjRecRANSAC::compute_oriented_point_pair_signature (const float *p1, const float *n1, const float *p2, const float *n2, float signature[3])
    {
      // Get the line from p1 to p2
      float line[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
      aux::vecNormalize3 (line);

      signature[0] = static_cast<float> (acos (aux::vecDot3 (n1,line))); line[0] = -line[0]; line[1] = -line[1]; line[2] = -line[2];
      signature[1] = static_cast<float> (acos (aux::vecDot3 (n2,line)));
      signature[2] = static_cast<float> (acos (aux::vecDot3 (n1,n2)));
    }
  } // namespace recognition
} // namespace pcl

#endif // PCL_RECOGNITION_OBJ_REC_RANSAC_H_
