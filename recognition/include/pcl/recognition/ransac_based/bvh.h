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

/*
 * bvh.h
 *
 *  Created on: Mar 7, 2013
 *      Author: papazov
 */

#ifndef PCL_RECOGNITION_BVH_H_
#define PCL_RECOGNITION_BVH_H_

#include <pcl/pcl_exports.h>
#include <cstring>
#include <algorithm>
#include <vector>
#include <list>

namespace pcl
{
  namespace recognition
  {
    /** \brief This class is an implementation of bounding volume hierarchies. Use the build method to construct
      * the data structure. To use the class, construct an std::vector of pointers to BVH::BoundedObject objects
      * and pass it to the build method. BVH::BoundedObject is a template class, so you can save user-defined data
      * in it.
      *
      * The tree is built such that each leaf contains exactly one object. */
    template<class UserData>
    class PCL_EXPORTS BVH
    {
      public:
        class BoundedObject
        {
          public:
            BoundedObject (const UserData& data)
            : data_ (data)
            {
            }

            virtual ~BoundedObject ()
            {
            }

            /** \brief This method is for std::sort. */
            inline static bool
            compareCentroidsXCoordinates (const BoundedObject* a, const BoundedObject* b)
            {
              return static_cast<bool> (a->getCentroid ()[0] < b->getCentroid ()[0]);
            }

            float*
            getBounds ()
            {
              return (bounds_);
            }

            float*
            getCentroid ()
            {
              return (centroid_);
            }

            const float*
            getCentroid () const
            {
              return (centroid_);
            }

            UserData&
            getData ()
            {
              return (data_);
            }

          protected:
            /** These are the bounds of the object.*/
            float bounds_[6];
            /** This is the centroid. */
            float centroid_[3];
            /** This is the user-defined data object. */
            UserData data_;
        };

      protected:
        class Node
        {
          public:
            /** \brief 'sorted_objects' is a sorted vector of bounded objects. It has to be sorted in ascending order according
              * to the objects' x-coordinates. The constructor recursively calls itself with the right 'first_id' and 'last_id'
              * and with the same vector 'sorted_objects'.  */
            Node (std::vector<BoundedObject*>& sorted_objects, int first_id, int last_id)
            {
              // Initialize the bounds of the node
              memcpy (bounds_, sorted_objects[first_id]->getBounds (), 6*sizeof (float));

              // Expand the bounds of the node
              for ( int i = first_id + 1 ; i <= last_id ; ++i )
                aux::expandBoundingBox(bounds_, sorted_objects[i]->getBounds());

              // Shall we create children?
              if ( first_id != last_id )
              {
                // Division by 2
                int mid_id = (first_id + last_id) >> 1;
                children_[0] = new Node(sorted_objects, first_id, mid_id);
                children_[1] = new Node(sorted_objects, mid_id + 1, last_id);
              }
              else
              {
                // We reached a leaf
                object_ = sorted_objects[first_id];
                children_[0] = children_[1] = 0;
              }
            }

            virtual ~Node ()
            {
              if ( children_[0] )
              {
                delete children_[0];
                delete children_[1];
              }
            }

            bool
            hasChildren () const
            {
              return static_cast<bool>(children_[0]);
            }

            Node*
            getLeftChild ()
            {
              return children_[0];
            }

            Node*
            getRightChild ()
            {
              return children_[1];
            }

            BoundedObject*
            getObject ()
            {
              return object_;
            }

            bool
            isLeaf () const
            {
              return !static_cast<bool>(children_[0]);
            }

            /** \brief Returns true if 'box' intersects or touches (with a side or a vertex) this node. */
            inline bool
            intersect(const float box[6]) const
            {
              if ( box[1] < bounds_[0] || box[3] < bounds_[2] || box[5] < bounds_[4] ||
                   box[0] > bounds_[1] || box[2] > bounds_[3] || box[4] > bounds_[5] )
                return false;

              return true;
            }

            /** \brief Computes and returns the volume of the bounding box of this node. */
            double
            computeBoundingBoxVolume() const
            {
              return (bounds_[1] - bounds_[0]) * (bounds_[3] - bounds_[2]) * (bounds_[5] - bounds_[4]);
            }

            friend class BVH;

          protected:
            float bounds_[6];
            Node* children_[2];
            BoundedObject* object_;
        };

      public:
        BVH()
        : root_ (0),
          sorted_objects_ (0)
        {
        }

        virtual ~BVH()
        {
          this->clear ();
        }

        /** \brief Creates the tree. No need to call clear, it's called within the method. 'objects' is a vector of
          * pointers to bounded objects which have to have valid bounds and centroids. Use the getData method of
          * BoundedObject to retrieve the user-defined data saved in the object. Note that vector will be sorted within
          * the method!
          *
          * The tree is built such that each leaf contains exactly one object. */
        void
        build(std::vector<BoundedObject*>& objects)
        {
          this->clear();

          if ( objects.size () == 0 )
            return;

          sorted_objects_ = &objects;

          // Now sort the objects according to the x-coordinates of their centroids
          std::sort (objects.begin (), objects.end (), BoundedObject::compareCentroidsXCoordinates);

          // Create the root -> it recursively creates the children nodes until each leaf contains exactly one object
          root_ = new Node (objects, 0, static_cast<int> (objects.size () - 1));
        }

        /** \brief Frees the memory allocated by this object. After that, you have to call build to use the tree again. */
        void
        clear()
        {
          if ( root_ )
          {
            delete root_;
            root_ = 0;
          }
        }

        inline const std::vector<BoundedObject*>*
        getInputObjects () const
        {
          return (sorted_objects_);
        }

        /** \brief Pushes back in 'intersected_objects' the bounded objects intersected by the input 'box' and returns true.
          * Returns false if no objects are intersected. */
        inline bool
        intersect(const float box[6], std::list<BoundedObject*>& intersected_objects) const
        {
          if ( !root_ )
            return false;

          bool got_intersection = false;

          // Start the intersection process at the root
          std::list<Node*> working_list;
          working_list.push_back (root_);

          while ( !working_list.empty () )
          {
            Node* node = working_list.front ();
            working_list.pop_front ();

            // Is 'node' intersected by the box?
            if ( node->intersect (box) )
            {
              // We have to check the children of the intersected 'node'
              if ( node->hasChildren () )
              {
                working_list.push_back (node->getLeftChild ());
                working_list.push_back (node->getRightChild ());
              }
              else // 'node' is a leaf -> save it's object in the output list
              {
                intersected_objects.push_back (node->getObject ());
                got_intersection = true;
              }
            }
          }

          return (got_intersection);
        }

      protected:
        Node* root_;
        std::vector<BoundedObject*>* sorted_objects_;
    };
  } // namespace recognition
} // namespace pcl

#endif /* PCL_RECOGNITION_BVH_H_ */
