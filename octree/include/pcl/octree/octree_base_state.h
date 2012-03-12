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

#ifndef OCTREE_TREE_BASE_STATE_H
#define OCTREE_TREE_BASE_STATE_H

#include <pcl/octree/octree_base.h>

namespace pcl
{
  namespace octree
  {
    class OctreeBranchWithState : public OctreeBranch
    {
    public:
      OctreeBranchWithState () : OctreeBranch () {}
      /** \brief Copy constructor */
      OctreeBranchWithState (const OctreeBranchWithState& source)
        : OctreeBranch (source)
        , state_ (source.state_)
      { }
      inline void setState (const int& state) { state_ = state; }

      inline int getState () const { return (state_); }

    private:
      int state_;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree Branch with state
     *  \note Octree Branch that stores a state
     *  \note Used for occlusion estimation in octrees
     *  \ingroup octree
     *  \author Christian Potthast (potthast@usc.edu)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename DataT, typename LeafT = OctreeLeafDataT<DataT> >
    class OctreeBaseWithState : public OctreeBase <DataT, LeafT, OctreeBranchWithState>
    {
    public:

      typedef OctreeBase <DataT, LeafT, OctreeBranchWithState> Base;
      using Base::OctreeKey;
      typedef typename Base::OctreeBranch OctreeBranch;
      
      /** \brief Constructor */
      OctreeBaseWithState () : Base () {}
        
      /** \brief Empty deconstructor. */
      ~OctreeBaseWithState () {}
 
    protected:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /**\brief @b Octree leaf branch class.
       * \note It stores only the state of the branch.
       * \author Christian Potthast (potthast@usc.edu)
       */
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      class OctreeLeafBranch : public OctreeNode
      {
        // OctreeBase is a friend!
        friend class OctreeBaseWithState;

      public:

        /** \brief Constructor */
        OctreeLeafBranch () {}

        /** \brief Empty deconstructor. */
        virtual ~OctreeLeafBranch () {}

        /** \brief deep copy function */
        virtual OctreeNode*
        deepCopy () const
        {
          return (OctreeNode*) new OctreeLeafBranch (*this);
        }
        
        /** \brief Get the type of octree node. Returns LEAF_BRANCH type
         *  \return Returns LEAF_BRANCH type.
         * */
        virtual node_type_t
        getNodeType () const
        {
          return LEAF_BRANCH;
        }
        
      private:

        // branch state
        int state_;
      };

    protected:

      /** \brief Set the state of a Branch
       *  \param[in] Branch
       *  \param[in] state
       * */    
      inline void
      setBranchState (OctreeBranch*& branch_arg, const int state)
      {
        branch_arg->setState (state);
      }

      /** \brief Set the state of a Leaf Branch
       *  \param[in] Leaf Branch
       *  \param[in] state
       * */    
      inline void
      setBranchState (OctreeLeafBranch*& branch_arg, const int state)
      {
        branch_arg->state_ = state;
      }
    
      /** \brief Get the state of a Branch
       *  \param[in] Branch
       *  \return Branch state
       * */
      inline int
      getBranchState ( OctreeBranch*& branch_arg)
      {
        return (branch_arg->getState ());
      }
      /** \brief Get the state of a Leaf Branch
       *  \param[in] Leaf Branch
       *  \return Leaf Branch state
       * */   
      inline int
      getBranchState ( OctreeLeafBranch*& branch_arg)
      {
        return branch_arg->state_;
      }

    };
  }
}

#endif
