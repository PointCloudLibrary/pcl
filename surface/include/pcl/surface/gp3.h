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
 * $Id$
 *
 */

#pragma once

// PCL includes
#include <pcl/surface/reconstruction.h>

#include <pcl/kdtree/kdtree.h>

#include <fstream>

#include <Eigen/Geometry> // for cross

namespace pcl
{
  struct PolygonMesh;

  /** \brief Returns if a point X is visible from point R (or the origin)
    * when taking into account the segment between the points S1 and S2
    * \param X 2D coordinate of the point
    * \param S1 2D coordinate of the segment's first point
    * \param S2 2D coordinate of the segment's second point
    * \param R 2D coordinate of the reference point (defaults to 0,0)
    * \ingroup surface
    */
  inline bool 
  isVisible (const Eigen::Vector2f &X, const Eigen::Vector2f &S1, const Eigen::Vector2f &S2, 
             const Eigen::Vector2f &R = Eigen::Vector2f::Zero ())
  {
    double a0 = S1[1] - S2[1];
    double b0 = S2[0] - S1[0];
    double c0 = S1[0]*S2[1] - S2[0]*S1[1];
    double a1 = -X[1];
    double b1 = X[0];
    double c1 = 0;
    if (R != Eigen::Vector2f::Zero())
    {
      a1 += R[1];
      b1 -= R[0];
      c1 = R[0]*X[1] - X[0]*R[1];
    }
    double div = a0*b1 - b0*a1;
    double x = (b0*c1 - b1*c0) / div;
    double y = (a1*c0 - a0*c1) / div;

    bool intersection_outside_XR;
    if (R == Eigen::Vector2f::Zero())
    {
      if (X[0] > 0)
        intersection_outside_XR = (x <= 0) || (x >= X[0]);
      else if (X[0] < 0)
        intersection_outside_XR = (x >= 0) || (x <= X[0]);
      else if (X[1] > 0)
        intersection_outside_XR = (y <= 0) || (y >= X[1]);
      else if (X[1] < 0)
        intersection_outside_XR = (y >= 0) || (y <= X[1]);
      else
        intersection_outside_XR = true;
    }
    else
    {
      if (X[0] > R[0])
        intersection_outside_XR = (x <= R[0]) || (x >= X[0]);
      else if (X[0] < R[0])
        intersection_outside_XR = (x >= R[0]) || (x <= X[0]);
      else if (X[1] > R[1])
        intersection_outside_XR = (y <= R[1]) || (y >= X[1]);
      else if (X[1] < R[1])
        intersection_outside_XR = (y >= R[1]) || (y <= X[1]);
      else
        intersection_outside_XR = true;
    }
    if (intersection_outside_XR)
      return true;
    if (S1[0] > S2[0])
      return (x <= S2[0]) || (x >= S1[0]);
    if (S1[0] < S2[0])
      return (x >= S2[0]) || (x <= S1[0]);
    if (S1[1] > S2[1])
      return (y <= S2[1]) || (y >= S1[1]);
    if (S1[1] < S2[1])                                                                                                                     
      return (y >= S2[1]) || (y <= S1[1]);
    return false;
  }

  /** \brief GreedyProjectionTriangulation is an implementation of a greedy triangulation algorithm for 3D points
    * based on local 2D projections. It assumes locally smooth surfaces and relatively smooth transitions between
    * areas with different point densities.
    * \author Zoltan Csaba Marton
    * \ingroup surface
    */
  template <typename PointInT>
  class GreedyProjectionTriangulation : public MeshConstruction<PointInT>
  {
    public:
      using Ptr = shared_ptr<GreedyProjectionTriangulation<PointInT> >;
      using ConstPtr = shared_ptr<const GreedyProjectionTriangulation<PointInT> >;

      using MeshConstruction<PointInT>::tree_;
      using MeshConstruction<PointInT>::input_;
      using MeshConstruction<PointInT>::indices_;

      using KdTree = pcl::KdTree<PointInT>;
      using KdTreePtr = typename KdTree::Ptr;

      using PointCloudIn = pcl::PointCloud<PointInT>;
      using PointCloudInPtr = typename PointCloudIn::Ptr;
      using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;

      enum GP3Type
      { 
        NONE = -1,    // not-defined
        FREE = 0,    
        FRINGE = 1,  
        BOUNDARY = 2,
        COMPLETED = 3
      };
    
      /** \brief Empty constructor. */
      GreedyProjectionTriangulation () : 
        mu_ (0), 
        search_radius_ (0), // must be set by user
        nnn_ (100),
        minimum_angle_ (M_PI/18), // 10 degrees
        maximum_angle_ (2*M_PI/3), // 120 degrees
        eps_angle_(M_PI/4), //45 degrees,
        consistent_(false), 
        consistent_ordering_ (false),
        angles_ (),
        R_ (),
        is_current_free_ (false),
        current_index_ (),
        prev_is_ffn_ (false),
        prev_is_sfn_ (false),
        next_is_ffn_ (false),
        next_is_sfn_ (false),
        changed_1st_fn_ (false),
        changed_2nd_fn_ (false),
        new2boundary_ (),
        already_connected_ (false)
      {};

      /** \brief Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point
       *  (this will make the algorithm adapt to different point densities in the cloud).
        * \param[in] mu the multiplier
        */
      inline void 
      setMu (double mu) { mu_ = mu; }

      /** \brief Get the nearest neighbor distance multiplier. */
      inline double 
      getMu () const { return (mu_); }

      /** \brief Set the maximum number of nearest neighbors to be searched for.
        * \param[in] nnn the maximum number of nearest neighbors
        */
      inline void 
      setMaximumNearestNeighbors (int nnn) { nnn_ = nnn; }

      /** \brief Get the maximum number of nearest neighbors to be searched for. */
      inline int 
      getMaximumNearestNeighbors () const { return (nnn_); }

      /** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors used for triangulating.
        * \param[in] radius the sphere radius that is to contain all k-nearest neighbors
        * \note This distance limits the maximum edge length!
        */
      inline void 
      setSearchRadius (double radius) { search_radius_ = radius; }

      /** \brief Get the sphere radius used for determining the k-nearest neighbors. */
      inline double 
      getSearchRadius () const { return (search_radius_); }

      /** \brief Set the minimum angle each triangle should have.
        * \param[in] minimum_angle the minimum angle each triangle should have
        * \note As this is a greedy approach, this will have to be violated from time to time
        */
      inline void 
      setMinimumAngle (double minimum_angle) { minimum_angle_ = minimum_angle; }

      /** \brief Get the parameter for distance based weighting of neighbors. */
      inline double 
      getMinimumAngle () const { return (minimum_angle_); }

      /** \brief Set the maximum angle each triangle can have.
        * \param[in] maximum_angle the maximum angle each triangle can have
        * \note For best results, its value should be around 120 degrees
        */
      inline void 
      setMaximumAngle (double maximum_angle) { maximum_angle_ = maximum_angle; }

      /** \brief Get the parameter for distance based weighting of neighbors. */
      inline double 
      getMaximumAngle () const { return (maximum_angle_); }

      /** \brief Don't consider points for triangulation if their normal deviates more than this value from the query point's normal.
        * \param[in] eps_angle maximum surface angle
        * \note As normal estimation methods usually give smooth transitions at sharp edges, this ensures correct triangulation
        *       by avoiding connecting points from one side to points from the other through forcing the use of the edge points.
        */
      inline void 
      setMaximumSurfaceAngle (double eps_angle) { eps_angle_ = eps_angle; }

      /** \brief Get the maximum surface angle. */
      inline double 
      getMaximumSurfaceAngle () const { return (eps_angle_); }

      /** \brief Set the flag if the input normals are oriented consistently.
        * \param[in] consistent set it to true if the normals are consistently oriented
        */
      inline void 
      setNormalConsistency (bool consistent) { consistent_ = consistent; }

      /** \brief Get the flag for consistently oriented normals. */
      inline bool 
      getNormalConsistency () const { return (consistent_); }

      /** \brief Set the flag to order the resulting triangle vertices consistently (positive direction around normal).
        * @note Assumes consistently oriented normals (towards the viewpoint) -- see setNormalConsistency ()
        * \param[in] consistent_ordering set it to true if triangle vertices should be ordered consistently
        */
      inline void 
      setConsistentVertexOrdering (bool consistent_ordering) { consistent_ordering_ = consistent_ordering; }

      /** \brief Get the flag signaling consistently ordered triangle vertices. */
      inline bool 
      getConsistentVertexOrdering () const { return (consistent_ordering_); }

      /** \brief Get the state of each point after reconstruction.
        * \note Options are defined as constants: FREE, FRINGE, COMPLETED, BOUNDARY and NONE
        */
      inline std::vector<int> 
      getPointStates () const { return (state_); }

      /** \brief Get the ID of each point after reconstruction.
        * \note parts are numbered from 0, a -1 denotes unconnected points
        */
      inline std::vector<int> 
      getPartIDs () const { return (part_); }


      /** \brief Get the sfn list. */
      inline pcl::Indices
      getSFN () const { return (sfn_); }

      /** \brief Get the ffn list. */
      inline pcl::Indices
      getFFN () const { return (ffn_); }

    protected:
      /** \brief The nearest neighbor distance multiplier to obtain the final search radius. */
      double mu_;

      /** \brief The nearest neighbors search radius for each point and the maximum edge length. */
      double search_radius_;

      /** \brief The maximum number of nearest neighbors accepted by searching. */
      int nnn_;

      /** \brief The preferred minimum angle for the triangles. */
      double minimum_angle_;

      /** \brief The maximum angle for the triangles. */
      double maximum_angle_;

      /** \brief Maximum surface angle. */
      double eps_angle_;

      /** \brief Set this to true if the normals of the input are consistently oriented. */
      bool consistent_;
      
      /** \brief Set this to true if the output triangle vertices should be consistently oriented. */
      bool consistent_ordering_;

     private:
      /** \brief Struct for storing the angles to nearest neighbors **/
      struct nnAngle
      {
        double angle;
        pcl::index_t index;
        pcl::index_t nnIndex;
        bool visible;
      };

      /** \brief Struct for storing the edges starting from a fringe point **/
      struct doubleEdge
      {
        doubleEdge () : index (0) {}
        int index;
        Eigen::Vector2f first;
        Eigen::Vector2f second;
      };

      // Variables made global to decrease the number of parameters to helper functions

      /** \brief Temporary variable to store a triangle (as a set of point indices) **/
      pcl::Vertices triangle_;
      /** \brief Temporary variable to store point coordinates **/
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > coords_;

      /** \brief A list of angles to neighbors **/
      std::vector<nnAngle> angles_;
      /** \brief Index of the current query point **/
      pcl::index_t R_;
      /** \brief List of point states **/
      std::vector<int> state_;
      /** \brief List of sources **/
      pcl::Indices source_;
      /** \brief List of fringe neighbors in one direction **/
      pcl::Indices ffn_;
      /** \brief List of fringe neighbors in other direction **/
      pcl::Indices sfn_;
      /** \brief Connected component labels for each point **/
      std::vector<int> part_;
      /** \brief Points on the outer edge from which the mesh has to be grown **/
      std::vector<int> fringe_queue_;

      /** \brief Flag to set if the current point is free **/
      bool is_current_free_;
      /** \brief Current point's index **/
      pcl::index_t current_index_;
      /** \brief Flag to set if the previous point is the first fringe neighbor **/
      bool prev_is_ffn_;
      /** \brief Flag to set if the next point is the second fringe neighbor **/
      bool prev_is_sfn_;
      /** \brief Flag to set if the next point is the first fringe neighbor **/
      bool next_is_ffn_;
      /** \brief Flag to set if the next point is the second fringe neighbor **/
      bool next_is_sfn_;
      /** \brief Flag to set if the first fringe neighbor was changed **/
      bool changed_1st_fn_;
      /** \brief Flag to set if the second fringe neighbor was changed **/
      bool changed_2nd_fn_;
      /** \brief New boundary point **/
      pcl::index_t new2boundary_;
      
      /** \brief Flag to set if the next neighbor was already connected in the previous step.
        * To avoid inconsistency it should not be connected again.
        */
      bool already_connected_; 

      /** \brief Point coordinates projected onto the plane defined by the point normal **/
      Eigen::Vector3f proj_qp_;
      /** \brief First coordinate vector of the 2D coordinate frame **/
      Eigen::Vector3f u_;
      /** \brief Second coordinate vector of the 2D coordinate frame **/
      Eigen::Vector3f v_;
      /** \brief 2D coordinates of the first fringe neighbor **/
      Eigen::Vector2f uvn_ffn_;
      /** \brief 2D coordinates of the second fringe neighbor **/
      Eigen::Vector2f uvn_sfn_;
      /** \brief 2D coordinates of the first fringe neighbor of the next point **/
      Eigen::Vector2f uvn_next_ffn_;
      /** \brief 2D coordinates of the second fringe neighbor of the next point **/
      Eigen::Vector2f uvn_next_sfn_;

      /** \brief Temporary variable to store 3 coordinates **/
      Eigen::Vector3f tmp_;

      /** \brief The actual surface reconstruction method.
        * \param[out] output the resultant polygonal mesh
        */
      void 
      performReconstruction (pcl::PolygonMesh &output) override;

      /** \brief The actual surface reconstruction method.
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      void 
      performReconstruction (std::vector<pcl::Vertices> &polygons) override;

      /** \brief The actual surface reconstruction method.
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      bool
      reconstructPolygons (std::vector<pcl::Vertices> &polygons);

      /** \brief Class get name method. */
      std::string 
      getClassName () const override { return ("GreedyProjectionTriangulation"); }

      /** \brief Forms a new triangle by connecting the current neighbor to the query point 
        * and the previous neighbor
        * \param[out] polygons the polygon mesh to be updated
        * \param[in] prev_index index of the previous point
        * \param[in] next_index index of the next point
        * \param[in] next_next_index index of the point after the next one
        * \param[in] uvn_current 2D coordinate of the current point
        * \param[in] uvn_prev 2D coordinates of the previous point
        * \param[in] uvn_next 2D coordinates of the next point
        */
      void 
      connectPoint (std::vector<pcl::Vertices> &polygons, 
                    const pcl::index_t prev_index, 
                    const pcl::index_t next_index, 
                    const pcl::index_t next_next_index, 
                    const Eigen::Vector2f &uvn_current, 
                    const Eigen::Vector2f &uvn_prev, 
                    const Eigen::Vector2f &uvn_next);

      /** \brief Whenever a query point is part of a boundary loop containing 3 points, that triangle is created
        * (called if angle constraints make it possible)
        * \param[out] polygons the polygon mesh to be updated
        */
      void 
      closeTriangle (std::vector<pcl::Vertices> &polygons);

      /** \brief Get the list of containing triangles for each vertex in a PolygonMesh
        * \param[in] polygonMesh the input polygon mesh
        */
      std::vector<std::vector<std::size_t> >
      getTriangleList (const pcl::PolygonMesh &input);

      /** \brief Add a new triangle to the current polygon mesh
        * \param[in] a index of the first vertex
        * \param[in] b index of the second vertex
        * \param[in] c index of the third vertex
        * \param[out] polygons the polygon mesh to be updated
        */
      inline void
      addTriangle (pcl::index_t a, pcl::index_t b, pcl::index_t c, std::vector<pcl::Vertices> &polygons)
      {
        triangle_.vertices.resize (3);
        if (consistent_ordering_)
        {
          const PointInT p = input_->at (indices_->at (a));
          const Eigen::Vector3f pv = p.getVector3fMap ();
          if (p.getNormalVector3fMap ().dot (
                (pv - input_->at (indices_->at (b)).getVector3fMap ()).cross (
                 pv - input_->at (indices_->at (c)).getVector3fMap ()) ) > 0)
          {
            triangle_.vertices[0] = a;
            triangle_.vertices[1] = b;
            triangle_.vertices[2] = c;
          }
          else
          {
            triangle_.vertices[0] = a;
            triangle_.vertices[1] = c;
            triangle_.vertices[2] = b;
          }
        }
        else
        {
          triangle_.vertices[0] = a;
          triangle_.vertices[1] = b;
          triangle_.vertices[2] = c;
        }
        polygons.push_back (triangle_);
      }

      /** \brief Add a new vertex to the advancing edge front and set its source point
        * \param[in] v index of the vertex that was connected
        * \param[in] s index of the source point
        */
      inline void
      addFringePoint (int v, int s)
      {
        source_[v] = s;
        part_[v] = part_[s];
        fringe_queue_.push_back(v);
      }

      /** \brief Function for ascending sort of nnAngle, taking visibility into account
        * (angles to visible neighbors will be first, to the invisible ones after).
        * \param[in] a1 the first angle
        * \param[in] a2 the second angle
        */
      static inline bool 
      nnAngleSortAsc (const nnAngle& a1, const nnAngle& a2)
      {
        if (a1.visible == a2.visible)
          return (a1.angle < a2.angle);
        return a1.visible;
      }
  };

} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/gp3.hpp>
#endif
