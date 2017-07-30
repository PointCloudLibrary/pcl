/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
  
#ifndef PCL_BALL_PIVOTING_FRONT_H_
#define PCL_BALL_PIVOTING_FRONT_H_

namespace pcl
{
  namespace ball_pivoting
  {
    /**
     * BallPivotingFront manages the edges: which ones are to be pivoted, which ones are pivoted.
     */
    class BallPivotingFront
    {
    public:
      typedef std::pair<uint32_t, uint32_t> Signature;

      /**
       * This class describes the pivoting ball, which rolls on points and links the mesh.
       * Its partial definition describes an edge of the reconstructed triangle.
       */
      class Edge
      {
      protected:
        /** list of vertice index, should have length 2 */
        std::vector<uint32_t> id_vertices_;
        /** index of the opposite vertice, it and id_vertices_ form the last triangle */
        uint32_t id_opposite_;
        /** center point of the last ball, it may not be one in the cloud */
        Eigen::Vector3f center_;
        /** whether this ball it to the it rolled in the back face of surface (not normal direction) */
        bool is_back_ball_;
    
      public:
        Edge ();
    
        /**
         * a fake constructor with only vertice index on edge
         * @param id0 index of start point on edge
         * @param id1 index of end point on edge
         */
        Edge (const uint32_t id0, const uint32_t id1);
    
        /**
         * real constructor for edge with full information
         * @param edge list of vertice index on edge
         * @param id_opposite index of opposite vertice
         * @param center center point of the ball
         * @param is_back_ball whether the ball was rolling on the back surface
         */
        Edge (const std::vector<uint32_t> &edge, const uint32_t id_opposite, 
              const Eigen::Vector3f &center, const bool is_back_ball = false);
    
        ~Edge ();
    
        /**
         * returns the center of ball
         * @return
         */
        Eigen::Vector3f
        getCenter () const
        { return center_; }
    
        /**
         * get the index of id-th vertice on edge, id should be either 0 or 1
         * @param id
         * @return
         */
        uint32_t
        getIdVertice (const size_t id) const
        { return id_vertices_.at (id); }
    
        /**
         * get the index of opposite vertice
         * @return
         */
        uint32_t
        getIdOpposite () const
        { return id_opposite_; }
    
        /**
         * set the index of id-th vertice to be id_vertice
         * @param id index inside edge. Should be 0 or 1
         * @param id_vertice value of vertice index
         */
        void
        setIdVertice (const size_t id, const uint32_t id_vertice)
        { id_vertices_.at (id) = id_vertice; }
    
        /**
         * checks whether the ball was rolling on back surface
         * @return
         */
        bool
        isBackBall () const
        { return is_back_ball_; }
    
        /**
         * get the signature of this edge, [index start vertice, index end vertice]
         * @return
         */
        Signature
        getSignature () const
        { return Signature (id_vertices_.at (0), id_vertices_.at (1)); }
    
        /**
         * get the reverse signature of this edge, [index end vertice, index start vertice]
         * @return
         */
        Signature
        getSignatureReverse () const
        { return Signature (id_vertices_.at (1), id_vertices_.at (0)); }
    
        typedef boost::shared_ptr<Edge> Ptr;
        typedef boost::shared_ptr<Edge const> ConstPtr;
      };

    protected:
      /** The set of edges to be pivoted */
      std::map<Signature, Edge> active_edges;
      /** The set of successfully pivoted edges */
      std::set<Signature> finished_signatures_;
      /** The edge that is being pivoted and waiting for pivoting result: boundary or pivoted? */
      Edge::Ptr current_edge_;
  
    public:
      BallPivotingFront ();
  
      ~BallPivotingFront ();
  
      /**
       * get one of the edges to pivot
       * @return
       */
      Edge::Ptr
      getActiveEdge ();
  
      /**
       * add the three edges of one triangle to edges to pivot
       * @param seed index of three vertices
       * @param center the center of the ball
       * @param is_back_ball whether the ball was pivoted on the back surface
       */
      void
      addTriangle (const pcl::Vertices::ConstPtr &seed, const Eigen::Vector3f &center, 
      	           const bool is_back_ball);
  
      /**
       * extend the edges with one pivoted edge and vertice on new ball. the vertice on new ball should
       * form two edges with the start vertice and end vertice of the old edge
       * @param last_edge the edge which was pivoted and contributes two vertices to new edges
       * @param id_vetice_extended index of the vertice to form new edges
       * @param center center of the new ball
       * @param is_back_ball whether ball is rolling on back surface
       */
      void
      addPoint (const Edge &last_edge, const uint32_t id_vetice_extended, 
      	        const Eigen::Vector3f &center, const bool is_back_ball);
  
      /**
       * add one edge to edges to pivot
       * @param edge
       */
      void
      addEdge (const Edge &edge);
  
      /**
       * checks whether edge is one the edges to pivot
       * @param edge
       * @return
       */
      bool
      isEdgeOnFront (const Edge &edge) const;
  
      /**
       * removes the edge with signature [id0,id1] or [id1,id0] in edges to pivoted
       * @param id0
       * @param id1
       */
      void
      removeEdge (const uint32_t id0, const uint32_t id1);
  
      /**
       * reset the front
       */
      void
      clear ();
  
      /**
       * checks whether edge or the reversed edge is pivoted
       * @param edge
       * @return
       */
      bool
      isEdgeFinished (const Edge &edge) const;
  
      typedef boost::shared_ptr<BallPivotingFront> Ptr;
      typedef boost::shared_ptr<BallPivotingFront const> ConstPtr;
    };
  }
}

#endif // PCL_BALL_PIVOTING_FRONT_H_
