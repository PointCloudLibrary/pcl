#ifndef MESH_TOPOLOGY_HPP
#define MESH_TOPOLOGY_HPP

#include <stack>

#include <boost/type_traits/integral_constant.hpp>

#include <pcl/geometry/impl/mesh_base.hpp>

namespace pcl
{
  template <bool is_manifold, class VertexDataT, class FaceDataT, class HalfEdgeDataT>
  class MeshTopology : public pcl::MeshBase <VertexDataT, FaceDataT, HalfEdgeDataT>
  {
    public:

      typedef pcl::MeshTopology <is_manifold, VertexDataT, FaceDataT, HalfEdgeDataT> Self;
      typedef pcl::MeshBase     <             VertexDataT, FaceDataT, HalfEdgeDataT> Base;

      typedef boost::integral_constant<bool, true>        ManifoldMeshTag;
      typedef boost::integral_constant<bool, false>       NonManifoldMeshTag;
      typedef boost::integral_constant<bool, is_manifold> IsManifold;

      typedef typename Base::VertexData   VertexData;
      typedef typename Base::HalfEdgeData HalfEdgeData;
      typedef typename Base::FaceData     FaceData;

      typedef typename Base::VertexIndex   VertexIndex;
      typedef typename Base::HalfEdgeIndex HalfEdgeIndex;
      typedef typename Base::FaceIndex     FaceIndex;

      typedef typename Base::VertexIndexes   VertexIndexes;
      typedef typename Base::HalfEdgeIndexes HalfEdgeIndexes;
      typedef typename Base::FaceIndexes     FaceIndexes;

      typedef typename Base::Vertex   Vertex;
      typedef typename Base::HalfEdge HalfEdge;
      typedef typename Base::Face     Face;

      typedef typename Base::Vertexes  Vertexes;
      typedef typename Base::HalfEdges HalfEdges;
      typedef typename Base::Faces     Faces;

      typedef typename Base::SizeType SizeType;

      typedef typename Base::VertexIterator   VertexIterator;
      typedef typename Base::HalfEdgeIterator HalfEdgeIterator;
      typedef typename Base::FaceIterator     FaceIterator;

      typedef typename Base::VertexConstIterator   VertexConstIterator;
      typedef typename Base::HalfEdgeConstIterator HalfEdgeConstIterator;
      typedef typename Base::FaceConstIterator     FaceConstIterator;

      typedef typename Base::VertexIndexIterator   VertexIndexIterator;
      typedef typename Base::HalfEdgeIndexIterator HalfEdgeIndexIterator;
      typedef typename Base::FaceIndexIterator     FaceIndexIterator;

      typedef typename Base::VertexIndexConstIterator   VertexIndexConstIterator;
      typedef typename Base::HalfEdgeIndexConstIterator HalfEdgeIndexConstIterator;
      typedef typename Base::FaceIndexConstIterator     FaceIndexConstIterator;

      typedef typename Base::VertexAroundVertexCirculator           VertexAroundVertexCirculator;
      typedef typename Base::OutgoingHalfEdgeAroundVertexCirculator OutgoingHalfEdgeAroundVertexCirculator;
      typedef typename Base::IncomingHalfEdgeAroundVertexCirculator IncomingHalfEdgeAroundVertexCirculator;
      typedef typename Base::FaceAroundVertexCirculator             FaceAroundVertexCirculator;
      typedef typename Base::VertexAroundFaceCirculator             VertexAroundFaceCirculator;
      typedef typename Base::InnerHalfEdgeAroundFaceCirculator      InnerHalfEdgeAroundFaceCirculator;
      typedef typename Base::OuterHalfEdgeAroundFaceCirculator      OuterHalfEdgeAroundFaceCirculator;
      typedef typename Base::HalfEdgeAroundBoundaryCirculator       HalfEdgeAroundBoundaryCirculator;

      typedef typename Base::VertexAroundVertexConstCirculator           VertexAroundVertexConstCirculator;
      typedef typename Base::OutgoingHalfEdgeAroundVertexConstCirculator OutgoingHalfEdgeAroundVertexConstCirculator;
      typedef typename Base::IncomingHalfEdgeAroundVertexConstCirculator IncomingHalfEdgeAroundVertexConstCirculator;
      typedef typename Base::FaceAroundVertexConstCirculator             FaceAroundVertexConstCirculator;
      typedef typename Base::VertexAroundFaceConstCirculator             VertexAroundFaceConstCirculator;
      typedef typename Base::InnerHalfEdgeAroundFaceConstCirculator      InnerHalfEdgeAroundFaceConstCirculator;
      typedef typename Base::OuterHalfEdgeAroundFaceConstCirculator      OuterHalfEdgeAroundFaceConstCirculator;
      typedef typename Base::HalfEdgeAroundBoundaryConstCirculator       HalfEdgeAroundBoundaryConstCirculator;

    public:

      MeshTopology ()
        : Base ()
      {
      }

      MeshTopology (const Self& other)
        : Base (other)
      {
      }

    public:

      //////////////////////////////////////////////////////////////////////////
      // isManifold
      //////////////////////////////////////////////////////////////////////////

      inline bool
      isManifold () const
      {
        return (this->isManifold (IsManifold ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // addVertex
      //////////////////////////////////////////////////////////////////////////

      inline VertexIndex
      addVertex (const VertexData& vertex_data)
      {
        return (Base::pushBackVertex (vertex_data));
      }

      //////////////////////////////////////////////////////////////////////////
      // deleteVertex
      //////////////////////////////////////////////////////////////////////////

      void
      deleteVertex (const VertexIndex& idx_vertex)
      {
        assert (Base::validateVertexIndex (idx_vertex));

        FaceAroundVertexConstCirculator       circ     = Base::getFaceAroundVertexConstCirculator (idx_vertex);
        const FaceAroundVertexConstCirculator circ_end = circ;

        do
        {
          const FaceIndex idx_face = (circ++).getDereferencedIndex ();
          if (idx_face.isValid ())
          {
            this->deleteFace (idx_face);
          }
        } while (circ!=circ_end);
      }

      //////////////////////////////////////////////////////////////////////////
      // deleteEdge
      //////////////////////////////////////////////////////////////////////////

      void
      deleteEdge (const HalfEdgeIndex& idx_half_edge)
      {
        assert (Base::validateHalfEdgeIndex (idx_half_edge));

        const HalfEdge& he = Base::getElement (idx_half_edge);
        this->deleteFace (he.getFaceIndex ());
        this->deleteFace (he.getOppositeFaceIndex (*this));
      }

      //////////////////////////////////////////////////////////////////////////
      // deleteFace
      //////////////////////////////////////////////////////////////////////////

      inline void
      deleteFace (const FaceIndex &idx_face)
      {
        assert (Base::validateFaceIndex (idx_face));

        std::stack<FaceIndex> delete_faces; // This is only needed for a manifold mesh. It should be removed by the compiler during the optimization for a non-manifold mesh.

        return (this->deleteFace (idx_face, delete_faces, IsManifold ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // cleanUp
      //////////////////////////////////////////////////////////////////////////

      void cleanUp (const bool remove_isolated = true) // TODO: what is better default value?
      {
        // Copy the non-isolated mesh elements and store the index to their new position
        const VertexIndexes   new_vertex_indexes    = this->removeMeshElements <Vertexes,  VertexIndexes>   (Base::vertexes_, remove_isolated);
        const HalfEdgeIndexes new_half_edge_indexes = this->removeMeshElements <HalfEdges, HalfEdgeIndexes> (Base::half_edges_, remove_isolated);
        const FaceIndexes     new_face_indexes      = this->removeMeshElements <Faces,     FaceIndexes>     (Base::faces_, remove_isolated);

        // Adjust the indexes
        VertexIterator              it_v      = Base::beginVertexes ();
        const VertexConstIterator   it_v_end  = Base::endVertexes ();
        HalfEdgeIterator            it_he     = Base::beginHalfEdges ();
        const HalfEdgeConstIterator it_he_end = Base::endHalfEdges ();
        FaceIterator                it_f      = Base::beginFaces ();
        const FaceConstIterator     it_f_end  = Base::endFaces ();

        for (; it_v!=it_v_end; ++it_v)
        {
          if (!it_v->isIsolated ())
          {
            it_v->setOutgoingHalfEdgeIndex (new_half_edge_indexes [it_v->getOutgoingHalfEdgeIndex ().idx ()]);
          }
        }

        for (; it_he!=it_he_end; ++it_he)
        {
          it_he->setTerminatingVertexIndex (new_vertex_indexes [it_he->getTerminatingVertexIndex ().idx ()]);
          it_he->setOppositeHalfEdgeIndex (new_half_edge_indexes [it_he->getOppositeHalfEdgeIndex ().idx ()]);
          it_he->setNextHalfEdgeIndex (new_half_edge_indexes [it_he->getNextHalfEdgeIndex ().idx ()]);
          it_he->setPrevHalfEdgeIndex (new_half_edge_indexes [it_he->getPrevHalfEdgeIndex ().idx ()]);
          if (!it_he->isBoundary ())
          {
            it_he->setFaceIndex (new_face_indexes [it_he->getFaceIndex ().idx ()]);
          }
        }

        for (; it_f!=it_f_end; ++it_f)
        {
          it_f->setInnerHalfEdgeIndex (new_half_edge_indexes [it_f->getInnerHalfEdgeIndex ().idx ()]);
        }
      }

    protected:

      //////////////////////////////////////////////////////////////////////////
      // firstTopologyCheck
      //////////////////////////////////////////////////////////////////////////

      // is_new must be initialized with true!
      // Returns true if addFace may be continued
      inline bool
      firstTopologyCheck (const VertexIndex& idx_v_a,
                          const VertexIndex& idx_v_b,
                          HalfEdgeIndex&     idx_he_a_out,
                          bool&              is_new_ab) const
      {
        return (this->firstTopologyCheck (idx_v_a, idx_v_b, idx_he_a_out, is_new_ab, IsManifold ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // secondTopologyCheck
      //////////////////////////////////////////////////////////////////////////

      inline bool
      secondTopologyCheck (const bool is_new_ab,
                           const bool is_new_bc,
                           const bool is_isolated_b) const
      {
        return (this->secondTopologyCheck (is_new_ab, is_new_bc, is_isolated_b, IsManifold ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // makeAdjacent
      //////////////////////////////////////////////////////////////////////////

      inline void
      makeAdjacent (const HalfEdgeIndex& idx_he_ab,
                    const HalfEdgeIndex& idx_he_bc)
      {
        return (this->makeAdjacent (idx_he_ab,idx_he_bc, IsManifold ()));
      }

      //////////////////////////////////////////////////////////////////////////
      // addHalfEdgePair
      //////////////////////////////////////////////////////////////////////////

      inline void
      addHalfEdgePair (const VertexIndex&  idx_v_a,
                       const VertexIndex&  idx_v_b,
                       const HalfEdgeData& he_data_ab,
                       const HalfEdgeData& he_data_ba,
                       HalfEdgeIndex&      idx_he_ab,
                       HalfEdgeIndex&      idx_he_ba)
      {
        // Only sets the unambiguous connections: OppositeHalfEdge and TerminatingVertex
        idx_he_ab = Base::pushBackHalfEdge (he_data_ab, idx_v_b);
        idx_he_ba = Base::pushBackHalfEdge (he_data_ba, idx_v_a, idx_he_ab);

        Base::getElement (idx_he_ab).setOppositeHalfEdgeIndex (idx_he_ba);
      }

      //////////////////////////////////////////////////////////////////////////
      // connectPrevNext
      //////////////////////////////////////////////////////////////////////////

      inline void
      connectPrevNext (const HalfEdgeIndex& idx_he_ab,
                       const HalfEdgeIndex& idx_he_bc)
      {
        Base::getElement (idx_he_ab).setNextHalfEdgeIndex (idx_he_bc);
        Base::getElement (idx_he_bc).setPrevHalfEdgeIndex (idx_he_ab);
      }

      //////////////////////////////////////////////////////////////////////////
      // connect
      //////////////////////////////////////////////////////////////////////////

      inline void
      connectHalfEdges (const bool           is_new_ab,
                        const bool           is_new_bc,
                        const HalfEdgeIndex& idx_he_ab,
                        const HalfEdgeIndex& idx_he_ba,
                        const HalfEdgeIndex& idx_he_bc,
                        const HalfEdgeIndex& idx_he_cb,
                        const VertexIndex&   idx_v_b)
      {
        if      ( is_new_ab &&  is_new_bc) this->connectNewNew (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb, idx_v_b, IsManifold ());
        else if ( is_new_ab && !is_new_bc) this->connectNewOld (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb, idx_v_b);
        else if (!is_new_ab &&  is_new_bc) this->connectOldNew (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb, idx_v_b);
        else                               this->connectOldOld (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb, idx_v_b, IsManifold ());
      }

      //////////////////////////////////////////////////////////////////////////
      // connectFace
      //////////////////////////////////////////////////////////////////////////

      inline FaceIndex
      connectFace (const FaceData&                  face_data,
                   HalfEdgeIndexConstIterator       it,
                   const HalfEdgeIndexConstIterator it_end)
      {
        // Add and connect the face
        const FaceIndex idx_face = Base::pushBackFace (face_data, *(it_end-1));

        while (it!=it_end)
        {
          Base::getElement (*it++).setFaceIndex (idx_face);
        }

        return (idx_face);
      }

    private:

      //////////////////////////////////////////////////////////////////////////
      // isManifold
      //////////////////////////////////////////////////////////////////////////

      inline bool
      isManifold (ManifoldMeshTag) const
      {
        return (true);
      }

      inline bool
      isManifold (NonManifoldMeshTag) const
      {
        VertexConstIterator       it     = Base::beginVertexes ();
        const VertexConstIterator it_end = Base::endVertexes ();

        for (; it!=it_end; ++it)
        {
          if (!it->isManifold (*this))
          {
            return (false);
          }
        }
        return (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // deleteFace
      //////////////////////////////////////////////////////////////////////////

      void
      deleteFace (const FaceIndex&       idx_face,
                  std::stack<FaceIndex>& delete_faces,
                  ManifoldMeshTag)
      {
        delete_faces.push (idx_face);

        while (!delete_faces.empty ())
        {
          const FaceIndex& idx_face_cur = delete_faces.top ();
          delete_faces.pop ();

          // This calls the non-manifold version of deleteFace, which will call the manifold version of reconnect (avoid code duplication).
          this->deleteFace (idx_face_cur, delete_faces, NonManifoldMeshTag ());
        }
      }

      void
      deleteFace (const FaceIndex&       idx_face,
                  std::stack<FaceIndex>& delete_faces,
                  NonManifoldMeshTag)
      {
        assert (Base::validateFaceIndex (idx_face));

        Face& face = Base::getElement (idx_face);

        if (face.getDeleted ())
        {
          return; // already deleted
        }

        // Get all inner & outer half-edges in the face and store if the outer half-edge is a boundary
        typedef std::vector <EdgeWithBoundaryInfo>               EdgeWithBoundaryInfoVec;
        typedef typename EdgeWithBoundaryInfoVec::const_iterator EdgeWithBoundaryInfoConstIterator;

        // TODO: maybe use a deque instead of vector here (no reserve necessary)
        EdgeWithBoundaryInfoVec ewbiv; ewbiv.reserve (4); // Minimum for a triangle

        InnerHalfEdgeAroundFaceCirculator       circ     = Base::getInnerHalfEdgeAroundFaceCirculator(face);
        const InnerHalfEdgeAroundFaceCirculator circ_end = circ;

        do
        {
          ewbiv.push_back (EdgeWithBoundaryInfo (circ.getDereferencedIndex (),
                                                 circ->getOppositeHalfEdgeIndex (),
                                                 circ->getOppositeHalfEdge (*this).isBoundary ()));
          ++circ;
        } while (circ!=circ_end);

        // This is needed to reconnect the last half-edge with the first half-edge in the face
        ewbiv.push_back (ewbiv.front ());

        // Reconnect
        EdgeWithBoundaryInfoConstIterator       it     = ewbiv.begin ();
        const EdgeWithBoundaryInfoConstIterator it_end = ewbiv.end ()-1;

        while (it!=it_end)
        {
          const EdgeWithBoundaryInfo& cur  = *it++;
          const EdgeWithBoundaryInfo& next = *it;

          this->reconnect (cur.is_boundary_ba_,next.is_boundary_ba_, cur.idx_he_ab_,cur.idx_he_ba_, next.idx_he_ab_,next.idx_he_ba_, delete_faces);
        }

        // Delete the face
        face.setDeleted (true);
        it = ewbiv.begin ();
        while (it!=it_end)
        {
          Base::getElement ((*it++).idx_he_ab_).getFaceIndex ().invalidate ();
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // firstTopologyCheck
      //////////////////////////////////////////////////////////////////////////

      inline bool
      firstTopologyCheck (const VertexIndex& idx_v_a,
                          const VertexIndex& idx_v_b,
                          HalfEdgeIndex&     idx_he_a_out,
                          bool&              is_new_ab,
                          ManifoldMeshTag) const
      {
        const Vertex& v_a = Base::getElement (idx_v_a);

        if (v_a.isIsolated ()) return (true);

        idx_he_a_out          = v_a.getOutgoingHalfEdgeIndex ();
        const HalfEdge& he_ab = Base::getElement (idx_he_a_out);

        if (!he_ab.isBoundary ())                          return (false);
        if (he_ab.getTerminatingVertexIndex () == idx_v_b) is_new_ab = false;

        return (true);
      }

      inline bool
      firstTopologyCheck (const VertexIndex& idx_v_a,
                          const VertexIndex& idx_v_b,
                          HalfEdgeIndex&     idx_he_a_out,
                          bool&              is_new_ab,
                          NonManifoldMeshTag) const
      {
        const Vertex& v_a = Base::getElement (idx_v_a);

        if (v_a.isIsolated ())                              return (true);
        if (!v_a.getOutgoingHalfEdge (*this).isBoundary ()) return (false);

        VertexAroundVertexConstCirculator       circ     = Base::getVertexAroundVertexConstCirculator (v_a.getOutgoingHalfEdgeIndex ());
        const VertexAroundVertexConstCirculator circ_end = circ;

        do
        {
          if (circ.getDereferencedIndex () == idx_v_b)
          {
            idx_he_a_out = circ.getCurrentHalfEdgeIndex ();
            if (!Base::getElement (idx_he_a_out).isBoundary ())
            {
              return (false);
            }

            is_new_ab = false;
            return (true);
          }
          ++circ;
        } while (circ!=circ_end);

        return (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // secondTopologyCheck
      //////////////////////////////////////////////////////////////////////////

      inline bool
      secondTopologyCheck (const bool is_new_ab,
                           const bool is_new_bc,
                           const bool is_isolated_b,
                           ManifoldMeshTag) const
      {
        if (is_new_ab && is_new_bc && !is_isolated_b) return (false);
        else                                          return (true);
      }

      inline bool
      secondTopologyCheck (const bool is_new_ab,
                           const bool is_new_bc,
                           const bool is_isolated_b,
                           NonManifoldMeshTag) const
      {
        return (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // makeAdjacent
      //////////////////////////////////////////////////////////////////////////

      inline void
      makeAdjacent (const HalfEdgeIndex& idx_he_ab,
                    const HalfEdgeIndex& idx_he_bc,
                    ManifoldMeshTag)
      {
        // Nothing to do here (manifold)
      }

      inline void
      makeAdjacent (const HalfEdgeIndex& idx_he_ab,
                    const HalfEdgeIndex& idx_he_bc,
                    NonManifoldMeshTag)
      {
        if (Base::getElement (idx_he_ab).getNextHalfEdgeIndex () == idx_he_bc)
        {
          return; // already adjacent
        }

        // Find the next boundary half edge (counter-clockwise around vertex b)
        OutgoingHalfEdgeAroundVertexConstCirculator circ = Base::getOutgoingHalfEdgeAroundVertexConstCirculator (Base::getElement (idx_he_ab).getOppositeHalfEdgeIndex ());

        while (true)
        {
          // The first half-edge (he_ba) is not on the boundary -> circ is incremented at least once
          // If re-linking is necessary then there are at least three boundary half-edges -> no infinite loop
          if (circ->isBoundary ())
          {
            // Re-link. No references!
            const HalfEdgeIndex idx_he_ab_next       = Base::getElement (idx_he_ab).getNextHalfEdgeIndex ();
            const HalfEdgeIndex idx_he_bc_prev       = Base::getElement (idx_he_bc).getPrevHalfEdgeIndex ();
            const HalfEdgeIndex idx_he_boundary      = circ.getDereferencedIndex ();
            const HalfEdgeIndex idx_he_boundary_prev = Base::getElement (idx_he_boundary).getPrevHalfEdgeIndex ();

            this->connectPrevNext (idx_he_ab, idx_he_bc);
            this->connectPrevNext (idx_he_boundary_prev, idx_he_ab_next);
            this->connectPrevNext (idx_he_bc_prev, idx_he_boundary);

            return;
          }
          ++circ;
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // connectNewNEw
      //////////////////////////////////////////////////////////////////////////

      inline void
      connectNewNew (const HalfEdgeIndex& idx_he_ab,
                     const HalfEdgeIndex& idx_he_ba,
                     const HalfEdgeIndex& idx_he_bc,
                     const HalfEdgeIndex& idx_he_cb,
                     const VertexIndex&   idx_v_b,
                     ManifoldMeshTag)
      {
        this->connectPrevNext (idx_he_ab, idx_he_bc);
        this->connectPrevNext (idx_he_cb, idx_he_ba);

        Base::getElement (idx_v_b).setOutgoingHalfEdgeIndex (idx_he_ba);
      }

      inline void
      connectNewNew (const HalfEdgeIndex& idx_he_ab,
                     const HalfEdgeIndex& idx_he_ba,
                     const HalfEdgeIndex& idx_he_bc,
                     const HalfEdgeIndex& idx_he_cb,
                     const VertexIndex&   idx_v_b,
                     NonManifoldMeshTag)
      {
        const Vertex& v_b = Base::getElement (idx_v_b);

        if (v_b.isIsolated ())
        {
          this->connectNewNew (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb, idx_v_b, ManifoldMeshTag ());
        }
        else
        {
          // No references!
          const HalfEdgeIndex idx_he_b_out      = v_b.getOutgoingHalfEdgeIndex ();
          const HalfEdgeIndex idx_he_b_out_prev = Base::getElement (idx_he_b_out).getPrevHalfEdgeIndex ();

          this->connectPrevNext (idx_he_ab, idx_he_bc);
          this->connectPrevNext (idx_he_cb, idx_he_b_out);
          this->connectPrevNext (idx_he_b_out_prev, idx_he_ba);
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // connectNewOld
      //////////////////////////////////////////////////////////////////////////

      inline void
      connectNewOld (const HalfEdgeIndex& idx_he_ab,
                     const HalfEdgeIndex& idx_he_ba,
                     const HalfEdgeIndex& idx_he_bc,
                     const HalfEdgeIndex& idx_he_cb,
                     const VertexIndex&   idx_v_b)
      {
        const HalfEdgeIndex idx_he_bc_prev = Base::getElement (idx_he_bc).getPrevHalfEdgeIndex (); // No reference!

        this->connectPrevNext (idx_he_ab, idx_he_bc);
        this->connectPrevNext (idx_he_bc_prev, idx_he_ba);

        Base::getElement (idx_v_b).setOutgoingHalfEdgeIndex (idx_he_ba);
      }

      //////////////////////////////////////////////////////////////////////////
      // connectOldNew
      //////////////////////////////////////////////////////////////////////////

      inline void
      connectOldNew (const HalfEdgeIndex& idx_he_ab,
                     const HalfEdgeIndex& idx_he_ba,
                     const HalfEdgeIndex& idx_he_bc,
                     const HalfEdgeIndex& idx_he_cb,
                     const VertexIndex&   idx_v_b)
      {
        const HalfEdgeIndex idx_he_ab_next = Base::getElement (idx_he_ab).getNextHalfEdgeIndex (); // No reference!

        this->connectPrevNext (idx_he_ab, idx_he_bc);
        this->connectPrevNext (idx_he_cb, idx_he_ab_next);

        Base::getElement (idx_v_b).setOutgoingHalfEdgeIndex (idx_he_ab_next);
      }

      //////////////////////////////////////////////////////////////////////////
      // connectOldOld
      //////////////////////////////////////////////////////////////////////////

      inline void
      connectOldOld (const HalfEdgeIndex& idx_he_ab,
                     const HalfEdgeIndex& idx_he_ba,
                     const HalfEdgeIndex& idx_he_bc,
                     const HalfEdgeIndex& idx_he_cb,
                     const VertexIndex&   idx_v_b,
                     ManifoldMeshTag)
      {
        // Nothing to do here (manifold)
      }

      inline void
      connectOldOld (const HalfEdgeIndex& idx_he_ab,
                     const HalfEdgeIndex& idx_he_ba,
                     const HalfEdgeIndex& idx_he_bc,
                     const HalfEdgeIndex& idx_he_cb,
                     const VertexIndex&   idx_v_b,
                     NonManifoldMeshTag)
      {
        Vertex& v_b = Base::getElement (idx_v_b);

        // The outgoing half edge MUST be a boundary half-edge (if there is one)
        if (v_b.getOutgoingHalfEdgeIndex () == idx_he_bc) // he_bc is no longer on the boundary
        {
          OutgoingHalfEdgeAroundVertexConstCirculator       circ     = Base::getOutgoingHalfEdgeAroundVertexConstCirculator (v_b);
          const OutgoingHalfEdgeAroundVertexConstCirculator circ_end = circ++;
          do
          {
            if (circ->isBoundary ())
            {
              v_b.setOutgoingHalfEdgeIndex (circ.getDereferencedIndex ());
              return;
            }
            ++circ;
          } while (circ!=circ_end);
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // reconnect
      //////////////////////////////////////////////////////////////////////////

      inline void
      reconnect (const bool             is_boundary_ba,
                 const bool             is_boundary_cb,
                 const HalfEdgeIndex&   idx_he_ab,
                 const HalfEdgeIndex&   idx_he_ba,
                 const HalfEdgeIndex&   idx_he_bc,
                 const HalfEdgeIndex&   idx_he_cb,
                 std::stack<FaceIndex>& delete_faces)
      {
        if      (is_boundary_ba && is_boundary_cb)  this->reconnectBB   (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb);
        else if (is_boundary_ba && !is_boundary_cb) this->reconnectBNB  (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb);
        else if (!is_boundary_ba && is_boundary_cb) this->reconnectNBB  (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb);
        else                                        this->reconnectNBNB (idx_he_ab,idx_he_ba, idx_he_bc,idx_he_cb, delete_faces, IsManifold ());
      }

      //////////////////////////////////////////////////////////////////////////
      // reconnectBB (BoundaryBoundary)
      //////////////////////////////////////////////////////////////////////////

      inline void
      reconnectBB (const HalfEdgeIndex& idx_he_ab,
                   const HalfEdgeIndex& idx_he_ba,
                   const HalfEdgeIndex& idx_he_bc,
                   const HalfEdgeIndex& idx_he_cb)
      {
        const HalfEdgeIndex& idx_he_cb_next = Base::getElement (idx_he_cb).getNextHalfEdgeIndex ();

        if (idx_he_cb_next == idx_he_ba) // Vertex b is isolated
        {
          Base::getElement (idx_he_ab).getTerminatingVertex (*this).setDeleted (true);
        }
        else
        {
          this->connectPrevNext (Base::getElement (idx_he_ba).getPrevHalfEdgeIndex (), idx_he_cb_next);
          Base::getElement (idx_he_ab).getTerminatingVertex (*this).setOutgoingHalfEdgeIndex (idx_he_cb_next);
        }

        Base::getElement (idx_he_ab).setDeleted (true);
        Base::getElement (idx_he_ba).setDeleted (true);
        // he_bc.setDeleted (true); // already done in reconnectBNB or reconnectBB
        // he_cb.setDeleted (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // reconnectBNB (BoundaryNoBoundary)
      //////////////////////////////////////////////////////////////////////////

      inline void
      reconnectBNB (const HalfEdgeIndex& idx_he_ab,
                    const HalfEdgeIndex& idx_he_ba,
                    const HalfEdgeIndex& idx_he_bc,
                    const HalfEdgeIndex& idx_he_cb)
      {
        this->connectPrevNext (Base::getElement (idx_he_ba).getPrevHalfEdgeIndex (), idx_he_bc);
        Base::getElement (idx_he_ab).getTerminatingVertex (*this).setOutgoingHalfEdgeIndex (idx_he_bc);

        Base::getElement (idx_he_ab).setDeleted (true);
        Base::getElement (idx_he_ba).setDeleted (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // reconnectNBB (NoBoundaryBoundary)
      //////////////////////////////////////////////////////////////////////////

      inline void
      reconnectNBB (const HalfEdgeIndex& idx_he_ab,
                    const HalfEdgeIndex& idx_he_ba,
                    const HalfEdgeIndex& idx_he_bc,
                    const HalfEdgeIndex& idx_he_cb)
      {
        const HalfEdgeIndex& idx_he_cb_next = Base::getElement (idx_he_cb).getNextHalfEdgeIndex ();
        this->connectPrevNext (idx_he_ab, idx_he_cb_next);
        Base::getElement (idx_he_ab).getTerminatingVertex (*this).setOutgoingHalfEdgeIndex (idx_he_cb_next);

        // he_bc.setDeleted (true); // already done in reconnectBB
        // he_cb.setDeleted (true);
      }

      //////////////////////////////////////////////////////////////////////////
      // reconnectNBNB (NoBoundaryNoBoundary)
      //////////////////////////////////////////////////////////////////////////

      inline void
      reconnectNBNB (const HalfEdgeIndex&   idx_he_ab,
                     const HalfEdgeIndex&   idx_he_ba,
                     const HalfEdgeIndex&   idx_he_bc,
                     const HalfEdgeIndex&   idx_he_cb,
                     std::stack<FaceIndex>& delete_faces,
                     ManifoldMeshTag)
      {
        Vertex& v_b = Base::getElement (idx_he_ab).getTerminatingVertex (*this);

        if(v_b.isBoundary (*this))
        {
          // Deletion of this face makes the mesh non-manifold
          // -> delete the neighbouring faces until it is manifold again
          FaceAroundVertexConstCirculator circ = Base::getFaceAroundVertexConstCirculator (idx_he_ba);

          FaceIndex idx_face = (circ++).getDereferencedIndex ();

          while (idx_face.isValid ())
          {
            delete_faces.push (idx_face);
            idx_face = (circ++).getDereferencedIndex ();
          }
        }

        v_b.setOutgoingHalfEdgeIndex (idx_he_bc);
      }

      inline void
      reconnectNBNB (const HalfEdgeIndex&   idx_he_ab,
                     const HalfEdgeIndex&   idx_he_ba,
                     const HalfEdgeIndex&   idx_he_bc,
                     const HalfEdgeIndex&   idx_he_cb,
                     std::stack<FaceIndex>& delete_faces,
                     NonManifoldMeshTag)
      {
        Vertex& v_b = Base::getElement (idx_he_ab).getTerminatingVertex (*this);

        if(!v_b.isBoundary (*this))
        {
          v_b.setOutgoingHalfEdgeIndex (idx_he_bc);
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // removeMeshElements
      //////////////////////////////////////////////////////////////////////////

      template <class MeshElementsT, class MeshElementIndexesT> inline MeshElementIndexesT
      removeMeshElements (MeshElementsT& mesh_elements,
                          const bool     remove_isolated) const
      {
        typedef typename MeshElementsT::value_type      MeshElement;

        typedef MeshElementIndexesT                     MeshElementIndexes;
        typedef typename MeshElementIndexes::value_type MeshElementIndex;
        typedef typename MeshElementIndexes::iterator   MeshElementIndexIterator;

        MeshElementIndexes new_mesh_element_indexes (mesh_elements.size (), MeshElementIndex ());
        MeshElementIndex   ind_old (0), ind_new (0);

        MeshElementIndexIterator       it_mei_new     = new_mesh_element_indexes.begin ();
        const MeshElementIndexIterator it_mei_new_end = new_mesh_element_indexes.end ();

        for (; it_mei_new!=it_mei_new_end; ++it_mei_new, ++ind_old)
        {
          MeshElement& me_old = mesh_elements [ind_old.idx ()];

          if (!(me_old.getDeleted () || (remove_isolated && me_old.isIsolated ())))
          {
            if (ind_new != ind_old) // avoid self assignment
            {
              mesh_elements [ind_new.idx ()] = me_old;
            }

            *it_mei_new = ind_new++;
          }
        }

        mesh_elements.resize (ind_new.idx ());

        return (new_mesh_element_indexes);
      }

    private:

      struct EdgeWithBoundaryInfo
      {
          EdgeWithBoundaryInfo (const HalfEdgeIndex& idx_he_ab,
                                const HalfEdgeIndex& idx_he_ba,
                                const bool           is_boundary_ba)
            : idx_he_ab_      (idx_he_ab),
              idx_he_ba_      (idx_he_ba),
              is_boundary_ba_ (is_boundary_ba)
          {
          }

          HalfEdgeIndex idx_he_ab_;
          HalfEdgeIndex idx_he_ba_;
          bool          is_boundary_ba_;
      };

  };

} // End namespace pcl

#endif // MESH_TOPOLOGY_HPP
