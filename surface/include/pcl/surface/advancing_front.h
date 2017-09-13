/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_SURFACE_ADVANCING_FRONT_H_
#define PCL_SURFACE_ADVANCING_FRONT_H_

#define PCL_NO_PRECOMPILE
#include <deque>
#include <pcl/surface/reconstruction.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/surface/advancing_front_utils.h>
#include <pcl/surface/mls.h>

#ifdef AFRONTDEBUG
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#endif

namespace pcl
{

  /** \brief The advancing front reconstruction algorithm. This implementaton is based on the following paper.
    *
    * Advancing Front Papers:
    *
    * 1. High-Quality Extraction of Isosurfaces from Regular and Irregular Grids
    *    J. Schreiner, C. Scheidegger, C. Silva
    *    IEEE Transactions on Visualization and Computer Graphics (Proceedings of IEEE Visualization); 2006
    *
    * 2. Direct (Re)Meshing for Efficient Surface Processing
    *    J. Schreiner, C. Scheidegger, S. Fleishman, C. Silva
    *    Computer Graphics Forum (Proceedings of Eurographics); 2006
    *
    * 3. Triangulating Point-Set Surfaces With Bounded Error
    *    C. Scheidegger, S. Fleishman, C. Silva
    *    Proceedings of the third Eurographics/ACM Symposium on Geometry Processing; 2005
    *
    * \author Levi H. Armstrong
    * \group surface
    */
  template <typename PointNT>
  class AfrontMesher : public SurfaceReconstruction<PointNT>
  {
    struct MeshTraits
    {
      typedef pcl::afront::AfrontVertexPointType VertexData;
      typedef int HalfEdgeData;
      typedef int EdgeData;
      typedef pcl::PointNormal FaceData;

      typedef boost::false_type IsManifold;
    };

    typedef pcl::geometry::PolygonMesh<MeshTraits> Mesh;

    typedef typename Mesh::VertexIndex VertexIndex;
    typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;
    typedef typename Mesh::FaceIndex FaceIndex;

    typedef typename Mesh::VertexIndices VertexIndices;
    typedef typename Mesh::HalfEdgeIndices HalfEdgeIndices;
    typedef typename Mesh::FaceIndices FaceIndices;

    typedef typename Mesh::VertexAroundVertexCirculator VAVC;
    typedef typename Mesh::OutgoingHalfEdgeAroundVertexCirculator OHEAVC;
    typedef typename Mesh::IncomingHalfEdgeAroundVertexCirculator IHEAVC;
    typedef typename Mesh::FaceAroundVertexCirculator FAVC;
    typedef typename Mesh::VertexAroundFaceCirculator VAFC;
    typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;
    typedef typename Mesh::OuterHalfEdgeAroundFaceCirculator OHEAFC;

    struct SamplePointResults
    {
      pcl::PointXYZ orig;                       /**< \brief The point to be projected on to the MLS surface */
      pcl::afront::AfrontVertexPointType point; /**< \brief The point projected on to the MLS surface */
      int closest;                              /**< \brief The closest point index on the MLS surface to the project point */
      pcl::MLSResult mls;                       /**< \brief The MLS Results for the closest point */
      double dist;                              /**< \brief The distance squared between point and closest */
    };

    struct TriangleData
    {
      double A;                   /**< \brief The length for the first half edge (p1->p2) */
      double B;                   /**< \brief The length for the second half edge (p2->p3) */
      double C;                   /**< \brief The length for the remaining side of the triangle (p3->p1) */
      double a;                   /**< \brief The angle BC */
      double b;                   /**< \brief The angle AC */
      double c;                   /**< \brief The anble AB */
      double aspect_ratio;        /**< \brief The quality of the triangle (1.0 is the best) */
      Eigen::Vector3f normal;     /**< \brief The normal of the triangle */
      Eigen::Vector3f p[3];       /**< \brief Stores the point information for the triangle */
      bool point_valid;           /**< \brief Indicates if the triangle is valid. Both edges must point in the same direction as the grow direction. */
      bool vertex_normals_valid;  /**< \brief The vertex normals are not within some tolerance */
      bool triangle_normal_valid; /**< \brief The triangle normal is not within some tolerance to the vertex normals*/
      bool valid;                 /**< \brief If all condition are valid: point_valid, vertex_normals_valid and triangle_normal_valid */

      /** /brief Print triangle data
        * /param[in] A description to be printed with the data.
        */
      void
      print (const std::string description = "") const
      {
        if (description == "")
          std::printf ("Triangle Data:\n");
        else
          std::printf ("Triangle Data (%s):\n", description.c_str ());

        std::printf ("\t  A: %-10.4f  B: %-10.4f  C: %-10.4f\n", A, B, C);
        std::printf ("\t  a: %-10.4f  b: %-10.4f  c: %-10.4f\n", a, b, c);
        std::printf ("\t x1: %-10.4f y1: %-10.4f z1: %-10.4f\n", p[0][0], p[0][1], p[0][2]);
        std::printf ("\t x2: %-10.4f y2: %-10.4f z2: %-10.4f\n", p[1][0], p[1][1], p[1][2]);
        std::printf ("\t x3: %-10.4f y3: %-10.4f z3: %-10.4f\n", p[2][0], p[2][1], p[2][2]);
        std::printf ("\t Aspect Ratio: %-10.4f\n", aspect_ratio);
        std::printf ("\t Valid: %s\n", valid ? "true" : "false");
      }
    };

    struct FrontData
    {
      HalfEdgeIndex he;              /**< \brief The half edge index from which to grow the triangle */
      double length;                 /**< \brief The half edge length */
      double max_step;               /**< \brief The maximum grow distance */
      double max_step_search_radius; /**< \brief The approximate search radius for the finding max step for new point. */
      Eigen::Vector3f mp;            /**< \brief The half edge mid point */
      Eigen::Vector3f d;             /**< \brief The half edge grow direction */
      VertexIndex vi[2];             /**< \brief The half edge vertex indicies */
      Eigen::Vector3f p[2];          /**< \brief The half edge points (Origninating, Terminating) */
      Eigen::Vector3f n[2];          /**< \brief The half edge point normals (Origninating, Terminating) */
    };

    struct CutEarData
    {
      HalfEdgeIndex primary;   /**< \brief The advancing front half edge */
      HalfEdgeIndex secondary; /**< \brief The Secondary half edge triangle (Previouse or Next) */
      VertexIndex vi[3];       /**< \brief The vertex indicies of the potential triangle */
      TriangleData tri;        /**< \brief The Triangle information */
    };

    struct AdvancingFrontData
    {
      FrontData front; /**< \brief The front data */
      CutEarData prev; /**< \brief The results using the previous half edge */
      CutEarData next; /**< \brief The results using the next half edge */
    };

    struct PredictVertexResults
    {
      enum PredictVertexTypes
      {
        Valid = 0,                 /**< \brief The project point is valid. */
        AtBoundary = 1,            /**< \brief At the point cloud boundary. */
        InvalidStepSize = 2,       /**< \brief The step size is invalid for the given half edge (2 * step size < front.length). */
        InvalidVertexNormal = 3,   /**< \brief The projected points normal is not consistant with the other triangle normals. */
        InvalidTriangleNormal = 4, /**< \brief The triangle normal created by the project point is not consistant with the vertex normals. */
        InvalidMLSResults = 5,     /**< \brief The nearest points mls results are invalid. */
        InvalidProjection = 6      /**< \brief The projected point is not in the grow direction */
      };

      AdvancingFrontData afront; /**< \brief Advancing front data */
      TriangleData tri;          /**< \brief The proposed triangle data */
      SamplePointResults pv;     /**< \brief The predicted point projected on the mls surface */
      Eigen::Vector2d k;         /**< \brief The principal curvature using the polynomial */
      PredictVertexTypes status; /**< \brief The predicted vertex is near the boundry of the point cloud. Don't Create Triangle */
    };

    struct CloseProximityResults
    {
      CloseProximityResults () : dist (0.0), found (false) {}

      std::vector<VertexIndex> verticies; /**< \brief The valid mesh verticies. */
      std::vector<HalfEdgeIndex> fences;  /**< \brief The valid half edges. */
      VertexIndex closest;                /**< \brief The closest mesh vertex */
      TriangleData tri;                   /**< \brief The triangle data created by the closest point. */
      double dist;                        /**< \brief This stores closest distance information. */
      bool found;                         /**< \brief If close proximity was found. */
    };

    struct FenceViolationResults
    {
      FenceViolationResults () : dist (0.0), found (false) {}

      HalfEdgeIndex he;                               /**< \brief The half edge index that was violated. */
      int index;                                      /**< \brief The index in the array CloseProximityResults.fences. */
      pcl::afront::IntersectionLine2PlaneResults lpr; /**< \brief The line to plane intersection results for fence violations. */
      double dist;                                    /**< \brief The distance from the intersection point and the advancing front. */
      bool found;                                     /**< \brief If a mesh half edge was violated. */
    };

    struct TriangleToCloseResults
    {
      TriangleToCloseResults () : found (false) {}

      PredictVertexResults pvr; /**< \brief The predicted vertex information provided */
      VertexIndex closest;      /**< \brief The closest vertex index */
      TriangleData tri;         /**< \brief The Triangle information */
      bool found;               /**< \brief True if triangle is close, otherwise false */
    };

  public:

#ifdef _OPENMP
    static const int    AFRONT_DEFAULT_THREADS = 1;
#endif
    static const int    AFRONT_DEFAULT_SAMPLE_SIZE = 0;
    static const int    AFRONT_DEFAULT_POLYNOMIAL_ORDER = 2;
    static const double AFRONT_DEFAULT_REDUCTION = 0.8;
    static const double AFRONT_DEFAULT_RHO = 0.9;
    static const double AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD = M_PI_2;
    static const double AFRONT_ASPECT_RATIO_TOLERANCE = 0.85;
    static const double AFRONT_CLOSE_PROXIMITY_FACTOR = 0.5;
    static const double AFRONT_FENCE_HEIGHT_FACTOR = 2.0;

    /** \brief AfrontMesher Constructor */
    AfrontMesher ();

    /** \brief AfrontMesher Destructor */
    ~AfrontMesher () {}

    /** \brief Get the current polygon mesh */
    inline Mesh
    getHalfEdgeMesh () const
    {
      return mesh_;
    }

    /** \brief Get the mesh vertex normals */
    pcl::PointCloud<pcl::Normal>::ConstPtr
    getMeshVertexNormals () const;

#ifdef AFRONTDEBUG
    /**  \brief Get the internal viewer */
    pcl::visualization::PCLVisualizer::Ptr
    getViewer ()
    {
      return viewer_;
    }
#endif

    /** \brief Set the primary variable used to control mesh triangulation size. (0 > rho < M_PI_2) */
    inline void
    setRho (double val)
    {
      if (val >= M_PI_2 || val <= 0)
      {
        PCL_ERROR ("AFront rho must be between 0 and PI/2. Using default value.\n");
        rho_ = AFRONT_DEFAULT_RHO;
      }
      else
      {
        rho_ = val;
      }

      hausdorff_error_ = (1.0 - sqrt ((1.0 + 2.0 * cos (rho_)) / 3.0)) * (1.0 / (2.0 * sin (rho_ / 2)));
      updateTriangleTolerances ();
    }

    /** \brief Get the primary variable used to control mesh triangulation size */
    inline double
    getRho () const
    {
      return rho_;
    }

    /** \brief Set how fast can the mesh grow and shrink. (val < 1.0) */
    inline void
    setReduction (double val)
    {
      if (val >= 1 || val <= 0)
      {
        PCL_ERROR ("AFront reduction must be between 0 and 1. Using default value.\n");
        reduction_ = AFRONT_DEFAULT_REDUCTION;
      }
      else
      {
        reduction_ = val;
      }
      updateTriangleTolerances ();
    }

    /** \brief Get the variable that controls how fast the mesh can grow and shrink. */
    inline double
    getReduction () const
    {
      return reduction_;
    }

    /** \brief Set the mls radius used for smoothing */
    inline void
    setSearchRadius (double val)
    {
      search_radius_ = val;
    }

    /** \brief Get the mls radius used for smoothing */
    inline double
    getSearchRadius () const
    {
      return search_radius_;
    }

    /** \brief Set the mls polynomial order. (order > 1) */
    inline void
    setPolynomialOrder (const int order)
    {
      if (order < 2)
      {
        PCL_ERROR ("AFront polynomial order must be greater than 1. Using default value.\n");
        polynomial_order_ = AFRONT_DEFAULT_POLYNOMIAL_ORDER;
      }
      else
      {
        polynomial_order_ = order;
      }

      int nr_coeff = (polynomial_order_ + 1) * (polynomial_order_ + 2) / 2;
      required_neighbors_ = 5 * nr_coeff;
    }

    /** \brief Get the mls polynomial order */
    inline int
    getPolynomialOrder () const
    {
      return polynomial_order_;
    }

    /** \brief Set the number of sample triangles to generate.
      * \note num_samples = 0 will mesh the entire PointCloud.
      */
    inline void
    setSampleSize (const int num_samples)
    {
      if (num_samples < 0)
      {
        PCL_ERROR ("AFront sample size must be greater than or equal to zero. Using default value.\n");
        samples_ = AFRONT_DEFAULT_SAMPLE_SIZE;
      }
      else
      {
        samples_ = num_samples;
      }
    }

    /** \brief Get the number of sample triangles to generate. */
    inline int
    getSampleSize () const
    {
      return samples_;
    }

    /** \brief Set the boundary angle threshold used to determine if a point is on the boundary of the point cloud. */
    inline void
    setBoundaryAngleThreshold (const double angle)
    {
      if (angle <= 0)
      {
        PCL_ERROR ("AFront boundary angle threshold must be greater than 0. Using default value.\n");
        boundary_angle_threshold_ = AFRONT_DEFAULT_BOUNDARY_ANGLE_THRESHOLD;
      }
      else
      {
        boundary_angle_threshold_ = angle;
      }
    }

    /** \brief Get the boundary angle threshold used to determine if a point is on the boundary of the point cloud. */
    inline double
    getBoundaryAngleThreshold () const
    {
      return boundary_angle_threshold_;
    }

#ifdef _OPENMP
    /** \brief Set the number of threads to use */
    inline void
    setNumberOfThreads (const int threads)
    {
      if (threads <= 0)
      {
        PCL_ERROR ("AFront number of threads must be greater than 0. Using default value.\n");
        threads_ = AFRONT_DEFAULT_THREADS;
      }
      else
      {
        threads_ = threads;
      }
    }

    /** \brief Get the number of threads to use */
    inline int
    getNumberOfThreads ()
    {
      return threads_;
    }
#endif

    /** \brief Print all of the meshes vertices */
    void
    printVertices () const;

    /** \brief Print all of the meshes faces */
    void
    printFaces () const;

    /** \brief Print a given half edges vertices */
    void
    printEdge (const HalfEdgeIndex &half_edge) const;

    /** \brief Print a given face's information */
    void
    printFace (const FaceIndex &idx_face) const;

  private:
    /** \brief This will process the input parameters and comput the guidance field */
    bool
    initialize ();

    /** \brief Extract the surface.
      * \param[out] output the resultant polygonal mesh
      */
    void
    performReconstruction (pcl::PolygonMesh &output);

    /** \brief Extract the surface.
      * \param[out] points the points of the extracted mesh
      * \param[out] polygons the connectivity between the point of the extracted mesh.
      */
    void
    performReconstruction (pcl::PointCloud<PointNT> &points,
                           std::vector<pcl::Vertices> &polygons);

    /** \brief Advance the mesh by adding one triangle */
    void
    stepReconstruction (const long unsigned int id);

    /** \brief Indicates if it has finished meshing the surface */
    bool
    isFinished ()
    {
      return finished_;
    }

    /** \brief Create the first triangle given a starting location. */
    void
    createFirstTriangle (const int &index);
    void
    createFirstTriangle (const double &x, const double &y, const double &z);

    /** \brief Get the predicted vertex for the provided front */
    PredictVertexResults
    predictVertex (const AdvancingFrontData &afront) const;

    /** \brief Check if features of the existing mesh are in close proximity to the proposed new triangle. */
    CloseProximityResults
    isCloseProximity (const PredictVertexResults &pvr) const;

    /**
      * \brief Check if a line intersects a mesh half edge fence.
      * \param[in] sp Origin point of the line
      * \param[in] ep Terminating point of the line
      * \param[in] fence The half edge index
      * \param[in] fence_height The height of the fence.
      * \param[in] lpr The results of the line plane intersection
      * \return True if line interesects fence, otherwise false
      */
    bool
    isFenceViolated (const Eigen::Vector3f &sp, const Eigen::Vector3f &ep, const HalfEdgeIndex &fence, const double fence_height, pcl::afront::IntersectionLine2PlaneResults &lpr) const;

    /**
      * \brief Check if a line intersects a list of fences.
      * \param[in] vi The vertex index representing the origin of the line.
      * \param[in] p The terminating point of the line
      * \param[in] fences The list of half edge indexes
      * \param[in] closest The closest point in the mesh if one exists
      * \param[in] pvr The predicted vertex results data
      * \return FenceViolationResults
      */
    FenceViolationResults
    isFencesViolated (const VertexIndex &vi, const Eigen::Vector3f &p, const std::vector<HalfEdgeIndex> &fences, const VertexIndex &closest, const PredictVertexResults &pvr) const;

    /**
      * \brief Check if the proposed triangle interferes with the previous or next half edge.
      *
      * If it does interfere the input variable tri will be modified to represent a tri created
      * by either the previous or next half edge.
      *
      * \param[in] afront The advancing front data
      * \param[out] tri The proposed triangle
      * \param[out] closest The closest point in the mesh if one exists
      * \return True if it does interfere, otherwise false
      */
    bool
    checkPrevNextHalfEdge (const AdvancingFrontData &afront, TriangleData &tri, VertexIndex &closest) const;

    /** \brief Check if the proposed triangle is to close to the existing mesh. */
    TriangleToCloseResults
    isTriangleToClose (const PredictVertexResults &pvr) const;

    /** \brief Grow a triangle */
    void
    grow (const PredictVertexResults &pvr);

    /** \brief Merge triangle with the existing mesh */
    void
    merge (const TriangleToCloseResults &ttcr);

    /** \brief Perform an ear cut operation */
    void
    cutEar (const CutEarData &ccer);

    SamplePointResults
    samplePoint (float x, float y, float z) const;
    SamplePointResults
    samplePoint (const pcl::afront::AfrontGuidanceFieldPointType &pt) const;

    /** \brief Used to get the next half edge */
    CutEarData
    getNextHalfEdge (const FrontData &front) const;

    /** \brief Used to get the previous half edge */
    CutEarData
    getPrevHalfEdge (const FrontData &front) const;

    /** \brief Add half edge to queue */
    bool
    addToQueue (const FaceIndex &face);
    void
    addToQueueHelper (const HalfEdgeIndex &half_edge);

    /** \brief Remove half edge from queue */
    void
    removeFromQueue (const HalfEdgeIndex &half_edge);
    void
    removeFromQueue (const HalfEdgeIndex &half_edge1, const HalfEdgeIndex &half_edge2);

    /** \brief Add half edge to boundary */
    void
    addToBoundary (const HalfEdgeIndex &half_edge);

    /** \brief Remove half edge from boundary */
    void
    removeFromBoundary (const HalfEdgeIndex &half_edge);
    void
    removeFromBoundary (const HalfEdgeIndex &half_edge1, const HalfEdgeIndex &half_edge2);

    /**
      * \brief This creates the MLS surface.
      * \return True if successful, otherwise false.
      */
    bool
    computeGuidanceField ();

    /**
      * \brief This calculates useful data about the advancing front used throughout.
      * \param[in] half_edge Advancing half edge index
      * \return Data about the advancing half edge
      */
    AdvancingFrontData
    getAdvancingFrontData (const HalfEdgeIndex &half_edge) const;

    /**
      * \brief Create face data to be stored with the face. Currently it stores the center
      * of the triangle and the normal of the face.
      */
    typename MeshTraits::FaceData
    createFaceData (const TriangleData &tri) const;

    /**
      * \brief Get the dirction to grow for a given half edge
      * \param[in] p A vertex of the half edge
      * \param[in] mp The mid point of the half edge
      * \param[in] fd The face data associated to the opposing half edge
      * \return The grow direction vector
      */
    Eigen::Vector3f
    getGrowDirection (const Eigen::Vector3f &p, const Eigen::Vector3f &mp, const typename MeshTraits::FaceData &fd) const;

    /**
       * \brief Get the maximum step required for a given point
       * \param[in] p The point for which to determine the max step
       * \param[out] radius_found The radius at which the criteria was meet.
       * \return The max step required
       */
    double
    getMaxStep (const Eigen::Vector3f &p, float &radius_found) const;

    /**
      * \brief Calculate triangle information.
      * \param[in] front The advancing front
      * \param[in] p Third point of triangle
      * \return Returns information about the triangle: angles, edge lengths, etc.
      */
    TriangleData
    getTriangleData (const FrontData &front, const pcl::afront::AfrontVertexPointType &p) const;

    /** \brief Update the allowed triangle tolerances. */
    void
    updateTriangleTolerances ()
    {
      vertex_normal_tol_ = (1.0 + AFRONT_CLOSE_PROXIMITY_FACTOR) * (rho_ / reduction_);
      triangle_normal_tol_ = vertex_normal_tol_ / 2.0;
    }

    /** \brief Check if a point is in the grow direction of the front. */
    bool
    isPointValid (const FrontData &front, const Eigen::Vector3f p) const;

    /** \brief Check if the front is at or near the boundary of the point cloud. */
    bool
    nearBoundary (const FrontData &front, const int index) const;

    /** \brief This is a direct copy of the pcl isBoundaryPoint function */
    bool
    isBoundaryPoint (const int index) const;

#ifdef AFRONTDEBUG
    /** \brief Generate a plygon mesh that represent the mls polynomial surface. */
    pcl::PolygonMesh
    getPolynomialSurface (const PredictVertexResults &pvr, const double step) const;

    /**
     * \brief keyboardEventOccurred
     * \param event
     * \return
     */
    void
    keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void *);
#endif

    // User defined data
    using SurfaceReconstruction<PointNT>::input_;
    using SurfaceReconstruction<PointNT>::indices_;
    using SurfaceReconstruction<PointNT>::tree_;

    double rho_;           /**< \brief The angle of the osculating circle where a triangle edge should optimally subtend */
    double reduction_;     /**< \brief The allowed percent reduction from triangle to triangle. */
    double search_radius_; /**< \brief The search radius used by mls */
    int polynomial_order_; /**< \brief The degree of the polynomial used by mls */
    int samples_;          /**< \brief The number of sample triangle to create. */
#ifdef _OPENMP
    int threads_;          /**< \brief The number of threads to be used by mls */
#endif

    // Algorithm Data
    double hausdorff_error_;
    double max_edge_length_;          /**< \brief This can be used to calculate the max error fo the reconstruction (max_edge_length_ * hausdorff_error_) */
    int required_neighbors_;          /**< \brief This the required number of neighbors for a given point found during the MLS. */
    double vertex_normal_tol_;        /**< \brief The angle tolerance for vertex normals for a given triangle */
    double triangle_normal_tol_;      /**< \brief The angle tolerance for the triangle normal relative to vertex normals */
    double boundary_angle_threshold_; /**< \brief The boundary angle threshold */

    // Guidance field data
    pcl::PointCloud<pcl::afront::AfrontGuidanceFieldPointType>::Ptr mls_cloud_;
    pcl::search::KdTree<pcl::afront::AfrontGuidanceFieldPointType>::Ptr mls_cloud_tree_;
    pcl::PointIndicesPtr mls_corresponding_input_indices_;
#ifdef _OPENMP
    pcl::MovingLeastSquaresOMP<PointNT, pcl::afront::AfrontGuidanceFieldPointType> mls_;
#else
    pcl::MovingLeastSquares<PointNT, pcl::afront::AfrontGuidanceFieldPointType> mls_;
#endif
    double max_curvature_;
    double min_curvature_;

    // Generated data
    Mesh mesh_; /**< The mesh object for inserting faces/vertices */
    typename pcl::PointCloud<typename MeshTraits::VertexData>::Ptr mesh_vertex_data_ptr_;
    typename pcl::octree::OctreePointCloudSearch<typename MeshTraits::VertexData>::Ptr mesh_octree_;
    pcl::IndicesPtr mesh_vertex_data_indices_;

    // Algorithm Status Data
    std::deque<HalfEdgeIndex> queue_;
    std::vector<HalfEdgeIndex> boundary_;
    bool finished_;
    bool initialized_;

#ifdef AFRONTDEBUG
    // Debug
    mutable long unsigned int fence_counter_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
#endif
  };
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/advancing_front.hpp>
#endif

#endif // PCL_SURFACE_ADVANCING_FRONT_H_
