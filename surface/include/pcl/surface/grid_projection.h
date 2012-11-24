/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_SURFACE_GRID_PROJECTION_H_
#define PCL_SURFACE_GRID_PROJECTION_H_

#include <pcl/surface/boost.h>
#include <pcl/surface/reconstruction.h>

namespace pcl
{
  /** \brief The 12 edges of a cell. */
  const int I_SHIFT_EP[12][2] = {
    {0, 4}, {1, 5}, {2, 6}, {3, 7}, 
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4}
  };

  const int I_SHIFT_PT[4] = {
    0, 4, 5, 7
  };

  const int I_SHIFT_EDGE[3][2] = {
    {0,1}, {1,3}, {1,2}
  };


  /** \brief Grid projection surface reconstruction method.
    * \author Rosie Li
    *
    * \note If you use this code in any academic work, please cite:
    *   - Ruosi Li, Lu Liu, Ly Phan, Sasakthi Abeysinghe, Cindy Grimm, Tao Ju.
    *     Polygonizing extremal surfaces with manifold guarantees.
    *     In Proceedings of the 14th ACM Symposium on Solid and Physical Modeling, 2010.
     * \ingroup surface
    */
  template <typename PointNT>
  class GridProjection : public SurfaceReconstruction<PointNT>
  {
    public:
      typedef boost::shared_ptr<GridProjection<PointNT> > Ptr;
      typedef boost::shared_ptr<const GridProjection<PointNT> > ConstPtr;

      using SurfaceReconstruction<PointNT>::input_;
      using SurfaceReconstruction<PointNT>::tree_;

      typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

      typedef typename pcl::KdTree<PointNT> KdTree;
      typedef typename pcl::KdTree<PointNT>::Ptr KdTreePtr;

      /** \brief Data leaf. */
      struct Leaf
      {
        Leaf () : data_indices (), pt_on_surface (), vect_at_grid_pt () {}

        std::vector<int> data_indices;
        Eigen::Vector4f pt_on_surface; 
        Eigen::Vector3f vect_at_grid_pt;
      };

      typedef boost::unordered_map<int, Leaf, boost::hash<int>, std::equal_to<int>, Eigen::aligned_allocator<int> > HashMap;

      /** \brief Constructor. */ 
      GridProjection ();

      /** \brief Constructor. 
        * \param in_resolution set the resolution of the grid
        */ 
      GridProjection (double in_resolution);

      /** \brief Destructor. */
      ~GridProjection ();

      /** \brief Set the size of the grid cell
        * \param resolution  the size of the grid cell
        */
      inline void 
      setResolution (double resolution)
      {
        leaf_size_ = resolution;
      }

      inline double 
      getResolution () const
      {
        return (leaf_size_);
      }

      /** \brief When averaging the vectors, we find the union of all the input data 
        *  points within the padding area,and do a weighted average. Say if the padding
        *  size is 1, when we process cell (x,y,z), we will find union of input data points
        *  from (x-1) to (x+1), (y-1) to (y+1), (z-1) to (z+1)(in total, 27 cells). In this
        *  way, even the cells itself doesnt contain any data points, we will stil process it
        *  because there are data points in the padding area. This can help us fix holes which 
        *  is smaller than the padding size.
        * \param padding_size The num of padding cells we want to create 
        */
      inline void 
      setPaddingSize (int padding_size)
      {
        padding_size_ = padding_size;
      }
      inline int 
      getPaddingSize () const
      {
        return (padding_size_);
      }

      /** \brief Set this only when using the k nearest neighbors search 
        * instead of finding the point union
        * \param k The number of nearest neighbors we are looking for
        */
      inline void 
      setNearestNeighborNum (int k)
      {
        k_ = k;
      }
      inline int 
      getNearestNeighborNum () const
      {
        return (k_);
      }

      /** \brief Binary search is used in projection. given a point x, we find another point
        *  which is 3*cell_size_ far away from x. Then we do a binary search between these 
        *  two points to find where the projected point should be.
        */
      inline void 
      setMaxBinarySearchLevel (int max_binary_search_level)
      {
        max_binary_search_level_ = max_binary_search_level;
      }
      inline int 
      getMaxBinarySearchLevel () const
      {
        return (max_binary_search_level_);
      }

      ///////////////////////////////////////////////////////////
      inline const HashMap& 
      getCellHashMap () const
      {
        return (cell_hash_map_);
      }

      inline const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& 
      getVectorAtDataPoint () const
      {
        return (vector_at_data_point_);
      }
      
      inline const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& 
      getSurface () const
      {
        return (surface_);
      }

    protected:
      /** \brief Get the bounding box for the input data points, also calculating the
        * cell size, and the gaussian scale factor
        */
      void 
      getBoundingBox ();

      /** \brief The actual surface reconstruction method.
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      bool
      reconstructPolygons (std::vector<pcl::Vertices> &polygons);

      /** \brief Create the surface. 
        *
        * The 1st step is filling the padding, so that all the cells in the padding
        * area are in the hash map. The 2nd step is store the vector, and projected
        * point. The 3rd step is finding all the edges intersects the surface, and
        * creating surface.
        *
        * \param[out] output the resultant polygonal mesh
        */
      void 
      performReconstruction (pcl::PolygonMesh &output);

      /** \brief Create the surface. 
        *
        * The 1st step is filling the padding, so that all the cells in the padding
        * area are in the hash map. The 2nd step is store the vector, and projected
        * point. The 3rd step is finding all the edges intersects the surface, and
        * creating surface.
        *
        * \param[out] points the resultant points lying on the surface
        * \param[out] polygons the resultant polygons, as a set of vertices. The Vertices structure contains an array of point indices.
        */
      void 
      performReconstruction (pcl::PointCloud<PointNT> &points, 
                             std::vector<pcl::Vertices> &polygons);

      /** \brief When the input data points don't fill into the 1*1*1 box, 
        * scale them so that they can be filled in the unit box. Otherwise, 
        * it will be some drawing problem when doing visulization
        * \param scale_factor scale all the input data point by scale_factor
        */
      void 
      scaleInputDataPoint (double scale_factor);

      /** \brief Get the 3d index (x,y,z) of the cell based on the location of
        * the cell
        * \param p the coordinate of the input point
        * \param index the output 3d index
        */
      inline void 
      getCellIndex (const Eigen::Vector4f &p, Eigen::Vector3i& index) const
      {
        for (int i = 0; i < 3; ++i)
          index[i] = static_cast<int> ((p[i] - min_p_(i)) / leaf_size_);
      }

      /** \brief Given the 3d index (x, y, z) of the cell, get the 
        * coordinates of the cell center
        * \param index the output 3d index
        * \param center the resultant cell center
        */
      inline void
      getCellCenterFromIndex (const Eigen::Vector3i &index, Eigen::Vector4f &center) const
      {
        for (int i = 0; i < 3; ++i)
          center[i] = 
            min_p_[i] + static_cast<float> (index[i]) * 
            static_cast<float> (leaf_size_) + 
            static_cast<float> (leaf_size_) / 2.0f;
      }

      /** \brief Given cell center, caluate the coordinates of the eight vertices of the cell
        * \param cell_center the coordinates of the cell center
        * \param pts the coordinates of the 8 vertices
        */
      void 
      getVertexFromCellCenter (const Eigen::Vector4f &cell_center, 
                               std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &pts) const;

      /** \brief Given an index (x, y, z) in 3d, translate it into the index 
        * in 1d
        * \param index the index of the cell in (x,y,z) 3d format
        */
      inline int 
      getIndexIn1D (const Eigen::Vector3i &index) const
      {
        //assert(data_size_ > 0);
        return (index[0] * data_size_ * data_size_ + 
                index[1] * data_size_ + index[2]);
      }

      /** \brief Given an index in 1d, translate it into the index (x, y, z) 
        * in 3d
        * \param index_1d the input 1d index
        * \param index_3d the output 3d index
        */
      inline void 
      getIndexIn3D (int index_1d, Eigen::Vector3i& index_3d) const
      {
        //assert(data_size_ > 0);
        index_3d[0] = index_1d / (data_size_ * data_size_);
        index_1d -= index_3d[0] * data_size_ * data_size_;
        index_3d[1] = index_1d / data_size_;
        index_1d -= index_3d[1] * data_size_;
        index_3d[2] = index_1d;
      }

      /** \brief For a given 3d index of a cell, test whether the cells within its
        * padding area exist in the hash table, if no, create an entry for that cell.
        * \param index the index of the cell in (x,y,z) format
        */
      void 
      fillPad (const Eigen::Vector3i &index);

      /** \brief Obtain the index of a cell and the pad size.
        * \param index the input index
        * \param pt_union_indices the union of input data points within the cell and padding cells
        */
      void 
      getDataPtsUnion (const Eigen::Vector3i &index, std::vector <int> &pt_union_indices);

      /** \brief Given the index of a cell, exam it's up, left, front edges, and add
        * the vectices to m_surface list.the up, left, front edges only share 4
        * points, we first get the vectors at these 4 points and exam whether those
        * three edges are intersected by the surface \param index the input index
        * \param pt_union_indices the union of input data points within the cell and padding cells
        */
      void 
      createSurfaceForCell (const Eigen::Vector3i &index, std::vector <int> &pt_union_indices);


      /** \brief Given the coordinates of one point, project it onto the surface, 
        * return the projected point. Do a binary search between p and p+projection_distance 
        * to find the projected point
        * \param p the coordinates of the input point
        * \param pt_union_indices the union of input data points within the cell and padding cells
        * \param projection the resultant point projected
        */
      void
      getProjection (const Eigen::Vector4f &p, std::vector<int> &pt_union_indices, Eigen::Vector4f &projection);

      /** \brief Given the coordinates of one point, project it onto the surface,
        * return the projected point. Find the plane which fits all the points in
        *  pt_union_indices, projected p to the plane to get the projected point.
        * \param p the coordinates of the input point
        * \param pt_union_indices the union of input data points within the cell and padding cells
        * \param projection the resultant point projected
        */
      void 
      getProjectionWithPlaneFit (const Eigen::Vector4f &p, 
                                 std::vector<int> &pt_union_indices, 
                                 Eigen::Vector4f &projection);


      /** \brief Given the location of a point, get it's vector
        * \param p the coordinates of the input point
        * \param pt_union_indices the union of input data points within the cell and padding cells
        * \param vo the resultant vector
        */
      void
      getVectorAtPoint (const Eigen::Vector4f &p, 
                        std::vector <int> &pt_union_indices, Eigen::Vector3f &vo);

      /** \brief Given the location of a point, get it's vector
        * \param p the coordinates of the input point
        * \param k_indices the k nearest neighbors of the query point
        * \param k_squared_distances the squared distances of the k nearest 
        * neighbors to the query point
        * \param vo the resultant vector
        */
      void
      getVectorAtPointKNN (const Eigen::Vector4f &p, 
                           std::vector<int> &k_indices, 
                           std::vector<float> &k_squared_distances,
                           Eigen::Vector3f &vo);

      /** \brief Get the magnitude of the vector by summing up the distance.
        * \param p the coordinate of the input point
        * \param pt_union_indices the union of input data points within the cell and padding cells
        */
      double 
      getMagAtPoint (const Eigen::Vector4f &p, const std::vector <int> &pt_union_indices);

      /** \brief Get the 1st derivative
        * \param p the coordinate of the input point
        * \param vec the vector at point p
        * \param pt_union_indices the union of input data points within the cell and padding cells
        */
      double 
      getD1AtPoint (const Eigen::Vector4f &p, const Eigen::Vector3f &vec, 
                    const std::vector <int> &pt_union_indices);

      /** \brief Get the 2nd derivative
        * \param p the coordinate of the input point
        * \param vec the vector at point p
        * \param pt_union_indices the union of input data points within the cell and padding cells
        */
      double 
      getD2AtPoint (const Eigen::Vector4f &p, const Eigen::Vector3f &vec, 
                    const std::vector <int> &pt_union_indices);

      /** \brief Test whether the edge is intersected by the surface by 
        * doing the dot product of the vector at two end points. Also test 
        * whether the edge is intersected by the maximum surface by examing 
        * the 2nd derivative of the intersection point 
        * \param end_pts the two points of the edge
        * \param vect_at_end_pts 
        * \param pt_union_indices the union of input data points within the cell and padding cells
        */
      bool 
      isIntersected (const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &end_pts, 
                     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &vect_at_end_pts, 
                     std::vector <int> &pt_union_indices);

      /** \brief Find point where the edge intersects the surface.
        * \param level binary search level
        * \param end_pts the two end points on the edge
        * \param vect_at_end_pts the vectors at the two end points
        * \param start_pt the starting point we use for binary search
        * \param pt_union_indices the union of input data points within the cell and padding cells
        * \param intersection the resultant intersection point
        */
      void
      findIntersection (int level, 
                        const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &end_pts, 
                        const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &vect_at_end_pts, 
                        const Eigen::Vector4f &start_pt, 
                        std::vector<int> &pt_union_indices,
                        Eigen::Vector4f &intersection);

      /** \brief Go through all the entries in the hash table and update the
       * cellData. 
       *
       * When creating the hash table, the pt_on_surface field store the center
       * point of the cell.After calling this function, the projection operator will
       * project the center point onto the surface, and the pt_on_surface field will
       * be updated using the projected point.Also the vect_at_grid_pt field will be
       * updated using the vector at the upper left front vertex of the cell.
       *
       * \param index_1d the index of the cell after flatting it's 3d index into a 1d array
       * \param index_3d the index of the cell in (x,y,z) 3d format
       * \param pt_union_indices the union of input data points within the cell and pads
       * \param cell_data information stored in the cell
       */
      void
      storeVectAndSurfacePoint (int index_1d, const Eigen::Vector3i &index_3d, 
                                std::vector<int> &pt_union_indices, const Leaf &cell_data);

      /** \brief Go through all the entries in the hash table and update the cellData. 
        * When creating the hash table, the pt_on_surface field store the center point
        * of the cell.After calling this function, the projection operator will project the 
        * center point onto the surface, and the pt_on_surface field will be updated 
        * using the projected point.Also the vect_at_grid_pt field will be updated using 
        * the vector at the upper left front vertex of the cell. When projecting the point 
        * and calculating the vector, using K nearest neighbors instead of using the 
        * union of input data point within the cell and pads.
        *
        * \param index_1d the index of the cell after flatting it's 3d index into a 1d array
        * \param index_3d the index of the cell in (x,y,z) 3d format
        * \param cell_data information stored in the cell
        */
      void 
      storeVectAndSurfacePointKNN (int index_1d, const Eigen::Vector3i &index_3d, const Leaf &cell_data);

    private:
      /** \brief Map containing the set of leaves. */
      HashMap cell_hash_map_;

      /** \brief Min and max data points. */
      Eigen::Vector4f min_p_, max_p_;

      /** \brief The size of a leaf. */
      double leaf_size_;

      /** \brief Gaussian scale. */
      double gaussian_scale_;

      /** \brief Data size. */
      int data_size_;

      /** \brief Max binary search level. */
      int max_binary_search_level_;

      /** \brief Number of neighbors (k) to use. */
      int k_;

      /** \brief Padding size. */
      int padding_size_;

      /** \brief The point cloud input (XYZ+Normals). */
      PointCloudPtr data_;

      /** \brief Store the surface normal(vector) at the each input data point. */
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vector_at_data_point_;
      
      /** \brief An array of points which lay on the output surface. */
      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > surface_;

      /** \brief Bit map which tells if there is any input data point in the cell. */
      boost::dynamic_bitset<> occupied_cell_list_;

      /** \brief Class get name method. */
      std::string getClassName () const { return ("GridProjection"); }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  // PCL_SURFACE_GRID_PROJECTION_H_
 
