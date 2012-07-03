/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2009-2012, Willow Garage, Inc.
 *  Copyright  (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: multi_grid_octree_data.hpp 5170 2012-03-18 04:21:56Z rusu $
 *
 */
#include <pcl/surface/poisson/octree_poisson.h>

#define ITERATION_POWER 1.0/3
#define MEMORY_ALLOCATOR_BLOCK_SIZE 1<<12

#define READ_SIZE 1024

#define PAD_SIZE  (Real (1.0))

namespace pcl
{
  namespace poisson
  {
    const Real EPSILON = Real (1e-6);
    const Real ROUND_EPS = Real (1e-5);

    //////////////////////////////////////////////////////////////////////////////////////////////
    SortedTreeNodes::SortedTreeNodes () : treeNodes (NULL), nodeCount (NULL), maxDepth (0)
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    SortedTreeNodes::~SortedTreeNodes ()
    {
      if (nodeCount)
        delete[] nodeCount;
      if (treeNodes)
        delete[] treeNodes;
      nodeCount = NULL;
      treeNodes = NULL;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void
    SortedTreeNodes::set (TreeOctNode& root, const int& setIndex)
    {
      if (nodeCount)
        delete[] nodeCount;
      if (treeNodes)
        delete[] treeNodes;
      maxDepth = root.maxDepth () + 1;
      nodeCount = new int[maxDepth + 1];
      treeNodes = new TreeOctNode*[root.nodes ()];

      TreeOctNode* temp = root.nextNode ();
      int i, cnt = 0;
      while (temp)
      {
        treeNodes[cnt++] = temp;
        temp = root.nextNode (temp);
      }
      qsort (treeNodes, cnt, sizeof (const TreeOctNode*), TreeOctNode::CompareForwardPointerDepths);
      for (i = 0; i <= maxDepth; i++)
        nodeCount[i] = 0;
      for (i = 0; i < cnt; i++)
      {
        if (setIndex)
          treeNodes[i]->nodeData.nodeIndex = i;
        nodeCount[treeNodes[i]->depth () + 1]++;
      }
      for (i = 1; i <= maxDepth; i++)
        nodeCount[i] += nodeCount[i - 1];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int TreeNodeData::UseIndex = 1;

    //////////////////////////////////////////////////////////////////////////////////////////////
    TreeNodeData::TreeNodeData () : value (0)
    {
      if (UseIndex)
      {
        nodeIndex = -1;
        centerWeightContribution = 0;
      }
      else
      {
        mcIndex = 0;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    TreeNodeData::~TreeNodeData ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> double Octree<Degree>::maxMemoryUsage = 0;

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> double 
    Octree<Degree>::MemoryUsage ()
    {
      double mem = 0.0; //MemoryInfo::Usage ()/ (1<<20);
      if (mem > maxMemoryUsage)
        maxMemoryUsage = mem;
      return (mem);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree>
    Octree<Degree>::Octree () : 
      neighborKey (), neighborKey2 (), 
      radius (0), width (0), normals (), postNormalSmooth (0), tree (), fData ()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::setNodeIndices (TreeOctNode& node, int& idx)
    {
      node.nodeData.nodeIndex = idx;
      idx++;
      if (node.children)
        for (int i = 0; i < Cube::CORNERS; i++)
          setNodeIndices (node.children[i], idx);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::NonLinearSplatOrientedPoint (TreeOctNode* node, const Point3D<Real>& position, const Point3D<Real>& normal)
    {
      double x, dxdy, dxdydz, dx[DIMENSION][3];
      int i, j, k;
      TreeOctNode::Neighbors& neighbors = neighborKey.setNeighbors (node);
      double width;
      Point3D<Real> center;
      Real w;

      node->centerAndWidth (center, w);
      width = w;
      for (int i = 0; i < 3; i++)
      {
        x = (center.coords[i] - position.coords[i] - width) / width;
        dx[i][0] = 1.125 + 1.500 * x + 0.500 * x*x;
        x = (center.coords[i] - position.coords[i]) / width;
        dx[i][1] = 0.750 - x*x;
        dx[i][2] = 1.0 - dx[i][1] - dx[i][0];
      }

      for (i = 0; i < 3; i++)
      {
        for (j = 0; j < 3; j++)
        {
          dxdy = dx[0][i] * dx[1][j];
          for (k = 0; k < 3; k++)
          {
            if (neighbors.neighbors[i][j][k])
            {
              dxdydz = dxdy * dx[2][k];
              int idx = neighbors.neighbors[i][j][k]->nodeData.nodeIndex;
              if (idx < 0)
              {
                Point3D<Real> n;
                n.coords[0] = n.coords[1] = n.coords[2] = 0;
                idx = neighbors.neighbors[i][j][k]->nodeData.nodeIndex = int (normals->size ());
                normals->push_back (n);
              }
              (*normals)[idx].coords[0] += Real (normal.coords[0] * dxdydz);
              (*normals)[idx].coords[1] += Real (normal.coords[1] * dxdydz);
              (*normals)[idx].coords[2] += Real (normal.coords[2] * dxdydz);
            }
          }
        }
      }
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::NonLinearSplatOrientedPoint (const Point3D<Real>& position,
                                                 const Point3D<Real>& normal,
                                                 const int& splatDepth,
                                                 const Real& samplesPerNode,
                                                 const int& minDepth,
                                                 const int& maxDepth)
    {
      double dx;
      Point3D<Real> n;
      TreeOctNode* temp;
      int i; //,cnt = 0;
      double width;
      Point3D<Real> myCenter;
      Real myWidth;
      myCenter.coords[0] = myCenter.coords[1] = myCenter.coords[2] = Real (0.5);
      myWidth = Real (1.0);

      temp = &tree;
      while (temp->depth () < splatDepth)
      {
        if (!temp->children)
        {
          PCL_ERROR ("Octree<Degree>::NonLinearSplatOrientedPoint error\n");
          return;
        }

        int cIndex = TreeOctNode::CornerIndex (myCenter, position);
        temp = &temp->children[cIndex];
        myWidth /= 2;
        if (cIndex & 1)
          myCenter.coords[0] += myWidth / 2;
        else
          myCenter.coords[0] -= myWidth / 2;

        if (cIndex & 2)
          myCenter.coords[1] += myWidth / 2;
        else
          myCenter.coords[1] -= myWidth / 2;

        if (cIndex & 4)
          myCenter.coords[2] += myWidth / 2;
        else
          myCenter.coords[2] -= myWidth / 2;
      }

      Real alpha, newDepth;
      NonLinearGetSampleDepthAndWeight (temp, position, samplesPerNode, newDepth, alpha);

      if (newDepth < minDepth)
        newDepth = Real (minDepth);
      if (newDepth > maxDepth)
        newDepth = Real (maxDepth);

      int topDepth = int (ceil (newDepth));

      //dx = static_cast<Real> (static_cast<Real> (1.0 - topDepth) - newDepth);
      dx = 1.0 - topDepth + newDepth;
      if (topDepth <= minDepth)
      {
        topDepth = minDepth;
        dx = 1;
      }
      else if (topDepth > maxDepth)
      {
        topDepth = maxDepth;
        dx = 1;
      }
      while (temp->depth () > topDepth)
        temp = temp->parent;
      while (temp->depth () < topDepth)
      {
        if (!temp->children)
          temp->initChildren ();
        int cIndex = TreeOctNode::CornerIndex (myCenter, position);
        temp = &temp->children[cIndex];
        myWidth /= 2;
        if (cIndex & 1)
          myCenter.coords[0] += myWidth / 2;
        else
          myCenter.coords[0] -= myWidth / 2;
        if (cIndex & 2)
          myCenter.coords[1] += myWidth / 2;
        else
          myCenter.coords[1] -= myWidth / 2;
        if (cIndex & 4)
          myCenter.coords[2] += myWidth / 2;
        else
          myCenter.coords[2] -= myWidth / 2;
      }
      width = 1.0 / (1 << temp->depth ());
      for (i = 0; i < DIMENSION; i++)
        n.coords[i] = normal.coords[i] * alpha / Real (pow (width, 3)) * Real (dx);
      NonLinearSplatOrientedPoint (temp, position, n);
      if (fabs (1.0 - dx) > EPSILON)
      {
        dx = Real (1.0 - dx);
        temp = temp->parent;
        width = 1.0 / (1 << temp->depth ());

        for (i = 0; i < DIMENSION; i++)
          n.coords[i] = normal.coords[i] * alpha / Real (pow (width, 3)) * Real (dx);
        NonLinearSplatOrientedPoint (temp, position, n);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::NonLinearGetSampleDepthAndWeight (TreeOctNode* node,
                                                      const Point3D<Real>& position,
                                                      const Real& samplesPerNode,
                                                      Real& depth,
                                                      Real& weight)
    {
      TreeOctNode* temp = node;
      weight = Real (1.0) / NonLinearGetSampleWeight (temp, position);
      if (weight >= samplesPerNode + 1)
        depth = Real (temp->depth () + log (weight / (samplesPerNode + 1)) / log (double (1 << (DIMENSION - 1))));
      else
      {
        Real oldAlpha, newAlpha;
        oldAlpha = newAlpha = weight;
        while (newAlpha < (samplesPerNode + 1) && temp->parent)
        {
          temp = temp->parent;
          oldAlpha = newAlpha;
          newAlpha = Real (1.0) / NonLinearGetSampleWeight (temp, position);
        }
        depth = Real (temp->depth () + log (newAlpha / (samplesPerNode + 1)) / log (newAlpha / oldAlpha));
      }
      weight = Real (pow (double (1 << (DIMENSION - 1)), -double (depth)));
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> Real 
    Octree<Degree>::NonLinearGetSampleWeight (TreeOctNode* node, const Point3D<Real>& position)
    {
      Real weight = 0;
      double x, dxdy, dx[DIMENSION][3];
      int i, j, k;
      TreeOctNode::Neighbors& neighbors = neighborKey.setNeighbors (node);
      double width;
      Point3D<Real> center;
      Real w;
      node->centerAndWidth (center, w);
      width = w;

      for (i = 0; i < DIMENSION; i++)
      {
        x = (center.coords[i] - position.coords[i] - width) / width;
        dx[i][0] = 1.125 + 1.500 * x + 0.500 * x*x;
        x = (center.coords[i] - position.coords[i]) / width;
        dx[i][1] = 0.750 - x*x;
        dx[i][2] = 1.0 - dx[i][1] - dx[i][0];
      }

      for (i = 0; i < 3; i++)
      {
        for (j = 0; j < 3; j++)
        {
          dxdy = dx[0][i] * dx[1][j];
          for (k = 0; k < 3; k++)
          {
            if (neighbors.neighbors[i][j][k])
              weight += Real (dxdy * dx[2][k] * neighbors.neighbors[i][j][k]->nodeData.centerWeightContribution);
          }
        }
      }
      return (Real (1.0 / weight));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::NonLinearUpdateWeightContribution (TreeOctNode* node,
                                                       const Point3D<Real>& position,
                                                       const Real& weight)
    {
      int i, j, k;
      TreeOctNode::Neighbors& neighbors = neighborKey.setNeighbors (node);
      double x, dxdy, dx[DIMENSION][3];
      double width;
      Point3D<Real> center;
      Real w;
      node->centerAndWidth (center, w);
      width = w;

      for (i = 0; i < DIMENSION; i++)
      {
        x = (center.coords[i] - position.coords[i] - width) / width;
        dx[i][0] = 1.125 + 1.500 * x + 0.500 * x*x;
        x = (center.coords[i] - position.coords[i]) / width;
        dx[i][1] = 0.750 - x*x;
        dx[i][2] = 1.0 - dx[i][1] - dx[i][0];
      }

      for (i = 0; i < 3; i++)
      {
        for (j = 0; j < 3; j++)
        {
          dxdy = dx[0][i] * dx[1][j] * weight;
          for (k = 0; k < 3; k++)
            if (neighbors.neighbors[i][j][k])
              neighbors.neighbors[i][j][k]->nodeData.centerWeightContribution += Real (dxdy * dx[2][k]);
        }
      }
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> template<typename PointNT> int 
    Octree<Degree>::setTree (boost::shared_ptr<const pcl::PointCloud<PointNT> > input_,
                             const int& maxDepth,
                             const int& kernelDepth,
                             const Real& samplesPerNode,
                             const Real& scaleFactor,
                             Point3D<Real>& center,
                             Real& scale,
                             const int& resetSamples,
                             const int& useConfidence)
    {
      Point3D<Real> min, max, position, normal, myCenter;
      Real myWidth;
      size_t i, cnt = 0;
      TreeOctNode* temp;
      int splatDepth = 0;
      float c[2 * DIMENSION];

      TreeNodeData::UseIndex = 1;
      neighborKey.set (maxDepth);
      splatDepth = kernelDepth;
      if (splatDepth < 0)
        splatDepth = 0;

      // Read through once to get the center and scale
      while (1)
      {
        if (cnt == input_->size ())
          break;
        c[0] = input_->points[cnt].x;
        c[1] = input_->points[cnt].y;
        c[2] = input_->points[cnt].z;
        c[3] = input_->points[cnt].normal_x;
        c[4] = input_->points[cnt].normal_y;
        c[5] = input_->points[cnt].normal_z;

        for (i = 0; i < DIMENSION; i++)
        {
          if (!cnt || c[i] < min.coords[i])
            min.coords[i] = c[i];
          if (!cnt || c[i] > max.coords[i])
            max.coords[i] = c[i];
        }
        cnt++;
      }

      for (i = 0; i < DIMENSION; i++)
      {
        if (!i || scale < max.coords[i] - min.coords[i])
          scale = Real (max.coords[i] - min.coords[i]);
        center.coords[i] = Real (max.coords[i] + min.coords[i]) / 2;
      }

      scale *= scaleFactor;
      for (i = 0; i < DIMENSION; i++)
        center.coords[i] -= scale / 2;
      if (splatDepth > 0)
      {
        cnt = 0;
        while (1)
        {
          if (cnt == input_->size ())
            break;
          c[0] = input_->points[cnt].x;
          c[1] = input_->points[cnt].y;
          c[2] = input_->points[cnt].z;
          c[3] = input_->points[cnt].normal_x;
          c[4] = input_->points[cnt].normal_y;
          c[5] = input_->points[cnt].normal_z;

          for (i = 0; i < DIMENSION; i++)
          {
            position.coords[i] = (c[i] - center.coords[i]) / scale;
            normal.coords[i] = c[DIMENSION + i];
          }
          myCenter.coords[0] = myCenter.coords[1] = myCenter.coords[2] = Real (0.5);
          myWidth = Real (1.0);
          for (i = 0; i < DIMENSION; i++)
            if (position.coords[i] < myCenter.coords[i] - myWidth / 2 || position.coords[i] > myCenter.coords[i] + myWidth / 2)
              break;
          if (i != DIMENSION)
            continue;

          temp = &tree;
          int d = 0;
          Real weight = Real (1.0);
          if (useConfidence)
            weight = Real (Length (normal));

          while (d < splatDepth)
          {
            NonLinearUpdateWeightContribution (temp, position, weight);
            if (!temp->children)
              temp->initChildren ();
            int cIndex = TreeOctNode::CornerIndex (myCenter, position);
            temp = &temp->children[cIndex];
            myWidth /= 2;
            if (cIndex & 1)
              myCenter.coords[0] += myWidth / 2;
            else
              myCenter.coords[0] -= myWidth / 2;
            if (cIndex & 2)
              myCenter.coords[1] += myWidth / 2;
            else
              myCenter.coords[1] -= myWidth / 2;
            if (cIndex & 4)
              myCenter.coords[2] += myWidth / 2;
            else
              myCenter.coords[2] -= myWidth / 2;
            d++;
          }
          NonLinearUpdateWeightContribution (temp, position, weight);
          cnt++;
        }
      }

      normals = new std::vector<Point3D<Real> > ();
      cnt = 0;
      while (1)
      {
        if (cnt == input_->size ())
          break;
        c[0] = input_->points[cnt].x;
        c[1] = input_->points[cnt].y;
        c[2] = input_->points[cnt].z;
        c[3] = input_->points[cnt].normal_x;
        c[4] = input_->points[cnt].normal_y;
        c[5] = input_->points[cnt].normal_z;
        cnt++;

        for (i = 0; i < DIMENSION; i++)
        {
          position.coords[i] = (c[i] - center.coords[i]) / scale;
          normal.coords[i] = c[DIMENSION + i];
        }

        myCenter.coords[0] = myCenter.coords[1] = myCenter.coords[2] = Real (0.5);
        myWidth = Real (1.0);
        for (i = 0; i < DIMENSION; i++)
          if (position.coords[i] < myCenter.coords[i] - myWidth / 2 || position.coords[i] > myCenter.coords[i] + myWidth / 2)
            break;
        if (i != DIMENSION)
          continue;
        Real l = Real (Length (normal));
        if (l != l || l < EPSILON)
          continue;
        if (!useConfidence)
        {
          normal.coords[0] /= l;
          normal.coords[1] /= l;
          normal.coords[2] /= l;
        }
        l = Real (2 << maxDepth);
        normal.coords[0] *= l;
        normal.coords[1] *= l;
        normal.coords[2] *= l;

        if (resetSamples && samplesPerNode > 0 && splatDepth)
          NonLinearSplatOrientedPoint (position, normal, splatDepth, samplesPerNode, 1, maxDepth);
        else
        {
          Real alpha = 1;
          temp = &tree;
          int d = 0;
          if (splatDepth)
          {
            while (d < splatDepth)
            {
              int cIndex = TreeOctNode::CornerIndex (myCenter, position);
              temp = &temp->children[cIndex];
              myWidth /= 2;
              if (cIndex & 1)
                myCenter.coords[0] += myWidth / 2;
              else
                myCenter.coords[0] -= myWidth / 2;
              if (cIndex & 2)
                myCenter.coords[1] += myWidth / 2;
              else
                myCenter.coords[1] -= myWidth / 2;
              if (cIndex & 4)
                myCenter.coords[2] += myWidth / 2;
              else
                myCenter.coords[2] -= myWidth / 2;
              d++;
            }
            alpha = NonLinearGetSampleWeight (temp, position);
          }
          for (i = 0; i < DIMENSION; i++)
            normal.coords[i] *= alpha;
          while (d < maxDepth)
          {
            if (!temp->children)
              temp->initChildren ();
            int cIndex = TreeOctNode::CornerIndex (myCenter, position);
            temp = &temp->children[cIndex];
            myWidth /= 2;
            if (cIndex & 1)
              myCenter.coords[0] += myWidth / 2;
            else
              myCenter.coords[0] -= myWidth / 2;
            if (cIndex & 2)
              myCenter.coords[1] += myWidth / 2;
            else
              myCenter.coords[1] -= myWidth / 2;
            if (cIndex & 4)
              myCenter.coords[2] += myWidth / 2;
            else
              myCenter.coords[2] -= myWidth / 2;
            d++;
          }
          NonLinearSplatOrientedPoint (temp, position, normal);
        }
      }
      return (static_cast<int> (cnt));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::setFunctionData (const PPolynomial<Degree>& ReconstructionFunction,
                                     const int& maxDepth, const int& normalize,
                                     const Real& normalSmooth)
    {
      radius = Real (fabs (ReconstructionFunction.polys[0].start));
      width = int (double (radius + 0.5 - EPSILON)*2);
      if (normalSmooth > 0)
        postNormalSmooth = normalSmooth;
      fData.set (maxDepth, ReconstructionFunction, normalize, 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::finalize1 (const int& refineNeighbors)
    {
      TreeOctNode* temp;

      if (refineNeighbors >= 0)
      {
        RefineFunction rf;
        temp = tree.nextNode ();
        while (temp)
        {
          if (temp->nodeData.nodeIndex >= 0 && Length ((*normals)[temp->nodeData.nodeIndex]) > EPSILON)
          {
            rf.depth = temp->depth () - refineNeighbors;
            TreeOctNode::ProcessMaxDepthNodeAdjacentNodes (fData.depth, temp, 2 * width, &tree, 1, temp->depth () - refineNeighbors, &rf);
          }
          temp = tree.nextNode (temp);
        }
      }
      else if (refineNeighbors == -1234)
      {
        temp = tree.nextLeaf ();
        while (temp)
        {
          if (!temp->children && temp->depth () < fData.depth)
            temp->initChildren ();
          temp = tree.nextLeaf (temp);
        }
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::finalize2 (const int& refineNeighbors)
    {
      TreeOctNode* temp;

      if (refineNeighbors >= 0)
      {
        RefineFunction rf;
        temp = tree.nextNode ();
        while (temp)
        {
          if (fabs (temp->nodeData.value) > EPSILON)
          {
            rf.depth = temp->depth () - refineNeighbors;
            TreeOctNode::ProcessMaxDepthNodeAdjacentNodes (fData.depth, temp, 2 * width, &tree, 1, temp->depth () - refineNeighbors, &rf);
          }
          temp = tree.nextNode (temp);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template <int Degree> Real 
    Octree<Degree>::GetDivergence (const int idx[DIMENSION], const Point3D<Real>& normal) const
    {
      double dot = fData.dotTable[idx[0]] * fData.dotTable[idx[1]] * fData.dotTable[idx[2]];
      return Real (dot * (fData.dDotTable[idx[0]] * normal.coords[0] + fData.dDotTable[idx[1]] * normal.coords[1] + fData.dDotTable[idx[2]] * normal.coords[2]));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> Real 
    Octree<Degree>::GetLaplacian (const int idx[DIMENSION]) const
    {
      return Real (fData.dotTable[idx[0]] * fData.dotTable[idx[1]] * fData.dotTable[idx[2]]* (fData.d2DotTable[idx[0]] + fData.d2DotTable[idx[1]] + fData.d2DotTable[idx[2]]));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> Real 
    Octree<Degree>::GetDotProduct (const int idx[DIMENSION]) const
    {
      return Real (fData.dotTable[idx[0]] * fData.dotTable[idx[1]] * fData.dotTable[idx[2]]);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetFixedDepthLaplacian (SparseSymmetricMatrix<float>& matrix,
                                            const int& depth,
                                            const SortedTreeNodes& sNodes)
    {
      LaplacianMatrixFunction mf;
      mf.ot = this;
      mf.offset = sNodes.nodeCount[depth];
      matrix.Resize (sNodes.nodeCount[depth + 1] - sNodes.nodeCount[depth]);
      mf.rowElements = reinterpret_cast<MatrixEntry<float>*> (malloc (sizeof (MatrixEntry<float>) * matrix.rows));
      for (int i = sNodes.nodeCount[depth]; i < sNodes.nodeCount[depth + 1]; i++)
      {
        mf.elementCount = 0;
        mf.d2 = int (sNodes.treeNodes[i]->d);
        mf.x2 = int (sNodes.treeNodes[i]->off[0]);
        mf.y2 = int (sNodes.treeNodes[i]->off[1]);
        mf.z2 = int (sNodes.treeNodes[i]->off[2]);
        mf.index[0] = mf.x2;
        mf.index[1] = mf.y2;
        mf.index[2] = mf.z2;
        TreeOctNode::ProcessTerminatingNodeAdjacentNodes (fData.depth, sNodes.treeNodes[i], 2 * width - 1, &tree, 1, &mf);
        matrix.SetRowSize (i - sNodes.nodeCount[depth], mf.elementCount);
        memcpy (matrix.m_ppElements[i - sNodes.nodeCount[depth]], mf.rowElements, sizeof (MatrixEntry<float>) * mf.elementCount);
      }
      free (mf.rowElements);
      return 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRestrictedFixedDepthLaplacian (SparseSymmetricMatrix<float>& matrix,
                                                      const int& /*depth*/,
                                                      const int* entries,
                                                      const int& entryCount,
                                                      const TreeOctNode* rNode,
                                                      const Real& radius,
                                                      const SortedTreeNodes& sNodes)
    {
      int i;
      RestrictedLaplacianMatrixFunction mf;
      //Real myRadius = int (2*radius-ROUND_EPS)+ROUND_EPS;
      mf.ot = this;
      mf.radius = radius;
      rNode->depthAndOffset (mf.depth, mf.offset);
      matrix.Resize (entryCount);
      mf.rowElements = reinterpret_cast<MatrixEntry<float>*> (malloc (sizeof (MatrixEntry<float>) * matrix.rows));

      for (i = 0; i < entryCount; i++)
        sNodes.treeNodes[entries[i]]->nodeData.nodeIndex = i;
      for (i = 0; i < entryCount; i++)
      {
        mf.elementCount = 0;
        mf.index[0] = int (sNodes.treeNodes[entries[i]]->off[0]);
        mf.index[1] = int (sNodes.treeNodes[entries[i]]->off[1]);
        mf.index[2] = int (sNodes.treeNodes[entries[i]]->off[2]);
        TreeOctNode::ProcessTerminatingNodeAdjacentNodes (fData.depth, sNodes.treeNodes[entries[i]], 2 * width - 1, &tree, 1, &mf);
        matrix.SetRowSize (i, mf.elementCount);
        memcpy (matrix.m_ppElements[i], mf.rowElements, sizeof (MatrixEntry<float>) * mf.elementCount);
      }
      for (i = 0; i < entryCount; i++)
        sNodes.treeNodes[entries[i]]->nodeData.nodeIndex = entries[i];
      free (mf.rowElements);
      return 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::LaplacianMatrixIteration (const int& subdivideDepth)
    {
      int i, iter = 0;
      SortedTreeNodes sNodes;
      //double t;
      fData.setDotTables (fData.D2_DOT_FLAG);
      sNodes.set (tree, 1);

      SparseMatrix<float>::SetAllocator (MEMORY_ALLOCATOR_BLOCK_SIZE);

      sNodes.treeNodes[0]->nodeData.value = 0;
      for (i = 1; i < sNodes.maxDepth; i++)
      {
        if (subdivideDepth > 0)
          iter += SolveFixedDepthMatrix (i, subdivideDepth, sNodes);
        else
          iter += SolveFixedDepthMatrix (i, sNodes);
      }

      SparseMatrix<float>::AllocatorMatrixEntry.reset ();
      fData.clearDotTables (fData.DOT_FLAG | fData.D_DOT_FLAG | fData.D2_DOT_FLAG);
      return iter;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::SolveFixedDepthMatrix (const int& depth, const SortedTreeNodes& sNodes)
    {
      int i, iter = 0;
      Vector<double> V, Solution;
      SparseSymmetricMatrix<Real> matrix;
      Real myRadius;
      Real dx, dy, dz;
      int x1, x2, y1, y2, z1, z2;
      Vector<Real> Diagonal;

      V.Resize (sNodes.nodeCount[depth + 1] - sNodes.nodeCount[depth]);
      for (i = sNodes.nodeCount[depth]; i < sNodes.nodeCount[depth + 1]; i++)
        V[i - sNodes.nodeCount[depth]] = sNodes.treeNodes[i]->nodeData.value;
      SparseSymmetricMatrix<float>::AllocatorMatrixEntry.rollBack ();
      GetFixedDepthLaplacian (matrix, depth, sNodes);
      iter += SparseSymmetricMatrix<Real>::Solve (matrix, V, int (pow (matrix.rows, ITERATION_POWER)), Solution, double (EPSILON), 1);

      for (i = sNodes.nodeCount[depth]; i < sNodes.nodeCount[depth + 1]; i++)
        sNodes.treeNodes[i]->nodeData.value = Real (Solution[i - sNodes.nodeCount[depth]]);

      myRadius = Real (radius + ROUND_EPS - 0.5);
      myRadius /= static_cast<Real> (1 << depth);

      if (depth < sNodes.maxDepth - 1)
      {
        LaplacianProjectionFunction pf;
        TreeOctNode *node1, *node2;
        pf.ot = this;
        int idx1, idx2, off = sNodes.nodeCount[depth];
        // First pass: idx2 is the solution coefficient propogated
        for (i = 0; i < matrix.rows; i++)
        {
          idx1 = i;
          node1 = sNodes.treeNodes[idx1 + off];
          if (!node1->children)
            continue;
          x1 = int (node1->off[0]);
          y1 = int (node1->off[1]);
          z1 = int (node1->off[2]);
          for (int j = 0; j < matrix.rowSizes[i]; j++)
          {
            idx2 = matrix.m_ppElements[i][j].N;
            node2 = sNodes.treeNodes[idx2 + off];
            x2 = int (node2->off[0]);
            y2 = int (node2->off[1]);
            z2 = int (node2->off[2]);
            pf.value = Solution[idx2];
            pf.index[0] = x2;
            pf.index[1] = y2;
            pf.index[2] = z2;
            dx = Real (x2 - x1) / static_cast<Real> (1 << depth);
            dy = Real (y2 - y1) / static_cast<Real> (1 << depth);
            dz = Real (z2 - z1) / static_cast<Real> (1 << depth);
            if (fabs (dx) < myRadius && fabs (dy) < myRadius && fabs (dz) < myRadius)
              node1->processNodeNodes (node2, &pf, 0);
            else
              TreeOctNode::ProcessNodeAdjacentNodes (fData.depth, node2, width, node1, width, &pf, 0);
          }
        }
        // Second pass: idx1 is the solution coefficient propogated
        for (i = 0; i < matrix.rows; i++)
        {
          idx1 = i;
          node1 = sNodes.treeNodes[idx1 + off];
          x1 = int (node1->off[0]);
          y1 = int (node1->off[1]);
          z1 = int (node1->off[2]);
          pf.value = Solution[idx1];
          pf.index[0] = x1;
          pf.index[1] = y1;
          pf.index[2] = z1;
          for (int j = 0; j < matrix.rowSizes[i]; j++)
          {
            idx2 = matrix.m_ppElements[i][j].N;
            node2 = sNodes.treeNodes[idx2 + off];
            if (idx1 != idx2 && node2->children)
            {
              x2 = int (node2->off[0]);
              y2 = int (node2->off[1]);
              z2 = int (node2->off[2]);
              dx = Real (x1 - x2) / static_cast<Real> (1 << depth);
              dy = Real (y1 - y2) / static_cast<Real> (1 << depth);
              dz = Real (z1 - z2) / static_cast<Real> (1 << depth);
              if (fabs (dx) < myRadius && fabs (dy) < myRadius && fabs (dz) < myRadius)
                node2->processNodeNodes (node1, &pf, 0);
              else
                TreeOctNode::ProcessNodeAdjacentNodes (fData.depth, node1, width, node2, width, &pf, 0);
            }
          }
        }
      }
      return iter;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::SolveFixedDepthMatrix (const int& depth,
                                           const int& startingDepth,
                                           const SortedTreeNodes& sNodes)
    {
      int i, j, d, iter = 0;
      SparseSymmetricMatrix<Real> matrix;
      AdjacencySetFunction asf;
      AdjacencyCountFunction acf;
      Vector<Real> Values;
      Vector<double> SubValues, SubSolution;
      Real myRadius, myRadius2;
      Real dx, dy, dz;
      Vector<Real> Diagonal;

      if (startingDepth >= depth)
        return SolveFixedDepthMatrix (depth, sNodes);

      Values.Resize (sNodes.nodeCount[depth + 1] - sNodes.nodeCount[depth]);

      for (i = sNodes.nodeCount[depth]; i < sNodes.nodeCount[depth + 1]; i++)
      {
        Values[i - sNodes.nodeCount[depth]] = sNodes.treeNodes[i]->nodeData.value;
        sNodes.treeNodes[i]->nodeData.value = 0;
      }

      myRadius = 2 * radius - Real (0.5);
      myRadius = static_cast<Real> (int (myRadius - ROUND_EPS)) + ROUND_EPS;
      myRadius2 = Real (radius + ROUND_EPS - 0.5);
      d = depth - startingDepth;
      for (i = sNodes.nodeCount[d]; i < sNodes.nodeCount[d + 1]; i++)
      {
        TreeOctNode* temp;
        // Get all of the entries associated to the subspace
        acf.adjacencyCount = 0;
        temp = sNodes.treeNodes[i]->nextNode ();
        while (temp)
        {
          if (temp->depth () == depth)
          {
            acf.Function (temp, temp);
            temp = sNodes.treeNodes[i]->nextBranch (temp);
          }
          else
            temp = sNodes.treeNodes[i]->nextNode (temp);
        }
        for (j = sNodes.nodeCount[d]; j < sNodes.nodeCount[d + 1]; j++)
        {
          if (i == j)
            continue;
          TreeOctNode::ProcessFixedDepthNodeAdjacentNodes (fData.depth, sNodes.treeNodes[i], 1, sNodes.treeNodes[j], 2 * width - 1, depth, &acf);
        }

        if (!acf.adjacencyCount)
          continue;
        asf.adjacencies = new int[acf.adjacencyCount];
        asf.adjacencyCount = 0;
        temp = sNodes.treeNodes[i]->nextNode ();
        while (temp
               )
        {
          if (temp->depth () == depth)
          {
            asf.Function (temp, temp);
            temp = sNodes.treeNodes[i]->nextBranch (temp);
          }
          else
            temp = sNodes.treeNodes[i]->nextNode (temp);
        }
        for (j = sNodes.nodeCount[d]; j < sNodes.nodeCount[d + 1]; j++)
        {
          if (i == j)
            continue;
          TreeOctNode::ProcessFixedDepthNodeAdjacentNodes (fData.depth, sNodes.treeNodes[i], 1, sNodes.treeNodes[j], 2 * width - 1, depth, &asf);
        }

        // Get the associated vector
        SubValues.Resize (asf.adjacencyCount);
        for (j = 0; j < asf.adjacencyCount; j++)
          SubValues[j] = Values[asf.adjacencies[j] - sNodes.nodeCount[depth]];
        SubSolution.Resize (asf.adjacencyCount);
        for (j = 0; j < asf.adjacencyCount; j++)
          SubSolution[j] = sNodes.treeNodes[asf.adjacencies[j]]->nodeData.value;
        // Get the associated matrix
        SparseSymmetricMatrix<float>::AllocatorMatrixEntry.rollBack ();
        GetRestrictedFixedDepthLaplacian (matrix, depth, asf.adjacencies, asf.adjacencyCount, sNodes.treeNodes[i], myRadius, sNodes);

        // Solve the matrix
        iter += SparseSymmetricMatrix<Real>::Solve (matrix, SubValues, int (pow (matrix.rows, ITERATION_POWER)), SubSolution, double (EPSILON), 0);

        LaplacianProjectionFunction lpf;
        lpf.ot = this;

        // Update the solution for all nodes in the sub-tree
        for (j = 0; j < asf.adjacencyCount; j++)
        {
          temp = sNodes.treeNodes[asf.adjacencies[j]];
          while (temp->depth () > sNodes.treeNodes[i]->depth ())
            temp = temp->parent;
          if (temp->nodeData.nodeIndex >= sNodes.treeNodes[i]->nodeData.nodeIndex)
            sNodes.treeNodes[asf.adjacencies[j]]->nodeData.value = Real (SubSolution[j]);
        }
        // Update the values in the next depth
        int x1, x2, y1, y2, z1, z2;
        if (depth < sNodes.maxDepth - 1)
        {
          int idx1, idx2;
          TreeOctNode *node1, *node2;
          // First pass: idx2 is the solution coefficient propogated
          for (j = 0; j < matrix.rows; j++)
          {
            idx1 = asf.adjacencies[j];
            node1 = sNodes.treeNodes[idx1];
            if (!node1->children)
              continue;

            x1 = int (node1->off[0]);
            y1 = int (node1->off[1]);
            z1 = int (node1->off[2]);

            for (int k = 0; k < matrix.rowSizes[j]; k++)
            {
              idx2 = asf.adjacencies[matrix.m_ppElements[j][k].N];
              node2 = sNodes.treeNodes[idx2];
              temp = node2;
              while (temp->depth () > d)
                temp = temp->parent;
              if (temp != sNodes.treeNodes[i])
                continue;
              lpf.value = Real (SubSolution[matrix.m_ppElements[j][k].N]);
              x2 = int (node2->off[0]);
              y2 = int (node2->off[1]);
              z2 = int (node2->off[2]);
              lpf.index[0] = x2;
              lpf.index[1] = y2;
              lpf.index[2] = z2;
              dx = Real (x2 - x1) / static_cast<Real> (1 << depth);
              dy = Real (y2 - y1) / static_cast<Real> (1 << depth);
              dz = Real (z2 - z1) / static_cast<Real> (1 << depth);
              if (fabs (dx) < myRadius2 && fabs (dy) < myRadius2 && fabs (dz) < myRadius2)
                node1->processNodeNodes (node2, &lpf, 0);
              else
                TreeOctNode::ProcessNodeAdjacentNodes (fData.depth, node2, width, node1, width, &lpf, 0);
            }
          }
          // Second pass: idx1 is the solution coefficient propogated
          for (j = 0; j < matrix.rows; j++)
          {
            idx1 = asf.adjacencies[j];
            node1 = sNodes.treeNodes[idx1];
            temp = node1;
            while (temp->depth () > d)
              temp = temp->parent;
            if (temp != sNodes.treeNodes[i])
              continue;
            x1 = int (node1->off[0]);
            y1 = int (node1->off[1]);
            z1 = int (node1->off[2]);

            lpf.value = Real (SubSolution[j]);
            lpf.index[0] = x1;
            lpf.index[1] = y1;
            lpf.index[2] = z1;
            for (int k = 0; k < matrix.rowSizes[j]; k++)
            {
              idx2 = asf.adjacencies[matrix.m_ppElements[j][k].N];
              node2 = sNodes.treeNodes[idx2];
              if (!node2->children)
                continue;

              if (idx1 != idx2)
              {
                x2 = int (node2->off[0]);
                y2 = int (node2->off[1]);
                z2 = int (node2->off[2]);
                dx = Real (x1 - x2) / static_cast<Real> (1 << depth);
                dy = Real (y1 - y2) / static_cast<Real> (1 << depth);
                dz = Real (z1 - z2) / static_cast<Real> (1 << depth);
                if (fabs (dx) < myRadius2 && fabs (dy) < myRadius2 && fabs (dz) < myRadius2)
                  node2->processNodeNodes (node1, &lpf, 0);
                else
                  TreeOctNode::ProcessNodeAdjacentNodes (fData.depth, node1, width, node2, width, &lpf, 0);
              }
            }
          }
        }
        delete[] asf.adjacencies;
      }
      return iter;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::HasNormals (TreeOctNode* node, const Real& epsilon)
    {
      int hasNormals = 0;
      if (node->nodeData.nodeIndex >= 0 && Length ((*normals)[node->nodeData.nodeIndex]) > epsilon)
        hasNormals = 1;
      if (node->children)
        for (int i = 0; i < Cube::CORNERS && !hasNormals; i++)
          hasNormals |= HasNormals (&node->children[i], epsilon);

      return hasNormals;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::ClipTree ()
    {
      TreeOctNode* temp;
      temp = tree.nextNode ();
      while (temp)
      {
        if (temp->children)
        {
          int hasNormals = 0;
          for (int i = 0; i < Cube::CORNERS && !hasNormals; i++)
            hasNormals = HasNormals (&temp->children[i], EPSILON);
          if (!hasNormals)
            temp->children = NULL;
        }
        temp = tree.nextNode (temp);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::SetLaplacianWeights ()
    {
      TreeOctNode* temp;

      fData.setDotTables (fData.DOT_FLAG | fData.D_DOT_FLAG);
      DivergenceFunction df;
      df.ot = this;
      temp = tree.nextNode ();
      while (temp)
      {
        if (temp->nodeData.nodeIndex < 0 || Length ((*normals)[temp->nodeData.nodeIndex]) <= EPSILON)
        {
          temp = tree.nextNode (temp);
          continue;
        }
        //int d = temp->depth ();
        df.normal = (*normals)[temp->nodeData.nodeIndex];
        df.index[0] = int (temp->off[0]);
        df.index[1] = int (temp->off[1]);
        df.index[2] = int (temp->off[2]);
        TreeOctNode::ProcessNodeAdjacentNodes (fData.depth, temp, width, &tree, width, &df);
        temp = tree.nextNode (temp);
      }
      fData.clearDotTables (fData.D_DOT_FLAG);
      temp = tree.nextNode ();
      while (temp)
      {
        if (temp->nodeData.nodeIndex < 0)
          temp->nodeData.centerWeightContribution = 0;
        else
          temp->nodeData.centerWeightContribution = Real (Length ((*normals)[temp->nodeData.nodeIndex]));
        temp = tree.nextNode (temp);
      }

      delete normals;
      normals = NULL;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::DivergenceFunction::Function (TreeOctNode* node1, const TreeOctNode* /*node2*/)
    {
      Point3D<Real> n = normal;
      if (FunctionData<Degree, Real>::SymmetricIndex (index[0], int (node1->off[0]), scratch[0]))
        n.coords[0] = -n.coords[0];
      if (FunctionData<Degree, Real>::SymmetricIndex (index[1], int (node1->off[1]), scratch[1]))
        n.coords[1] = -n.coords[1];
      if (FunctionData<Degree, Real>::SymmetricIndex (index[2], int (node1->off[2]), scratch[2]))
        n.coords[2] = -n.coords[2];
      double dot = ot->fData.dotTable[scratch[0]] * ot->fData.dotTable[scratch[1]] * ot->fData.dotTable[scratch[2]];
      node1->nodeData.value += Real (dot * (ot->fData.dDotTable[scratch[0]] * n.coords[0] + ot->fData.dDotTable[scratch[1]] * n.coords[1] + ot->fData.dDotTable[scratch[2]] * n.coords[2]));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::LaplacianProjectionFunction::Function (TreeOctNode* node1, const TreeOctNode* /*node2*/)
    {
      scratch[0] = FunctionData<Degree, Real>::SymmetricIndex (index[0], int (node1->off[0]));
      scratch[1] = FunctionData<Degree, Real>::SymmetricIndex (index[1], int (node1->off[1]));
      scratch[2] = FunctionData<Degree, Real>::SymmetricIndex (index[2], int (node1->off[2]));
      node1->nodeData.value -= Real (ot->GetLaplacian (scratch) * value);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::AdjacencyCountFunction::Function (const TreeOctNode* /*node1*/, const TreeOctNode* /*node2*/)
    {
      adjacencyCount++;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::AdjacencySetFunction::Function (const TreeOctNode* node1, const TreeOctNode* /*node2*/)
    {
      adjacencies[adjacencyCount++] = node1->nodeData.nodeIndex;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::RefineFunction::Function (TreeOctNode* node1, const TreeOctNode* /*node2*/)
    {
      if (!node1->children && node1->depth () < depth)
        node1->initChildren ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::FaceEdgesFunction::Function (const TreeOctNode* node1, const TreeOctNode* /*node2*/)
    {
      if (!node1->children && MarchingCubes::HasRoots (node1->nodeData.mcIndex))
      {
        RootInfo ri1, ri2;
        hash_map<long long, std::pair<RootInfo, int> >::iterator iter;
        int isoTri[DIMENSION * MarchingCubes::MAX_TRIANGLES];
        int count = MarchingCubes::AddTriangleIndices (node1->nodeData.mcIndex, isoTri);

        for (int j = 0; j < count; j++)
        {
          for (int k = 0; k < 3; k++)
          {
            if (fIndex == Cube::FaceAdjacentToEdges (isoTri[j * 3 + k], isoTri[j * 3 + ((k + 1) % 3)]))
            {
              if (GetRootIndex (node1, isoTri[j * 3 + k], maxDepth, ri1) && GetRootIndex (node1, isoTri[j * 3 + ((k + 1) % 3)], maxDepth, ri2))
              {
                edges->push_back (std::pair<long long, long long> (ri2.key, ri1.key));
                iter = vertexCount->find (ri1.key);
                if (iter == vertexCount->end ())
                {
                  (*vertexCount)[ri1.key].first = ri1;
                  (*vertexCount)[ri1.key].second = 0;
                }
                iter = vertexCount->find (ri2.key);
                if (iter == vertexCount->end ())
                {
                  (*vertexCount)[ri2.key].first = ri2;
                  (*vertexCount)[ri2.key].second = 0;
                }
                (*vertexCount)[ri1.key].second--;
                (*vertexCount)[ri2.key].second++;
              }
              else
                PCL_ERROR ("Bad Edge 1: %d %d\n", ri1.key, ri2.key);
            }
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::PointIndexValueFunction::Function (const TreeOctNode* node)
    {
      int idx[DIMENSION];
      idx[0] = index[0] + int (node->off[0]);
      idx[1] = index[1] + int (node->off[1]);
      idx[2] = index[2] + int (node->off[2]);
      value += node->nodeData.value * Real (valueTables[idx[0]] * valueTables[idx[1]] * valueTables[idx[2]]);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::PointIndexValueAndNormalFunction::Function (const TreeOctNode* node)
    {
      int idx[DIMENSION];
      idx[0] = index[0] + int (node->off[0]);
      idx[1] = index[1] + int (node->off[1]);
      idx[2] = index[2] + int (node->off[2]);
      value += node->nodeData.value * Real (valueTables[idx[0]] * valueTables[idx[1]] * valueTables[idx[2]]);
      normal.coords[0] += node->nodeData.value * Real (dValueTables[idx[0]] * valueTables[idx[1]] * valueTables[idx[2]]);
      normal.coords[1] += node->nodeData.value * Real (valueTables[idx[0]] * dValueTables[idx[1]] * valueTables[idx[2]]);
      normal.coords[2] += node->nodeData.value * Real (valueTables[idx[0]] * valueTables[idx[1]] * dValueTables[idx[2]]);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::LaplacianMatrixFunction::Function (const TreeOctNode* node1, const TreeOctNode* node2)
    {
      Real temp;
      int d1 = int (node1->d);
      int x1, y1, z1;
      x1 = int (node1->off[0]);
      y1 = int (node1->off[1]);
      z1 = int (node1->off[2]);
      int dDepth = d2 - d1;
      int d;
      d = (x2 >> dDepth) - x1;
      if (d < 0)
        return 0;
      if (!dDepth)
      {
        if (!d)
        {
          d = y2 - y1;
          if (d < 0)
            return 0;
          else if (!d)
          {
            d = z2 - z1;
            if (d < 0)
              return 0;
          }
        }
        scratch[0] = FunctionData<Degree, Real>::SymmetricIndex (index[0], x1);
        scratch[1] = FunctionData<Degree, Real>::SymmetricIndex (index[1], y1);
        scratch[2] = FunctionData<Degree, Real>::SymmetricIndex (index[2], z1);
        temp = ot->GetLaplacian (scratch);
        if (node1 == node2)
          temp /= 2;
        if (fabs (temp) > EPSILON)
        {
          rowElements[elementCount].Value = temp;
          rowElements[elementCount].N = node1->nodeData.nodeIndex - offset;
          elementCount++;
        }
        return 0;
      }
      return 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::RestrictedLaplacianMatrixFunction::Function (const TreeOctNode* node1, const TreeOctNode* node2)
    {
      int d1, d2, off1[3], off2[3];
      node1->depthAndOffset (d1, off1);
      node2->depthAndOffset (d2, off2);
      int dDepth = d2 - d1;
      int d;
      d = (off2[0] >> dDepth) - off1[0];
      if (d < 0)
        return 0;

      if (!dDepth)
      {
        if (!d)
        {
          d = off2[1] - off1[1];
          if (d < 0)
            return 0;
          else if (!d)
          {
            d = off2[2] - off1[2];
            if (d < 0)
              return 0;
          }
        }
        // Since we are getting the restricted matrix, we don't want to propogate out to terms that don't contribute...
        if (!TreeOctNode::Overlap2 (depth, offset, 0.5, d1, off1, radius))
          return 0;
        scratch[0] = FunctionData<Degree, Real>::SymmetricIndex (index[0], BinaryNode<Real>::Index (d1, off1[0]));
        scratch[1] = FunctionData<Degree, Real>::SymmetricIndex (index[1], BinaryNode<Real>::Index (d1, off1[1]));
        scratch[2] = FunctionData<Degree, Real>::SymmetricIndex (index[2], BinaryNode<Real>::Index (d1, off1[2]));
        Real temp = ot->GetLaplacian (scratch);
        if (node1 == node2)
          temp /= 2;
        if (fabs (temp) > EPSILON)
        {
          rowElements[elementCount].Value = temp;
          rowElements[elementCount].N = node1->nodeData.nodeIndex;
          elementCount++;
        }
        return 0;
      }
      return 1;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::GetMCIsoTriangles (const Real& isoValue,
                                       CoredMeshData* mesh,
                                       const int& fullDepthIso,
                                       const int& nonLinearFit,
                                       bool addBarycenter,
                                       bool polygonMesh)
    {
      //double t;
      TreeOctNode* temp;

      hash_map<long long, int> roots;
      hash_map<long long, std::pair<Real, Point3D<Real> > > *normalHash = new hash_map<long long, std::pair<Real, Point3D<Real> > > ();

      SetIsoSurfaceCorners (isoValue, 0, fullDepthIso);
      // At the point all of the corner values have been set and all nodes are valid. Now it's just a matter
      // of running marching cubes.

      fData.setValueTables (fData.VALUE_FLAG | fData.D_VALUE_FLAG, 0, postNormalSmooth);
      temp = tree.nextLeaf ();
      while (temp)
      {
        SetMCRootPositions (temp, 0, isoValue, roots, NULL, *normalHash, NULL, NULL, mesh, nonLinearFit);
        temp = tree.nextLeaf (temp);
      }

      fData.clearValueTables ();
      delete normalHash;

      // Now get the iso-surfaces, running from finest nodes to coarsest in order to allow for edge propogation from
      // finer faces to coarser ones.
      temp = tree.nextLeaf ();
      while (temp)
      {
        GetMCIsoTriangles (temp, mesh, roots, NULL, NULL, 0, 0, addBarycenter, polygonMesh);
        temp = tree.nextLeaf (temp);
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::GetMCIsoTriangles (const Real& isoValue,
                                       const int& subdivideDepth,
                                       CoredMeshData* mesh,
                                       const int& fullDepthIso,
                                       const int& nonLinearFit,
                                       bool addBarycenter,
                                       bool polygonMesh)
    {
      TreeOctNode* temp;
      hash_map<long long, int> boundaryRoots, *interiorRoots;
      hash_map<long long, std::pair<Real, Point3D<Real> > > *boundaryNormalHash, *interiorNormalHash;
      std::vector<Point3D<float> >* interiorPoints;

      int sDepth;
      if (subdivideDepth <= 0)
        sDepth = 0;
      else
        sDepth = fData.depth - subdivideDepth;
      if (sDepth < 0)
        sDepth = 0;

      SetIsoSurfaceCorners (isoValue, sDepth, fullDepthIso);
      // At this point all of the corner values have been set and all nodes are valid. Now it's just a matter
      // of running marching cubes.

      boundaryNormalHash = new hash_map<long long, std::pair<Real, Point3D<Real> > > ();
      int offSet = 0;
      SortedTreeNodes sNodes;
      sNodes.set (tree, 0);
      fData.setValueTables (fData.VALUE_FLAG | fData.D_VALUE_FLAG, 0, postNormalSmooth);

      // Set the root positions for all leaf nodes below the subdivide threshold
      SetBoundaryMCRootPositions (sDepth, isoValue, boundaryRoots, *boundaryNormalHash, mesh, nonLinearFit);

      for (int i = sNodes.nodeCount[sDepth]; i < sNodes.nodeCount[sDepth + 1]; i++)
      {
        interiorRoots = new hash_map<long long, int> ();
        interiorNormalHash = new hash_map<long long, std::pair<Real, Point3D<Real> > > ();
        interiorPoints = new std::vector<Point3D<float> > ();

        temp = sNodes.treeNodes[i]->nextLeaf ();
        while (temp)
        {
          if (MarchingCubes::HasRoots (temp->nodeData.mcIndex))
            SetMCRootPositions (temp, sDepth, isoValue, boundaryRoots, interiorRoots, *boundaryNormalHash, interiorNormalHash, interiorPoints, mesh, nonLinearFit);
          temp = sNodes.treeNodes[i]->nextLeaf (temp);
        }
        delete interiorNormalHash;

        temp = sNodes.treeNodes[i]->nextLeaf ();
        while (temp)
        {
          GetMCIsoTriangles (temp, mesh, boundaryRoots, interiorRoots, interiorPoints, offSet, sDepth, addBarycenter, polygonMesh);
          temp = sNodes.treeNodes[i]->nextLeaf (temp);
        }
        delete interiorRoots;
        delete interiorPoints;
        offSet = mesh->outOfCorePointCount ();
      }
      delete boundaryNormalHash;

      temp = tree.nextLeaf ();
      while (temp)
      {
        if (temp->depth () < sDepth)
          GetMCIsoTriangles (temp, mesh, boundaryRoots, NULL, NULL, 0, 0, addBarycenter, polygonMesh);
        temp = tree.nextLeaf (temp);
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> Real 
    Octree<Degree>::getCenterValue (const TreeOctNode* node)
    {
      int idx[3];
      Real value = 0;

      neighborKey2.getNeighbors (node);
      VertexData::CenterIndex (node, fData.depth, idx);
      idx[0] *= fData.res;
      idx[1] *= fData.res;
      idx[2] *= fData.res;
      for (int i = 0; i <= node->depth (); i++)
      {
        for (int j = 0; j < 3; j++)
        {
          for (int k = 0; k < 3; k++)
          {
            for (int l = 0; l < 3; l++)
            {
              const TreeOctNode* n = neighborKey2.neighbors[i].neighbors[j][k][l];
              if (n)
              {
                Real temp = n->nodeData.value;
                value += temp * Real (
                                      fData.valueTables[idx[0] + int (n->off[0])] *
                                      fData.valueTables[idx[1] + int (n->off[1])] *
                                      fData.valueTables[idx[2] + int (n->off[2])]);
              }
            }
          }
        }
      }
      if (node->children)
      {
        for (int i = 0; i < Cube::CORNERS; i++)
        {
          int ii = Cube::AntipodalCornerIndex (i);
          const TreeOctNode* n = &node->children[i];
          while (1)
          {
            value += n->nodeData.value * Real (
                                               fData.valueTables[idx[0] + int (n->off[0])] *
                                               fData.valueTables[idx[1] + int (n->off[1])] *
                                               fData.valueTables[idx[2] + int (n->off[2])]);
            if (n->children)
            {
              n = &n->children[ii];
            }
            else
            {
              break;
            }
          }
        }
      }
      return (static_cast<Real> (value));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> Real 
    Octree<Degree>::getCornerValue (const TreeOctNode* node, const int& corner)
    {
      int idx[3];
      Real value = 0;

      neighborKey2.getNeighbors (node);
      VertexData::CornerIndex (node, corner, fData.depth, idx);
      idx[0] *= fData.res;
      idx[1] *= fData.res;
      idx[2] *= fData.res;
      for (int i = 0; i <= node->depth (); i++)
      {
        for (int j = 0; j < 3; j++)
        {
          for (int k = 0; k < 3; k++)
          {
            for (int l = 0; l < 3; l++)
            {
              const TreeOctNode* n = neighborKey2.neighbors[i].neighbors[j][k][l];
              if (n)
              {
                Real temp = n->nodeData.value;
                value += temp * Real (
                                      fData.valueTables[idx[0] + int (n->off[0])] *
                                      fData.valueTables[idx[1] + int (n->off[1])] *
                                      fData.valueTables[idx[2] + int (n->off[2])]);
              }
            }
          }
        }
      }
      int x, y, z, d = node->depth ();
      Cube::FactorCornerIndex (corner, x, y, z);
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 2; j++)
        {
          for (int k = 0; k < 2; k++)
          {
            const TreeOctNode* n = neighborKey2.neighbors[d].neighbors[x + i][y + j][z + k];
            if (n)
            {
              int ii = Cube::AntipodalCornerIndex (Cube::CornerIndex (i, j, k));
              while (n->children)
              {
                n = &n->children[ii];
                value += n->nodeData.value * Real (
                                                   fData.valueTables[idx[0] + int (n->off[0])] *
                                                   fData.valueTables[idx[1] + int (n->off[1])] *
                                                   fData.valueTables[idx[2] + int (n->off[2])]);
              }
            }
          }
        }
      }
      return value;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::getCornerValueAndNormal (const TreeOctNode* node, const int& corner, Real& value, Point3D<Real>& normal)
    {
      int idx[3], index[3];
      value = normal.coords[0] = normal.coords[1] = normal.coords[2] = 0;

      neighborKey2.getNeighbors (node);
      VertexData::CornerIndex (node, corner, fData.depth, idx);
      idx[0] *= fData.res;
      idx[1] *= fData.res;
      idx[2] *= fData.res;
      for (int i = 0; i <= node->depth (); i++)
      {
        for (int j = 0; j < 3; j++)
        {
          for (int k = 0; k < 3; k++)
          {
            for (int l = 0; l < 3; l++)
            {
              const TreeOctNode* n = neighborKey2.neighbors[i].neighbors[j][k][l];
              if (n)
              {
                Real temp = n->nodeData.value;
                index[0] = idx[0] + int (n->off[0]);
                index[1] = idx[1] + int (n->off[1]);
                index[2] = idx[2] + int (n->off[2]);
                value += temp * Real (fData.valueTables[index[0]] * fData.valueTables[index[1]] * fData.valueTables[index[2]]);
                normal.coords[0] += temp * Real (fData.dValueTables[index[0]] * fData.valueTables[index[1]] * fData.valueTables[index[2]]);
                normal.coords[1] += temp * Real (fData.valueTables[index[0]] * fData.dValueTables[index[1]] * fData.valueTables[index[2]]);
                normal.coords[2] += temp * Real (fData.valueTables[index[0]] * fData.valueTables[index[1]] * fData.dValueTables[index[2]]);
              }
            }
          }
        }
      }
      int x, y, z, d = node->depth ();
      Cube::FactorCornerIndex (corner, x, y, z);
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 2; j++)
        {
          for (int k = 0; k < 2; k++)
          {
            const TreeOctNode* n = neighborKey2.neighbors[d].neighbors[x + i][y + j][z + k];
            if (n)
            {
              int ii = Cube::AntipodalCornerIndex (Cube::CornerIndex (i, j, k));
              while (n->children)
              {
                n = &n->children[ii];
                Real temp = n->nodeData.value;
                index[0] = idx[0] + int (n->off[0]);
                index[1] = idx[1] + int (n->off[1]);
                index[2] = idx[2] + int (n->off[2]);
                value += temp * Real (fData.valueTables[index[0]] * fData.valueTables[index[1]] * fData.valueTables[index[2]]);
                normal.coords[0] += temp * Real (fData.dValueTables[index[0]] * fData.valueTables[index[1]] * fData.valueTables[index[2]]);
                normal.coords[1] += temp * Real (fData.valueTables[index[0]] * fData.dValueTables[index[1]] * fData.valueTables[index[2]]);
                normal.coords[2] += temp * Real (fData.valueTables[index[0]] * fData.valueTables[index[1]] * fData.dValueTables[index[2]]);
              }
            }
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> Real 
    Octree<Degree>::GetIsoValue ()
    {
      if (this->width <= 3)
      {
        TreeOctNode* temp;
        Real isoValue, weightSum, w;

        neighborKey2.set (fData.depth);
        fData.setValueTables (fData.VALUE_FLAG, 0);

        isoValue = weightSum = 0;
        temp = tree.nextNode ();
        while (temp)
        {
          w = temp->nodeData.centerWeightContribution;
          if (w > EPSILON)
          {
            isoValue += getCenterValue (temp) * w;
            weightSum += w;
          }
          temp = tree.nextNode (temp);
        }
        return isoValue / weightSum;
      }
      else
      {
        const TreeOctNode* temp;
        Real isoValue, weightSum, w;
        //Real myRadius;
        PointIndexValueFunction cf;

        fData.setValueTables (fData.VALUE_FLAG, 0);
        cf.valueTables = fData.valueTables;
        cf.res2 = fData.res2;
        //myRadius = radius;
        isoValue = weightSum = 0;
        temp = tree.nextNode ();
        while (temp)
        {
          w = temp->nodeData.centerWeightContribution;
          if (w > EPSILON)
          {
            cf.value = 0;
            int idx[3];
            VertexData::CenterIndex (temp, fData.depth, idx);
            cf.index[0] = idx[0] * fData.res;
            cf.index[1] = idx[1] * fData.res;
            cf.index[2] = idx[2] * fData.res;
            TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, width, &cf);
            isoValue += cf.value*w;
            weightSum += w;
          }
          temp = tree.nextNode (temp);
        }
        return isoValue / weightSum;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::SetIsoSurfaceCorners (const Real& isoValue, const int& subdivideDepth, const int& /*fullDepthIso*/)
    {
      int i, j;
      hash_map<long long, Real> values;
      Real cornerValues[Cube::CORNERS];
      PointIndexValueFunction cf;
      TreeOctNode* temp;
      //int leafCount = tree.leaves ();
      long long key;
      SortedTreeNodes *sNodes = new SortedTreeNodes ();
      sNodes->set (tree, 0);
      temp = tree.nextNode ();
      while (temp)
      {
        temp->nodeData.mcIndex = 0;
        temp = tree.nextNode (temp);
      }
      TreeNodeData::UseIndex = 0;
      // Start by setting the corner values of all the nodes
      cf.valueTables = fData.valueTables;
      cf.res2 = fData.res2;
      for (i = 0; i < sNodes->nodeCount[subdivideDepth]; i++)
      {
        temp = sNodes->treeNodes[i];
        if (!temp->children)
        {
          for (j = 0; j < Cube::CORNERS; j++)
          {
            if (this->width <= 3)
            {
              cornerValues[j] = getCornerValue (temp, j);
            }
            else
            {
              cf.value = 0;
              int idx[3];
              VertexData::CornerIndex (temp, j, fData.depth, idx);
              cf.index[0] = idx[0] * fData.res;
              cf.index[1] = idx[1] * fData.res;
              cf.index[2] = idx[2] * fData.res;
              TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, width, &cf);
              cornerValues[j] = cf.value;
            }
          }
          temp->nodeData.mcIndex = MarchingCubes::GetIndex (cornerValues, isoValue);

          if (temp->parent)
          {
            TreeOctNode* parent = temp->parent;
            int c = int (temp - temp->parent->children);
            int mcid = temp->nodeData.mcIndex & (1 << MarchingCubes::cornerMap[c]);

            if (mcid)
            {
              parent->nodeData.mcIndex |= mcid;
              while (1)
              {
                if (parent->parent && (parent - parent->parent->children) == c)
                {
                  parent->parent->nodeData.mcIndex |= mcid;
                  parent = parent->parent;
                }
                else
                {
                  break;
                }
              }
            }
          }
        }
      }

      MemoryUsage ();

      for (i = sNodes->nodeCount[subdivideDepth]; i < sNodes->nodeCount[subdivideDepth + 1]; i++)
      {
        temp = sNodes->treeNodes[i]->nextLeaf ();
        while (temp)
        {
          for (j = 0; j < Cube::CORNERS; j++)
          {
            int idx[3];
            key = VertexData::CornerIndex (temp, j, fData.depth, idx);
            cf.index[0] = idx[0] * fData.res;
            cf.index[1] = idx[1] * fData.res;
            cf.index[2] = idx[2] * fData.res;
            if (values.find (key) != values.end ())
            {
              cornerValues[j] = values[key];
            }
            else
            {
              if (this->width <= 3)
              {
                values[key] = cornerValues[j] = getCornerValue (temp, j);
              }
              else
              {
                cf.value = 0;
                TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, width, &cf);
                values[key] = cf.value;
                cornerValues[j] = cf.value;
              }
            }
          }
          temp->nodeData.mcIndex = MarchingCubes::GetIndex (cornerValues, isoValue);

          if (temp->parent)
          {
            TreeOctNode* parent = temp->parent;
            int c = int (temp - temp->parent->children);
            int mcid = temp->nodeData.mcIndex & (1 << MarchingCubes::cornerMap[c]);

            if (mcid)
            {
              parent->nodeData.mcIndex |= mcid;
              while (1)
              {
                if (parent->parent && (parent - parent->parent->children) == c)
                {
                  parent->parent->nodeData.mcIndex |= mcid;
                  parent = parent->parent;
                }
                else
                {
                  break;
                }
              }
            }
          }

          temp = sNodes->treeNodes[i]->nextLeaf (temp);
        }
        MemoryUsage ();
        values.clear ();
      }
      delete sNodes;
      printf ("Memory Usage: %.3f MB\n", float (MemoryUsage ()));

      if (subdivideDepth)
      {
        PreValidate (isoValue, fData.depth, subdivideDepth);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::Subdivide (TreeOctNode* node, const Real& isoValue, const int& maxDepth)
    {
      int i, j, c[4];
      Real value;
      int cornerIndex2[Cube::CORNERS];
      PointIndexValueFunction cf;
      cf.valueTables = fData.valueTables;
      cf.res2 = fData.res2;
      node->initChildren ();
      // Since we are allocating blocks, it is possible that some of the memory was pre-allocated with
      // the wrong initialization

      // Now set the corner values for the new children
      // Copy old corner values
      for (i = 0; i < Cube::CORNERS; i++)
      {
        cornerIndex2[i] = node->nodeData.mcIndex & (1 << MarchingCubes::cornerMap[i]);
      }
      // 8 of 27 corners set

      // Set center corner
      cf.value = 0;
      int idx[3];
      VertexData::CenterIndex (node, maxDepth, idx);
      cf.index[0] = idx[0] * fData.res;
      cf.index[1] = idx[1] * fData.res;
      cf.index[2] = idx[2] * fData.res;
      if (this->width <= 3)
      {
        value = getCenterValue (node);
      }
      else
      {
        TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, width, &cf);
        value = cf.value;
      }
      if (value < isoValue)
      {
        for (i = 0; i < Cube::CORNERS; i++)
        {
          cornerIndex2[i] |= 1 << MarchingCubes::cornerMap[Cube::AntipodalCornerIndex (i)];
        }
      }
      // 9 of 27 set

      // Set face corners
      for (i = 0; i < Cube::NEIGHBORS; i++)
      {
        int dir, offset, e;
        Cube::FactorFaceIndex (i, dir, offset);
        cf.value = 0;
        int idx[3];
        VertexData::FaceIndex (node, i, maxDepth, idx);
        cf.index[0] = idx[0] * fData.res;
        cf.index[1] = idx[1] * fData.res;
        cf.index[2] = idx[2] * fData.res;
        TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, width, &cf);
        value = cf.value;
        Cube::FaceCorners (i, c[0], c[1], c[2], c[3]);
        e = Cube::EdgeIndex (dir, 0, 0);
        if (value < isoValue)
        {
          for (j = 0; j < 4; j++)
          {
            cornerIndex2[c[j]] |= 1 << MarchingCubes::cornerMap[Cube::EdgeReflectCornerIndex (c[j], e)];
          }
        }
      }
      // 15 of 27 set

      // Set edge corners
      for (i = 0; i < Cube::EDGES; i++)
      {
        int o, i1, i2, f;
        Cube::FactorEdgeIndex (i, o, i1, i2);
        cf.value = 0;
        int idx[3];
        VertexData::EdgeIndex (node, i, maxDepth, idx);
        cf.index[0] = idx[0] * fData.res;
        cf.index[1] = idx[1] * fData.res;
        cf.index[2] = idx[2] * fData.res;
        TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, width, &cf);
        value = cf.value;
        Cube::EdgeCorners (i, c[0], c[1]);
        f = Cube::FaceIndex (o, 0);
        if (value < isoValue)
        {
          for (j = 0; j < 2; j++)
          {
            cornerIndex2[c[j]] |= 1 << MarchingCubes::cornerMap[Cube::FaceReflectCornerIndex (c[j], f)];
          }
        }
      }
      // 27 of 27 set

      for (i = 0; i < Cube::CORNERS; i++)
      {
        node->children[i].nodeData.mcIndex = cornerIndex2[i];
      }
    }

    template<int Degree>
    int Octree<Degree>
    ::InteriorFaceRootCount (const TreeOctNode* node, const int &faceIndex, const int& maxDepth)
    {
      int c1, c2, e1, e2, dir, off, cnt = 0;
      int corners[Cube::CORNERS / 2];
      if (node->children)
      {
        Cube::FaceCorners (faceIndex, corners[0], corners[1], corners[2], corners[3]);
        Cube::FactorFaceIndex (faceIndex, dir, off);
        c1 = corners[0];
        c2 = corners[3];
        switch (dir)
        {
          case 0:
            e1 = Cube::EdgeIndex (1, off, 1);
            e2 = Cube::EdgeIndex (2, off, 1);
            break;
          case 1:
            e1 = Cube::EdgeIndex (0, off, 1);
            e2 = Cube::EdgeIndex (2, 1, off);
            break;
          case 2:
            e1 = Cube::EdgeIndex (0, 1, off);
            e2 = Cube::EdgeIndex (1, 1, off);
            break;
        };
        cnt += EdgeRootCount (&node->children[c1], e1, maxDepth) + EdgeRootCount (&node->children[c1], e2, maxDepth);
        switch (dir)
        {
          case 0:
            e1 = Cube::EdgeIndex (1, off, 0);
            e2 = Cube::EdgeIndex (2, off, 0);
            break;
          case 1:
            e1 = Cube::EdgeIndex (0, off, 0);
            e2 = Cube::EdgeIndex (2, 0, off);
            break;
          case 2:
            e1 = Cube::EdgeIndex (0, 0, off);
            e2 = Cube::EdgeIndex (1, 0, off);
            break;
        };
        cnt += EdgeRootCount (&node->children[c2], e1, maxDepth) + EdgeRootCount (&node->children[c2], e2, maxDepth);
        for (int i = 0; i < Cube::CORNERS / 2; i++)
        {
          if (node->children[corners[i]].children)
          {
            cnt += InteriorFaceRootCount (&node->children[corners[i]], faceIndex, maxDepth);
          }
        }
      }
      return cnt;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::EdgeRootCount (const TreeOctNode* node, const int& edgeIndex, const int& maxDepth)
    {
      int f1, f2, c1, c2;
      const TreeOctNode* temp;
      Cube::FacesAdjacentToEdge (edgeIndex, f1, f2);

      int eIndex;
      const TreeOctNode* finest = node;
      eIndex = edgeIndex;
      if (node->depth () < maxDepth)
      {
        temp = node->faceNeighbor (f1);
        if (temp && temp->children)
        {
          finest = temp;
          eIndex = Cube::FaceReflectEdgeIndex (edgeIndex, f1);
        }
        else
        {
          temp = node->faceNeighbor (f2);
          if (temp && temp->children)
          {
            finest = temp;
            eIndex = Cube::FaceReflectEdgeIndex (edgeIndex, f2);
          }
          else
          {
            temp = node->edgeNeighbor (edgeIndex);
            if (temp && temp->children)
            {
              finest = temp;
              eIndex = Cube::EdgeReflectEdgeIndex (edgeIndex);
            }
          }
        }
      }

      Cube::EdgeCorners (eIndex, c1, c2);
      if (finest->children)
      {
        return EdgeRootCount (&finest->children[c1], eIndex, maxDepth) + EdgeRootCount (&finest->children[c2], eIndex, maxDepth);
      }
      else
      {
        return MarchingCubes::HasEdgeRoots (finest->nodeData.mcIndex, eIndex);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::IsBoundaryFace (const TreeOctNode* node, const int& faceIndex, const int& subdivideDepth)
    {
      int dir, offset, d, o[3], idx;

      if (subdivideDepth < 0)
      {
        return 0;
      }
      if (node->d <= subdivideDepth)
      {
        return 1;
      }
      Cube::FactorFaceIndex (faceIndex, dir, offset);
      node->depthAndOffset (d, o);

      idx = (int (o[dir]) << 1) + (offset << 1);
      return !(idx % (2 << (int (node->d) - subdivideDepth)));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::IsBoundaryEdge (const TreeOctNode* node, const int& edgeIndex, const int& subdivideDepth)
    {
      int dir, x, y;
      Cube::FactorEdgeIndex (edgeIndex, dir, x, y);
      return IsBoundaryEdge (node, dir, x, y, subdivideDepth);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::IsBoundaryEdge (const TreeOctNode* node, const int& dir, const int& x, const int& y, const int& subdivideDepth)
    {
      int d, o[3], idx1, idx2, mask;

      if (subdivideDepth < 0)
      {
        return 0;
      }
      if (node->d <= subdivideDepth)
      {
        return 1;
      }
      node->depthAndOffset (d, o);

      // initialize to remove warnings
      idx1 = idx2 = 0;
      switch (dir)
      {
        case 0:
          idx1 = (int (o[1]) << 1) + (x << 1);
          idx2 = (int (o[2]) << 1) + (y << 1);
          break;
        case 1:
          idx1 = (int (o[0]) << 1) + (x << 1);
          idx2 = (int (o[2]) << 1) + (y << 1);
          break;
        case 2:
          idx1 = (int (o[0]) << 1) + (x << 1);
          idx2 = (int (o[1]) << 1) + (y << 1);
          break;
      }
      mask = 2 << (int (node->d) - subdivideDepth);
      return !(idx1 % (mask)) || !(idx2 % (mask));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::PreValidate (TreeOctNode* node, const Real& isoValue, const int& maxDepth, const int& subdivideDepth)
    {
      int sub = 0;
      if (node->children)
      {
        printf ("Bad Pre-Validate\n");
      }
      //	if (int (node->d)<subdivideDepth){sub = 1;}
      for (int i = 0; i < Cube::NEIGHBORS && !sub; i++)
      {
        TreeOctNode* neighbor = node->faceNeighbor (i);
        if (neighbor && neighbor->children)
        {
          if (IsBoundaryFace (node, i, subdivideDepth) && InteriorFaceRootCount (neighbor, Cube::FaceReflectFaceIndex (i, i), maxDepth))
          {
            sub = 1;
          }
        }
      }
      if (sub)
      {
        Subdivide (node, isoValue, maxDepth);
        for (int i = 0; i < Cube::NEIGHBORS; i++)
        {
          if (IsBoundaryFace (node, i, subdivideDepth) && InteriorFaceRootCount (node, i, maxDepth))
          {
            TreeOctNode* neighbor = node->faceNeighbor (i);
            while (neighbor && !neighbor->children)
            {
              PreValidate (neighbor, isoValue, maxDepth, subdivideDepth);
              neighbor = node->faceNeighbor (i);
            }
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::PreValidate (const Real& isoValue, const int& maxDepth, const int& subdivideDepth)
    {
      TreeOctNode* temp;

      temp = tree.nextLeaf ();
      while (temp)
      {
        PreValidate (temp, isoValue, maxDepth, subdivideDepth);
        temp = tree.nextLeaf (temp);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::Validate (TreeOctNode* node, const Real& isoValue, const int& maxDepth, const int& fullDepthIso)
    {
      int i, sub = 0;
      TreeOctNode* treeNode = node;
      TreeOctNode* neighbor;
      if (node->depth () >= maxDepth || node->children)
      {
        return;
      }

      // Check if full-depth extraction is enabled and we have an iso-node that is not at maximum depth
      if (!sub && fullDepthIso && MarchingCubes::HasRoots (node->nodeData.mcIndex))
      {
        sub = 1;
      }

      // Check if the node has faces that are ambiguous and are adjacent to finer neighbors
      for (i = 0; i < Cube::NEIGHBORS && !sub; i++)
      {
        neighbor = treeNode->faceNeighbor (i);
        if (neighbor && neighbor->children)
        {
          if (MarchingCubes::IsAmbiguous (node->nodeData.mcIndex, i))
          {
            sub = 1;
          }
        }
      }

      // Check if the node has edges with more than one root
      for (i = 0; i < Cube::EDGES && !sub; i++)
      {
        if (EdgeRootCount (node, i, maxDepth) > 1)
        {
          sub = 1;
        }
      }

      for (i = 0; i < Cube::NEIGHBORS && !sub; i++)
      {
        neighbor = node->faceNeighbor (i);
        if (neighbor && neighbor->children &&
            !MarchingCubes::HasFaceRoots (node->nodeData.mcIndex, i) &&
            InteriorFaceRootCount (neighbor, Cube::FaceReflectFaceIndex (i, i), maxDepth))
        {
          sub = 1;
        }
      }
      if (sub)
      {
        Subdivide (node, isoValue, maxDepth);
        for (i = 0; i < Cube::NEIGHBORS; i++)
        {
          neighbor = treeNode->faceNeighbor (i);
          if (neighbor && !neighbor->children)
          {
            Validate (neighbor, isoValue, maxDepth, fullDepthIso);
          }
        }
        for (i = 0; i < Cube::EDGES; i++)
        {
          neighbor = treeNode->edgeNeighbor (i);
          if (neighbor && !neighbor->children)
          {
            Validate (neighbor, isoValue, maxDepth, fullDepthIso);
          }
        }
        for (i = 0; i < Cube::CORNERS; i++)
        {
          if (!node->children[i].children)
          {
            Validate (&node->children[i], isoValue, maxDepth, fullDepthIso);
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::Validate (TreeOctNode* node, const Real& isoValue, const int& maxDepth, const int& fullDepthIso, const int& subdivideDepth)
    {
      int i, sub = 0;
      TreeOctNode* treeNode = node;
      TreeOctNode* neighbor;
      if (node->depth () >= maxDepth || node->children)
      {
        return;
      }

      // Check if full-depth extraction is enabled and we have an iso-node that is not at maximum depth
      if (!sub && fullDepthIso && MarchingCubes::HasRoots (node->nodeData.mcIndex))
      {
        sub = 1;
      }

      // Check if the node has faces that are ambiguous and are adjacent to finer neighbors
      for (i = 0; i < Cube::NEIGHBORS && !sub; i++)
      {
        neighbor = treeNode->faceNeighbor (i);
        if (neighbor && neighbor->children)
        {
          if (MarchingCubes::IsAmbiguous (node->nodeData.mcIndex, i) || IsBoundaryFace (node, i, subdivideDepth))
          {
            sub = 1;
          }
        }
      }

      // Check if the node has edges with more than one root
      for (i = 0; i < Cube::EDGES && !sub; i++)
      {
        if (EdgeRootCount (node, i, maxDepth) > 1)
        {
          sub = 1;
        }
      }

      for (i = 0; i < Cube::NEIGHBORS && !sub; i++)
      {
        neighbor = node->faceNeighbor (i);
        if (neighbor && neighbor->children && !MarchingCubes::HasFaceRoots (node->nodeData.mcIndex, i) &&
            InteriorFaceRootCount (neighbor, Cube::FaceReflectFaceIndex (i, i), maxDepth))
        {
          sub = 1;
        }
      }
      if (sub)
      {
        Subdivide (node, isoValue, maxDepth);
        for (i = 0; i < Cube::NEIGHBORS; i++)
        {
          neighbor = treeNode->faceNeighbor (i);
          if (neighbor && !neighbor->children)
          {
            Validate (neighbor, isoValue, maxDepth, fullDepthIso, subdivideDepth);
          }
        }
        for (i = 0; i < Cube::EDGES; i++)
        {
          neighbor = treeNode->edgeNeighbor (i);
          if (neighbor && !neighbor->children)
          {
            Validate (neighbor, isoValue, maxDepth, fullDepthIso, subdivideDepth);
          }
        }
        for (i = 0; i < Cube::CORNERS; i++)
        {
          if (!node->children[i].children)
          {
            Validate (&node->children[i], isoValue, maxDepth, fullDepthIso, subdivideDepth);
          }
        }
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////
    // The assumption made when calling this code is that the edge has at most one root //
    //////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRoot (const RootInfo& ri, const Real& isoValue, Point3D<Real> & position, hash_map<long long, std::pair<Real, Point3D<Real> > >& normalHash, const int& nonLinearFit)
    {
      int c1, c2;
      Cube::EdgeCorners (ri.edgeIndex, c1, c2);
      if (!MarchingCubes::HasEdgeRoots (ri.node->nodeData.mcIndex, ri.edgeIndex))
      {
        return 0;
      }

      long long key;
      Point3D<Real> n[2];
      PointIndexValueAndNormalFunction cnf;
      cnf.valueTables = fData.valueTables;
      cnf.dValueTables = fData.dValueTables;
      cnf.res2 = fData.res2;

      int i, o, i1, i2, rCount = 0;
      Polynomial<2> P;
      std::vector<double> roots;
      double x0, x1;
      Real center, width;
      Real averageRoot = 0;
      Cube::FactorEdgeIndex (ri.edgeIndex, o, i1, i2);
      int idx[3];
      key = VertexData::CornerIndex (ri.node, c1, fData.depth, idx);
      cnf.index[0] = idx[0] * fData.res;
      cnf.index[1] = idx[1] * fData.res;
      cnf.index[2] = idx[2] * fData.res;

      if (normalHash.find (key) == normalHash.end ())
      {
        cnf.value = 0;
        cnf.normal.coords[0] = cnf.normal.coords[1] = cnf.normal.coords[2] = 0;
        // Careful here as the normal isn't quite accurate...  (i.e. postNormalSmooth is ignored)
#if 0
        if (this->width <= 3)
        {
          getCornerValueAndNormal (ri.node, c1, cnf.value, cnf.normal);
        }
        else
        {
          TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, this->width, &cnf);
        }
#else
        TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, this->width, &cnf);
#endif
        normalHash[key] = std::pair<Real, Point3D<Real> > (cnf.value, cnf.normal);
      }
      x0 = normalHash[key].first;
      n[0] = normalHash[key].second;

      key = VertexData::CornerIndex (ri.node, c2, fData.depth, idx);
      cnf.index[0] = idx[0] * fData.res;
      cnf.index[1] = idx[1] * fData.res;
      cnf.index[2] = idx[2] * fData.res;
      if (normalHash.find (key) == normalHash.end ())
      {
        cnf.value = 0;
        cnf.normal.coords[0] = cnf.normal.coords[1] = cnf.normal.coords[2] = 0;
#if 0
        if (this->width <= 3)
        {
          getCornerValueAndNormal (ri.node, c2, cnf.value, cnf.normal);
        }
        else
        {
          TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, this->width, &cnf);
        }
#else
        TreeOctNode::ProcessPointAdjacentNodes (fData.depth, idx, &tree, this->width, &cnf);
#endif
        normalHash[key] = std::pair<Real, Point3D<Real> > (cnf.value, cnf.normal);
      }
      x1 = normalHash[key].first;
      n[1] = normalHash[key].second;

      Point3D<Real> c;
      ri.node->centerAndWidth (c, width);
      center = c.coords[o];
      for (i = 0; i < DIMENSION; i++)
      {
        n[0].coords[i] *= width;
        n[1].coords[i] *= width;
      }

      switch (o)
      {
        case 0:
          position.coords[1] = c.coords[1] - width / 2 + width * static_cast<Real> (i1);
          position.coords[2] = c.coords[2] - width / 2 + width * static_cast<Real> (i2);
          break;
        case 1:
          position.coords[0] = c.coords[0] - width / 2 + width * static_cast<Real> (i1);
          position.coords[2] = c.coords[2] - width / 2 + width * static_cast<Real> (i2);
          break;
        case 2:
          position.coords[0] = c.coords[0] - width / 2 + width * static_cast<Real> (i1);
          position.coords[1] = c.coords[1] - width / 2 + width * static_cast<Real> (i2);
          break;
      }
      double dx0, dx1;
      dx0 = n[0].coords[o];
      dx1 = n[1].coords[o];

      // The scaling will turn the Hermite Spline into a quadratic
      double scl = (x1 - x0) / ((dx1 + dx0) / 2);
      dx0 *= scl;
      dx1 *= scl;

      // Hermite Spline
      P.coefficients[0] = x0;
      P.coefficients[1] = dx0;
      P.coefficients[2] = 3 * (x1 - x0) - dx1 - 2 * dx0;

      P.getSolutions (isoValue, roots, EPSILON);
      for (i = 0; i<int (roots.size ()); i++)
      {
        if (roots[i] >= 0 && roots[i] <= 1)
        {
          averageRoot += Real (roots[i]);
          rCount++;
        }
      }
      if (rCount && nonLinearFit)
      {
        averageRoot /= static_cast<Real> (rCount);
      }
      else
      {
        averageRoot = Real ((x0 - isoValue) / (x0 - x1));
      }

      position.coords[o] = Real (center - width / 2 + width * averageRoot);
      return 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRoot (
        const RootInfo& ri, const Real& isoValue, const int& /*maxDepth*/, 
        Point3D<Real>& position, hash_map<long long, 
        std::pair<Real, Point3D<Real> > >& normals, Point3D<Real>* /*normal*/, const int& nonLinearFit)
    {
      if (!MarchingCubes::HasRoots (ri.node->nodeData.mcIndex))
      {
        return 0;
      }
      return GetRoot (ri, isoValue, position, normals, nonLinearFit);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRootIndex (const TreeOctNode* node, const int& edgeIndex, const int& maxDepth, const int& sDepth, RootInfo& ri)
    {
      int c1, c2, f1, f2;
      const TreeOctNode *temp, *finest;
      int finestIndex;

      Cube::FacesAdjacentToEdge (edgeIndex, f1, f2);

      finest = node;
      finestIndex = edgeIndex;
      if (node->depth () < maxDepth)
      {
        if (IsBoundaryFace (node, f1, sDepth))
        {
          temp = NULL;
        }
        else
        {
          temp = node->faceNeighbor (f1);
        }
        if (temp && temp->children)
        {
          finest = temp;
          finestIndex = Cube::FaceReflectEdgeIndex (edgeIndex, f1);
        }
        else
        {
          if (IsBoundaryFace (node, f2, sDepth))
          {
            temp = NULL;
          }
          else
          {
            temp = node->faceNeighbor (f2);
          }
          if (temp && temp->children)
          {
            finest = temp;
            finestIndex = Cube::FaceReflectEdgeIndex (edgeIndex, f2);
          }
          else
          {
            if (IsBoundaryEdge (node, edgeIndex, sDepth))
            {
              temp = NULL;
            }
            else
            {
              temp = node->edgeNeighbor (edgeIndex);
            }
            if (temp && temp->children)
            {
              finest = temp;
              finestIndex = Cube::EdgeReflectEdgeIndex (edgeIndex);
            }
          }
        }
      }

      Cube::EdgeCorners (finestIndex, c1, c2);
      if (finest->children)
      {
        if (GetRootIndex (&finest->children[c1], finestIndex, maxDepth, sDepth, ri))
        {
          return 1;
        }
        else if (GetRootIndex (&finest->children[c2], finestIndex, maxDepth, sDepth, ri))
        {
          return 1;
        }
        else
        {
          return 0;
        }
      }
      else
      {
        if (!(MarchingCubes::edgeMask[finest->nodeData.mcIndex] & (1 << finestIndex)))
        {
          return 0;
        }

        int o, i1, i2;
        Cube::FactorEdgeIndex (finestIndex, o, i1, i2);
        int d, off[3];
        finest->depthAndOffset (d, off);
        ri.node = finest;
        ri.edgeIndex = finestIndex;
        int eIndex[2], offset;
        offset = BinaryNode<Real>::Index (d, off[o]);
        switch (o)
        {
          case 0:
            eIndex[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[1], i1);
            eIndex[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[2], i2);
            break;
          case 1:
            eIndex[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[0], i1);
            eIndex[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[2], i2);
            break;
          case 2:
            eIndex[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[0], i1);
            eIndex[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[1], i2);
            break;
        }
        ri.key = static_cast<long long> (o) | static_cast<long long> (eIndex[0]) << 5 | static_cast<long long> (eIndex[1]) << 25 | static_cast<long long> (offset) << 45;
        return 1;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRootIndex (const TreeOctNode* node, const int& edgeIndex, const int& maxDepth, RootInfo& ri)
    {
      int c1, c2, f1, f2;
      const TreeOctNode *temp, *finest;
      int finestIndex;


      // The assumption is that the super-edge has a root along it.
      if (!(MarchingCubes::edgeMask[node->nodeData.mcIndex] & (1 << edgeIndex)))
      {
        return 0;
      }

      Cube::FacesAdjacentToEdge (edgeIndex, f1, f2);

      finest = node;
      finestIndex = edgeIndex;
      if (node->depth () < maxDepth)
      {
        temp = node->faceNeighbor (f1);
        if (temp && temp->children)
        {
          finest = temp;
          finestIndex = Cube::FaceReflectEdgeIndex (edgeIndex, f1);
        }
        else
        {
          temp = node->faceNeighbor (f2);
          if (temp && temp->children)
          {
            finest = temp;
            finestIndex = Cube::FaceReflectEdgeIndex (edgeIndex, f2);
          }
          else
          {
            temp = node->edgeNeighbor (edgeIndex);
            if (temp && temp->children)
            {
              finest = temp;
              finestIndex = Cube::EdgeReflectEdgeIndex (edgeIndex);
            }
          }
        }
      }

      Cube::EdgeCorners (finestIndex, c1, c2);
      if (finest->children)
      {
        if (GetRootIndex (&finest->children[c1], finestIndex, maxDepth, ri))
        {
          return 1;
        }
        else if (GetRootIndex (&finest->children[c2], finestIndex, maxDepth, ri))
        {
          return 1;
        }
        else
        {
          return 0;
        }
      }
      else
      {
        int o, i1, i2;
        Cube::FactorEdgeIndex (finestIndex, o, i1, i2);
        int d, off[3];
        finest->depthAndOffset (d, off);
        ri.node = finest;
        ri.edgeIndex = finestIndex;
        int offset, eIndex[2];
        offset = BinaryNode<Real>::Index (d, off[o]);
        //initialize to remove warnings
        eIndex[0] = eIndex[1] = 0;
        switch (o)
        {
          case 0:
            eIndex[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[1], i1);
            eIndex[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[2], i2);
            break;
          case 1:
            eIndex[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[0], i1);
            eIndex[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[2], i2);
            break;
          case 2:
            eIndex[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[0], i1);
            eIndex[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[1], i2);
            break;
        }
        ri.key = static_cast<long long> (o) | static_cast<long long> (eIndex[0]) << 5 | static_cast<long long> (eIndex[1]) << 25 | static_cast<long long> (offset) << 45;
        return (1);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRootPair (const RootInfo& ri, const int& maxDepth, RootInfo& pair)
    {
      const TreeOctNode* node = ri.node;
      int c1, c2, c;
      Cube::EdgeCorners (ri.edgeIndex, c1, c2);
      while (node->parent)
      {
        c = int (node - node->parent->children);
        if (c != c1 && c != c2)
        {
          return 0;
        }
        if (!MarchingCubes::HasEdgeRoots (node->parent->nodeData.mcIndex, ri.edgeIndex))
        {
          if (c == c1)
          {
            return GetRootIndex (&node->parent->children[c2], ri.edgeIndex, maxDepth, pair);
          }
          else
          {
            return GetRootIndex (&node->parent->children[c1], ri.edgeIndex, maxDepth, pair);
          }
        }
        node = node->parent;
      }
      return 0;

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetRootIndex (const long long& key, hash_map<long long, int>& boundaryRoots, hash_map<long long, int>* interiorRoots, CoredPointIndex& index)
    {
      hash_map<long long, int>::iterator rootIter = boundaryRoots.find (key);
      if (rootIter != boundaryRoots.end ())
      {
        index.inCore = 1;
        index.index = rootIter->second;
        return 1;
      }
      else if (interiorRoots)
      {
        rootIter = interiorRoots->find (key);
        if (rootIter != interiorRoots->end ())
        {
          index.inCore = 0;
          index.index = rootIter->second;
          return 1;
        }
      }
      return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::SetMCRootPositions (
        TreeOctNode* node, const int& sDepth, const Real& isoValue,
        hash_map<long long, int>& boundaryRoots, hash_map<long long, int>* interiorRoots,
        hash_map<long long, std::pair<Real, Point3D<Real> > >& boundaryNormalHash, hash_map<long long, std::pair<Real, Point3D<Real> > >* interiorNormalHash,
        std::vector<Point3D<float> >* interiorPositions,
        CoredMeshData* mesh, const int& nonLinearFit)
    {
      Point3D<Real> position;
      int i, j, k, eIndex;
      RootInfo ri;
      int count = 0;
      if (!MarchingCubes::HasRoots (node->nodeData.mcIndex))
      {
        return 0;
      }
      for (i = 0; i < DIMENSION; i++)
      {
        for (j = 0; j < 2; j++)
        {
          for (k = 0; k < 2; k++)
          {
            long long key;
            eIndex = Cube::EdgeIndex (i, j, k);
            if (GetRootIndex (node, eIndex, fData.depth, ri))
            {
              key = ri.key;
              if (!interiorRoots || IsBoundaryEdge (node, i, j, k, sDepth))
              {
                if (boundaryRoots.find (key) == boundaryRoots.end ())
                {
                  GetRoot (ri, isoValue, fData.depth, position, boundaryNormalHash, NULL, nonLinearFit);
                  mesh->inCorePoints.push_back (position);
                  boundaryRoots[key] = int (mesh->inCorePoints.size ()) - 1;
                  count++;
                }
              }
              else
              {
                if (interiorRoots->find (key) == interiorRoots->end ())
                {
                  GetRoot (ri, isoValue, fData.depth, position, *interiorNormalHash, NULL, nonLinearFit);
                  (*interiorRoots)[key] = mesh->addOutOfCorePoint (position);
                  interiorPositions->push_back (position);
                  count++;
                }
              }
            }
          }
        }
      }
      return count;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::SetBoundaryMCRootPositions (
        const int& sDepth, const Real& isoValue, hash_map<long long, int>& boundaryRoots, hash_map<long long, std::pair<Real, Point3D<Real> > >& boundaryNormalHash,
                                  CoredMeshData* mesh, const int& nonLinearFit)
    {
      Point3D<Real> position;
      int i, j, k, eIndex, hits = 0;
      RootInfo ri;
      int count = 0;
      TreeOctNode* node;

      node = tree.nextLeaf ();
      while (node)
      {
        if (MarchingCubes::HasRoots (node->nodeData.mcIndex))
        {
          hits = 0;
          for (i = 0; i < DIMENSION; i++)
          {
            for (j = 0; j < 2; j++)
            {
              for (k = 0; k < 2; k++)
              {
                if (IsBoundaryEdge (node, i, j, k, sDepth))
                {
                  hits++;
                  long long key;
                  eIndex = Cube::EdgeIndex (i, j, k);
                  if (GetRootIndex (node, eIndex, fData.depth, ri))
                  {
                    key = ri.key;
                    if (boundaryRoots.find (key) == boundaryRoots.end ())
                    {
                      GetRoot (ri, isoValue, fData.depth, position, boundaryNormalHash, NULL, nonLinearFit);
                      mesh->inCorePoints.push_back (position);
                      boundaryRoots[key] = int (mesh->inCorePoints.size ()) - 1;
                      count++;
                    }
                  }
                }
              }
            }
          }
        }
        if (hits)
        {
          node = tree.nextLeaf (node);
        }
        else
        {
          node = tree.nextBranch (node);
        }
      }
      return count;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> void 
    Octree<Degree>::GetMCIsoEdges (TreeOctNode* node, hash_map<long long, int>& /*boundaryRoots*/, hash_map<long long, int>* /*interiorRoots*/, const int& sDepth,
                                   std::vector<std::pair<long long, long long> >& edges)
    {
      TreeOctNode* temp;
      int count = 0; //,tris = 0;
      int isoTri[DIMENSION * MarchingCubes::MAX_TRIANGLES];
      FaceEdgesFunction fef;
      int ref, fIndex;
      hash_map<long long, std::pair<RootInfo, int> >::iterator iter;
      hash_map<long long, std::pair<RootInfo, int> > vertexCount;

      fef.edges = &edges;
      fef.maxDepth = fData.depth;
      fef.vertexCount = &vertexCount;
      count = MarchingCubes::AddTriangleIndices (node->nodeData.mcIndex, isoTri);
      for (fIndex = 0; fIndex < Cube::NEIGHBORS; fIndex++)
      {
        ref = Cube::FaceReflectFaceIndex (fIndex, fIndex);
        fef.fIndex = ref;
        temp = node->faceNeighbor (fIndex);
        // If the face neighbor exists and has higher resolution than the current node,
        // get the iso-curve from the neighbor
        if (temp && temp->children && !IsBoundaryFace (node, fIndex, sDepth))
        {
          temp->processNodeFaces (temp, &fef, ref);
        }
          // Otherwise, get it from the node
        else
        {
          RootInfo ri1, ri2;
          for (int j = 0; j < count; j++)
          {
            for (int k = 0; k < 3; k++)
            {
              if (fIndex == Cube::FaceAdjacentToEdges (isoTri[j * 3 + k], isoTri[j * 3 + ((k + 1) % 3)]))
              {
                if (GetRootIndex (node, isoTri[j * 3 + k], fData.depth, ri1) && GetRootIndex (node, isoTri[j * 3 + ((k + 1) % 3)], fData.depth, ri2))
                {
                  edges.push_back (std::pair<long long, long long> (ri1.key, ri2.key));
                  iter = vertexCount.find (ri1.key);
                  if (iter == vertexCount.end ())
                  {
                    vertexCount[ri1.key].first = ri1;
                    vertexCount[ri1.key].second = 0;
                  }
                  iter = vertexCount.find (ri2.key);
                  if (iter == vertexCount.end ())
                  {
                    vertexCount[ri2.key].first = ri2;
                    vertexCount[ri2.key].second = 0;
                  }
                  vertexCount[ri1.key].second++;
                  vertexCount[ri2.key].second--;
                }
                else
                {
                  fprintf (stderr, "Bad Edge 1: %d %d\n", int (ri1.key), int (ri2.key));
                }
              }
            }
          }
        }
      }
      for (int i = 0; i<int (edges.size ()); i++)
      {
        iter = vertexCount.find (edges[i].first);
        if (iter == vertexCount.end ())
          std::cerr << "Could not find vertex: " << edges[i].first << std::endl;
        else if (vertexCount[edges[i].first].second)
        {
          RootInfo ri;
          GetRootPair (vertexCount[edges[i].first].first, fData.depth, ri);
          iter = vertexCount.find (ri.key);
          if (iter == vertexCount.end ())
          {
            printf ("Vertex pair not in list\n");
          }
          else
          {
            edges.push_back (std::pair<long long, long long> (ri.key, edges[i].first));
            vertexCount[ri.key].second++;
            vertexCount[edges[i].first].second--;
          }
        }

        iter = vertexCount.find (edges[i].second);
        if (iter == vertexCount.end ())
          std::cerr << "Could not find vertex: " << edges[i].second << std::endl;
        else if (vertexCount[edges[i].second].second)
        {
          RootInfo ri;
          GetRootPair (vertexCount[edges[i].second].first, fData.depth, ri);
          iter = vertexCount.find (ri.key);
          if (iter == vertexCount.end ())
          {
            printf ("Vertex pair not in list\n");
          }
          else
          {
            edges.push_back (std::pair<long long, long long> (edges[i].second, ri.key));
            vertexCount[edges[i].second].second++;
            vertexCount[ri.key].second--;
          }
        }
      }
    }

    template<int Degree>
    int Octree<Degree>
    ::GetMCIsoTriangles (TreeOctNode* node, CoredMeshData* mesh, hash_map<long long, int>& boundaryRoots,
                         hash_map<long long, int>* interiorRoots, std::vector<Point3D<float> >* interiorPositions, const int& offSet, const int& sDepth, bool addBarycenter, bool polygonMesh)
    {
      int tris = 0;
      std::vector<std::pair<long long, long long> > edges;
      std::vector<std::vector<std::pair<long long, long long> > > edgeLoops;
      GetMCIsoEdges (node, boundaryRoots, interiorRoots, sDepth, edges);

      GetEdgeLoops (edges, edgeLoops);
      for (int i = 0; i<int (edgeLoops.size ()); i++)
      {
        CoredPointIndex p;
        std::vector<CoredPointIndex> edgeIndices;
        for (int j = 0; j<int (edgeLoops[i].size ()); j++)
        {
          if (!GetRootIndex (edgeLoops[i][j].first, boundaryRoots, interiorRoots, p))
          {
            printf ("Bad Point Index\n");
          }
          else
          {
            edgeIndices.push_back (p);
          }
        }
        tris += AddTriangles (mesh, edgeIndices, interiorPositions, offSet, addBarycenter, polygonMesh);
      }
      return tris;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::GetEdgeLoops (std::vector<std::pair<long long, long long> >& edges, std::vector<std::vector<std::pair<long long, long long> > >& loops)
    {
      int loopSize = 0;
      long long frontIdx, backIdx;
      std::pair<long long, long long> e, temp;
      loops.clear ();

      while (edges.size ())
      {
        std::vector<std::pair<long long, long long> > front, back;
        e = edges[0];
        loops.resize (loopSize + 1);
        edges[0] = edges[edges.size () - 1];
        edges.pop_back ();
        frontIdx = e.second;
        backIdx = e.first;
        for (int j = int (edges.size ()) - 1; j >= 0; j--)
        {
          if (edges[j].first == frontIdx || edges[j].second == frontIdx)
          {
            if (edges[j].first == frontIdx)
            {
              temp = edges[j];
            }
            else
            {
              temp.first = edges[j].second;
              temp.second = edges[j].first;
            }
            frontIdx = temp.second;
            front.push_back (temp);
            edges[j] = edges[edges.size () - 1];
            edges.pop_back ();
            j = int (edges.size ());
          }
          else if (edges[j].first == backIdx || edges[j].second == backIdx)
          {
            if (edges[j].second == backIdx)
            {
              temp = edges[j];
            }
            else
            {
              temp.first = edges[j].second;
              temp.second = edges[j].first;
            }
            backIdx = temp.first;
            back.push_back (temp);
            edges[j] = edges[edges.size () - 1];
            edges.pop_back ();
            j = int (edges.size ());
          }
        }
        for (int j = int (back.size ()) - 1; j >= 0; j--)
        {
          loops[loopSize].push_back (back[j]);
        }
        loops[loopSize].push_back (e);
        for (int j = 0; j<int (front.size ()); j++)
        {
          loops[loopSize].push_back (front[j]);
        }
        loopSize++;
      }
      return int (loops.size ());
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::AddTriangles (CoredMeshData* mesh, std::vector<CoredPointIndex> edges[3], std::vector<Point3D<float> >* interiorPositions, const int& offSet)
    {
      std::vector<CoredPointIndex> e;
      for (int i = 0; i < 3; i++)
      {
        for (size_t j = 0; j < edges[i].size (); j++)
        {
          e.push_back (edges[i][j]);
        }
      }
      return AddTriangles (mesh, e, interiorPositions, offSet);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<int Degree> int 
    Octree<Degree>::AddTriangles (CoredMeshData* mesh, std::vector<CoredPointIndex>& edges, std::vector<Point3D<float> >* interiorPositions, const int& offSet, bool addBarycenter, bool polygonMesh)
    {
      if (polygonMesh)
      {
        std::vector< CoredVertexIndex > vertices (edges.size ());
        for (size_t i = 0; i < edges.size (); i++)
        {
          vertices[i].idx = edges[i].index;
          vertices[i].inCore = edges[i].inCore;
        }
        mesh->addPolygon (vertices);
        return 1;
      }
      if (edges.size () > 3)
      {
#if 1
        bool isCoplanar = false;

        for (size_t i = 0; i < edges.size (); i++)
          for (size_t j = 0; j < i; j++)
            if ((i + 1) % edges.size () != j && (j + 1) % edges.size () != i)
            {
              Point3D< Real > v1, v2;
              if (edges[i].inCore) for (int k = 0; k < 3; k++) v1.coords[k] = mesh->inCorePoints[ edges[i].index ].coords[k];
              else for (int k = 0; k < 3; k++) v1.coords[k] = (*interiorPositions)[ edges[i].index - offSet ].coords[k];
              if (edges[j].inCore) for (int k = 0; k < 3; k++) v2.coords[k] = mesh->inCorePoints[ edges[j].index ].coords[k];
              else for (int k = 0; k < 3; k++) v2.coords[k] = (*interiorPositions)[ edges[j].index - offSet ].coords[k];
              for (int k = 0; k < 3; k++) if (v1.coords[k] == v2.coords[k]) isCoplanar = true;
            }
        if (addBarycenter && isCoplanar)
#else
        if (addBarycenter)
#endif
        {
          Point3D< Real > c;
          c.coords[0] = c.coords[1] = c.coords[2] = 0;
          for (int i = 0; i<int (edges.size ()); i++)
          {
            Point3D<Real> p;
            if (edges[i].inCore) for (int j = 0; j < 3; j++) p.coords[j] = mesh->inCorePoints[edges[i].index].coords[j];
            else for (int j = 0; j < 3; j++) p.coords[j] = (*interiorPositions)[edges[i].index - offSet].coords[j];
            c.coords[0] += p.coords[0], c.coords[1] += p.coords[1], c.coords[2] += p.coords[2];
          }
          c.coords[0] /= static_cast<Real> (edges.size ());
          c.coords[1] /= static_cast<Real> (edges.size ()); 
          c.coords[2] /= static_cast<Real> (edges.size ());
          int cIdx = mesh->addOutOfCorePoint (c);
          for (int i = 0; i<int (edges.size ()); i++)
          {
            std::vector< CoredVertexIndex > vertices (3);
            vertices[0].idx = edges[i].index;
            vertices[1].idx = edges[ (i + 1) % edges.size ()].index;
            vertices[2].idx = cIdx;
            vertices[0].inCore = edges[i ].inCore;
            vertices[1].inCore = edges[ (i + 1) % edges.size ()].inCore;
            vertices[2].inCore = 0;
            mesh->addPolygon (vertices);
          }
          return (static_cast<int> (edges.size ()));
        }
        else
        {
          Triangulation<float> t;

          // Add the points to the triangulation
          for (int i = 0; i<int (edges.size ()); i++)
          {
            Point3D<Real> p;
            if (edges[i].inCore)
            {
              for (int j = 0; j < 3; j++)
              {
                p.coords[j] = mesh->inCorePoints[edges[i].index].coords[j];
              }
            }
            else
            {
              for (int j = 0; j < 3; j++)
              {
                p.coords[j] = (*interiorPositions)[edges[i].index - offSet].coords[j];
              }
            }
            t.points.push_back (p);
          }

          // Create a fan triangulation
          for (int i = 1; i<int (edges.size ()) - 1; i++)
          {
            t.addTriangle (0, i, i + 1);
          }

          // Minimize
          while (1)
          {
            size_t i;
            for (i = 0; i < t.edges.size (); i++)
            {
              if (t.flipMinimize (static_cast<int> (i)))
                break;
            }
            if (i == t.edges.size ())
              break;
          }
          // Add the triangles to the mesh
          for (int i = 0; i<int (t.triangles.size ()); i++)
          {
            std::vector< CoredVertexIndex > vertices (3);
            int idx[3];
            t.factor (i, idx[0], idx[1], idx[2]);
            for (int j = 0; j < 3; j++)
            {
              vertices[j].idx = edges[ idx[j] ].index;
              vertices[j].inCore = edges[ idx[j] ].inCore;
            }
            mesh->addPolygon (vertices);
          }
        }
      }
      else if (edges.size () == 3)
      {
        std::vector< CoredVertexIndex > vertices (3);
        for (int i = 0; i < 3; i++)
        {
          vertices[i].idx = edges[i].index;
          vertices[i].inCore = edges[i].inCore;
        }
        mesh->addPolygon (vertices);
      }
      return int (edges.size ()) - 2;
    }
    ////////////////
    // VertexData //
    ////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::CenterIndex (const TreeOctNode* node, const int& maxDepth)
    {
      int idx[DIMENSION];
      return CenterIndex (node, maxDepth, idx);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::CenterIndex (const TreeOctNode* node, const int& maxDepth, int idx[DIMENSION])
    {
      int d, o[3];
      node->depthAndOffset (d, o);
      for (int i = 0; i < DIMENSION; i++)
      {
        idx[i] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d + 1, o[i] << 1, 1);
      }
      return (static_cast<long long> (idx[0]) | static_cast<long long> (idx[1]) << 15 | static_cast<long long> (idx[2]) << 30);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::CenterIndex (const int& depth, const int offSet[DIMENSION], const int& maxDepth, int idx[DIMENSION])
    {
      for (int i = 0; i < DIMENSION; i++)
      {
        idx[i] = BinaryNode<Real>::CornerIndex (maxDepth + 1, depth + 1, offSet[i] << 1, 1);
      }
      return (static_cast<long long> (idx[0]) | static_cast<long long> (idx[1]) << 15 | static_cast<long long> (idx[2]) << 30);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::CornerIndex (const TreeOctNode* node, const int& cIndex, const int& maxDepth)
    {
      int idx[DIMENSION];
      return CornerIndex (node, cIndex, maxDepth, idx);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::CornerIndex (const TreeOctNode* node, const int& cIndex, const int& maxDepth, int idx[DIMENSION])
    {
      int x[DIMENSION];
      Cube::FactorCornerIndex (cIndex, x[0], x[1], x[2]);
      int d, o[3];
      node->depthAndOffset (d, o);
      for (int i = 0; i < DIMENSION; i++)
      {
        idx[i] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, o[i], x[i]);
      }
      return (static_cast<long long> (idx[0]) | static_cast<long long> (idx[1]) << 15 | static_cast<long long> (idx[2]) << 30);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::CornerIndex (const int& depth, const int offSet[DIMENSION], const int& cIndex, const int& maxDepth, int idx[DIMENSION])
    {
      int x[DIMENSION];
      Cube::FactorCornerIndex (cIndex, x[0], x[1], x[2]);
      for (int i = 0; i < DIMENSION; i++)
      {
        idx[i] = BinaryNode<Real>::CornerIndex (maxDepth + 1, depth, offSet[i], x[i]);
      }
      return (static_cast<long long> (idx[0]) | static_cast<long long> (idx[1]) << 15 | static_cast<long long> (idx[2]) << 30);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::FaceIndex (const TreeOctNode* node, const int& fIndex, const int& maxDepth)
    {
      int idx[DIMENSION];
      return FaceIndex (node, fIndex, maxDepth, idx);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::FaceIndex (const TreeOctNode* node, const int& fIndex, const int& maxDepth, int idx[DIMENSION])
    {
      int dir, offset;
      Cube::FactorFaceIndex (fIndex, dir, offset);
      int d, o[3];
      node->depthAndOffset (d, o);
      for (int i = 0; i < DIMENSION; i++)
      {
        idx[i] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d + 1, o[i] << 1, 1);
      }
      idx[dir] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, o[dir], offset);
      return (static_cast<long long> (idx[0]) | static_cast<long long> (idx[1]) << 15 | static_cast<long long> (idx[2]) << 30);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::EdgeIndex (const TreeOctNode* node, const int& eIndex, const int& maxDepth)
    {
      int idx[DIMENSION];
      return EdgeIndex (node, eIndex, maxDepth, idx);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    long long
    VertexData::EdgeIndex (const TreeOctNode* node, const int& eIndex, const int& maxDepth, int idx[DIMENSION])
    {
      int o, i1, i2;
      int d, off[3];
      node->depthAndOffset (d, off);
      for (int i = 0; i < DIMENSION; i++)
      {
        idx[i] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d + 1, off[i] << 1, 1);
      }
      Cube::FactorEdgeIndex (eIndex, o, i1, i2);
      switch (o)
      {
        case 0:
          idx[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[1], i1);
          idx[2] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[2], i2);
          break;
        case 1:
          idx[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[0], i1);
          idx[2] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[2], i2);
          break;
        case 2:
          idx[0] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[0], i1);
          idx[1] = BinaryNode<Real>::CornerIndex (maxDepth + 1, d, off[1], i2);
          break;
      };
      return (static_cast<long long> (idx[0]) | static_cast<long long> (idx[1]) << 15 | static_cast<long long> (idx[2]) << 30);
    }
  }
}

