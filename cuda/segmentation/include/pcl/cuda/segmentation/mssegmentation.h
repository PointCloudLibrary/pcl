// this file is extracted from OpenCV/modules/gpu/src/mssegmentation.cpp
// to get access to the intermediate results of meanShiftSegmentation()
// minor changes to compile correctly with pcl::cuda and namespace reorganization

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#pragma once

#include <pcl/pcl_exports.h>

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>

namespace pcl
{
namespace cuda
{
namespace detail
{

//
// Declarations
//

class DjSets
{
public:
    DjSets(int n);
    int find(int elem);
    int merge(int set1, int set2);

    void init (int n);

    std::vector<int> parent;
    std::vector<int> rank;
    std::vector<int> size;
private:
    DjSets(const DjSets&);
    void operator =(const DjSets&);
};


template <typename T>
struct GraphEdge
{
    GraphEdge() {}
    GraphEdge(int to, int next, const T& val) : to(to), next(next), val(val) {}
    int to;
    int next;
    T val;
};


template <typename T>
class Graph
{
public:
    using Edge = GraphEdge<T>;

    Graph(int numv, int nume_max);

    void addEdge(int from, int to, const T& val=T());

    std::vector<int> start;
    std::vector<Edge> edges;

    int numv;
    int nume_max;
    int nume;
private:
    Graph(const Graph&);
    void operator =(const Graph&);
};


struct SegmLinkVal
{
    SegmLinkVal() {}
    SegmLinkVal(int dr, int dsp) : dr(dr), dsp(dsp) {}
    bool operator <(const SegmLinkVal& other) const
    {
        return dr + dsp < other.dr + other.dsp;
    }
    int dr;
    int dsp;
};


struct SegmLink
{
    SegmLink() {}
    SegmLink(int from, int to, const SegmLinkVal& val)
        : from(from), to(to), val(val) {}
    bool operator <(const SegmLink& other) const
    {
        return val < other.val;
    }
    int from;
    int to;
    SegmLinkVal val;
};

//
// Implementation
//

DjSets::DjSets(int n)
{
    init (n);
}


inline int DjSets::find(int elem)
{
    int set = elem;
    while (set != parent[set])
        set = parent[set];
    while (elem != parent[elem])
    {
        int next = parent[elem];
        parent[elem] = set;
        elem = next;
    }
    return set;
}

inline void DjSets::init(int n)
{
    parent.resize(n);
    rank.resize(n, 0);
    size.resize(n, 1);
    for (int i = 0; i < n; ++i)
        parent[i] = i;
}

inline int DjSets::merge(int set1, int set2)
{
    if (rank[set1] < rank[set2])
    {
        parent[set1] = set2;
        size[set2] += size[set1];
        return set2;
    }
    if (rank[set2] < rank[set1])
    {
        parent[set2] = set1;
        size[set1] += size[set2];
        return set1;
    }
    parent[set1] = set2;
    rank[set2]++;
    size[set2] += size[set1];
    return set2;
}


template <typename T>
Graph<T>::Graph(int numv, int nume_max) : start(numv, -1), edges(nume_max)
{
    this->numv = numv;
    this->nume_max = nume_max;
    nume = 0;
}


template <typename T>
inline void Graph<T>::addEdge(int from, int to, const T& val)
{
    edges[nume] = Edge(to, start[from], val);
    start[from] = nume;
    nume++;
}


inline int pix(int y, int x, int ncols)
{
    return y * ncols + x;
}


inline int sqr(int x)
{
    return x * x;
}


inline int dist2(const cv::Vec4b& lhs, const cv::Vec4b& rhs)
{
    return sqr(lhs[0] - rhs[0]) + sqr(lhs[1] - rhs[1]) + sqr(lhs[2] - rhs[2]);
}


inline int dist2(const cv::Vec2s& lhs, const cv::Vec2s& rhs)
{
    return sqr(lhs[0] - rhs[0]) + sqr(lhs[1] - rhs[1]);
}

} // namespace

PCL_EXPORTS void meanShiftSegmentation(const cv::gpu::GpuMat& src, cv::Mat& dst, int sp, int sr, int minsize, detail::DjSets &comps, cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1) );

} // namespace
} // namespace
