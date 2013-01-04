##
#  Software License Agreement (BSD License)
#
#  Point Cloud Library (PCL) - www.pointclouds.org
#  Copyright (c) 2009-2012, Willow Garage, Inc.
#  Copyright (c) 2012-, Open Perception, Inc.
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  #  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  #  Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  #  Neither the name of the copyright holder(s) nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

import os

filename    = os.path.join (os.path.dirname (__file__), 'mesh_indices.h')
class_names = ['VertexIndex', 'HalfEdgeIndex', 'EdgeIndex', 'FaceIndex']

################################################################################

f = open (filename, 'w')

f.write ('/*\n')
f.write (' * Software License Agreement (BSD License)\n')
f.write (' *\n')
f.write (' * Point Cloud Library (PCL) - www.pointclouds.org\n')
f.write (' * Copyright (c) 2009-2012, Willow Garage, Inc.\n')
f.write (' * Copyright (c) 2012-, Open Perception, Inc.\n')
f.write (' *\n')
f.write (' * All rights reserved.\n')
f.write (' *\n')
f.write (' * Redistribution and use in source and binary forms, with or without\n')
f.write (' * modification, are permitted provided that the following conditions\n')
f.write (' * are met:\n')
f.write (' *\n')
f.write (' *  * Redistributions of source code must retain the above copyright\n')
f.write (' *    notice, this list of conditions and the following disclaimer.\n')
f.write (' *  * Redistributions in binary form must reproduce the above\n')
f.write (' *    copyright notice, this list of conditions and the following\n')
f.write (' *    disclaimer in the documentation and/or other materials provided\n')
f.write (' *    with the distribution.\n')
f.write (' *  * Neither the name of the copyright holder(s) nor the names of its\n')
f.write (' *    contributors may be used to endorse or promote products derived\n')
f.write (' *    from this software without specific prior written permission.\n')
f.write (' *\n')
f.write (' * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n')
f.write (' * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n')
f.write (' * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS\n')
f.write (' * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE\n')
f.write (' * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,\n')
f.write (' * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,\n')
f.write (' * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;\n')
f.write (' * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER\n')
f.write (' * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT\n')
f.write (' * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN\n')
f.write (' * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE\n')
f.write (' * POSSIBILITY OF SUCH DAMAGE.\n')
f.write (' *\n')
f.write (' * $Id$\n')
f.write (' *\n')
f.write (' */\n\n')

f.write ("// NOTE: This file has been created with 'pcl_src/geometry/include/pcl/geometry/mesh_indices.py'\n\n")

f.write ('#ifndef PCL_GEOMETRY_MESH_INDICES_H\n')
f.write ('#define PCL_GEOMETRY_MESH_INDICES_H\n\n')

f.write ('#include <iostream>\n\n')

f.write ('#include <pcl/geometry/boost.h>\n\n')

for cn in class_names:

    f.write ('////////////////////////////////////////////////////////////////////////////////\n')
    f.write ('// ' + cn + '\n')
    f.write ('////////////////////////////////////////////////////////////////////////////////\n\n')

    f.write ('namespace pcl\n')
    f.write ('{\n')
    f.write ('  namespace geometry\n')
    f.write ('  {\n')
    f.write ('    /** \\brief Index used to access elements in the half-edge mesh. It is basically just a wrapper around an integer with a few added methods.\n')
    f.write ('      * \\author Martin Saelzle\n')
    f.write ('      * \ingroup geometry\n')
    f.write ('      */\n')
    f.write ('    class ' + cn + '\n')
    f.write ('        : boost::totally_ordered <pcl::geometry::' + cn + ' // < > <= >= == !=\n')
    f.write ('        , boost::unit_steppable  <pcl::geometry::' + cn + ' // ++ -- (pre and post)\n')
    f.write ('        , boost::additive        <pcl::geometry::' + cn + ' // += + -= -\n')
    f.write ('        > > >\n')
    f.write ('    {\n')
    f.write ('      public:\n\n')

    f.write ('        typedef boost::totally_ordered <pcl::geometry::' + cn + ',\n')
    f.write ('                boost::unit_steppable  <pcl::geometry::' + cn + ',\n')
    f.write ('                boost::additive        <pcl::geometry::' + cn + '> > > Base;\n')
    f.write ('        typedef pcl::geometry::' + cn + '                              Self;\n\n')

    f.write ('        /** \\brief Constructor. Initializes with an invalid index. */\n')
    f.write ('        ' + cn + ' ()\n')
    f.write ('          : index_ (-1)\n')
    f.write ('        {\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Constructor.\n')
    f.write ('          * \param[in] index The integer index.\n')
    f.write ('          */\n')
    f.write ('        explicit ' + cn + ' (const int index)\n')
    f.write ('          : index_ (index)\n')
    f.write ('        {\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Returns true if the index is valid. */\n')
    f.write ('        inline bool\n')
    f.write ('        isValid () const\n')
    f.write ('        {\n')
    f.write ('          return (index_ >= 0);\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Invalidate the index. */\n')
    f.write ('        inline void\n')
    f.write ('        invalidate ()\n')
    f.write ('        {\n')
    f.write ('          index_ = -1;\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Get the index. */\n')
    f.write ('        inline int\n')
    f.write ('        get () const\n')
    f.write ('        {\n')
    f.write ('          return (index_);\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Set the index. */\n')
    f.write ('        inline void\n')
    f.write ('        set (const int index)\n')
    f.write ('        {\n')
    f.write ('          index_ = index;\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Comparison operators (with boost::operators): < > <= >= */\n')
    f.write ('        inline bool\n')
    f.write ('        operator < (const Self& other) const\n')
    f.write ('        {\n')
    f.write ('          return (this->get () < other.get ());\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Comparison operators (with boost::operators): == != */\n')
    f.write ('        inline bool\n')
    f.write ('        operator == (const Self& other) const\n')
    f.write ('        {\n')
    f.write ('          return (this->get () == other.get ());\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Increment operators (with boost::operators): ++ (pre and post) */\n')
    f.write ('        inline Self&\n')
    f.write ('        operator ++ ()\n')
    f.write ('        {\n')
    f.write ('          ++index_;\n')
    f.write ('          return (*this);\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Decrement operators (with boost::operators): \-\- (pre and post) */\n')
    f.write ('        inline Self&\n')
    f.write ('        operator -- ()\n')
    f.write ('        {\n')
    f.write ('          --index_;\n')
    f.write ('          return (*this);\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Addition operators (with boost::operators): + += */\n')
    f.write ('        inline Self&\n')
    f.write ('        operator += (const Self& other)\n')
    f.write ('        {\n')
    f.write ('          index_ += other.get ();\n')
    f.write ('          return (*this);\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Subtraction operators (with boost::operators): - -= */\n')
    f.write ('        inline Self&\n')
    f.write ('        operator -= (const Self& other)\n')
    f.write ('        {\n')
    f.write ('          index_ -= other.get ();\n')
    f.write ('          return (*this);\n')
    f.write ('        }\n\n')

    f.write ('      private:\n\n')

    f.write ('        /** \\brief Stored index. */\n')
    f.write ('        int index_;\n\n')

    f.write ('        friend std::istream&\n')
    f.write ('        operator >> (std::istream& is, pcl::geometry::' + cn + '& index);\n')
    f.write ('    };\n\n')

    f.write ('    /** \\brief ostream operator. */\n')
    f.write ('    inline std::ostream&\n')
    f.write ('    operator << (std::ostream& os, const pcl::geometry::' + cn + '& index)\n')
    f.write ('    {\n')
    f.write ('      return (os << index.get ());\n')
    f.write ('    }\n\n')

    f.write ('    /** \\brief istream operator. */\n')
    f.write ('    inline std::istream&\n')
    f.write ('    operator >> (std::istream& is, pcl::geometry::' + cn + '& index)\n')
    f.write ('    {\n')
    f.write ('      return (is >> index.index_);\n')
    f.write ('    }\n\n')

    f.write ('  } // End namespace geometry\n')
    f.write ('} // End namespace pcl\n\n')

f.write ('////////////////////////////////////////////////////////////////////////////////\n')
f.write ('// Conversions\n')
f.write ('////////////////////////////////////////////////////////////////////////////////\n\n')

f.write ('namespace pcl\n')
f.write ('{\n')
f.write ('  namespace geometry\n')
f.write ('  {\n')
f.write ('    /** \\brief Convert the given half-edge index to an edge index. */\n')
f.write ('    inline pcl::geometry::EdgeIndex\n')
f.write ('    toEdgeIndex (const HalfEdgeIndex& index)\n')
f.write ('    {\n')
f.write ('      return (index.isValid () ? EdgeIndex (index.get () / 2) : EdgeIndex ());\n')
f.write ('    }\n\n')

f.write ('    /** \\brief Convert the given edge index to a half-edge index.\n')
f.write ('      * \\param[in] get_first The first half-edge of the edge is returned if this variable is true; elsewise the second.\n')
f.write ('      */\n')
f.write ('    inline pcl::geometry::HalfEdgeIndex\n')
f.write ('    toHalfEdgeIndex (const EdgeIndex& index, const bool get_first=true)\n')
f.write ('    {\n')
f.write ('      return (index.isValid () ? HalfEdgeIndex (index.get () * 2 + static_cast <int> (!get_first)) : HalfEdgeIndex ());\n')
f.write ('    }\n')
f.write ('  } // End namespace geometry\n')
f.write ('} // End namespace pcl\n\n')

f.write ('#endif // PCL_GEOMETRY_MESH_INDICES_H\n')

f.close()
