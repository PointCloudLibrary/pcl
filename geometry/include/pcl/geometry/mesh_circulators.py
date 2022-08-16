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

filename  = os.path.join (os.path.dirname (__file__), 'mesh_circulators.h')

class Class:
    def __init__ (self, value_prefix, value_type, around_type, current_he, deref, inc1, inc2, dec1, dec2):
        self.value_prefix  = value_prefix
        self.value_type  = value_type
        self.around_type = around_type
        self.current_he  = current_he
        self.deref       = deref
        self.inc1        = inc1
        self.inc2        = inc2
        self.dec1        = dec1
        self.dec2        = dec2
        self.docstring   = 'TODO: Add documentation!'

classes = []
classes.append (Class (''        , 'Vertex'  , 'Vertex', 'Outgoing', 'TerminatingVertex', 'Opposite', 'Next'    , 'Prev'    , 'Opposite')) # 0
classes.append (Class ('Outgoing', 'HalfEdge', 'Vertex', 'Outgoing', ''                 , 'Opposite', 'Next'    , 'Prev'    , 'Opposite')) # 1
classes.append (Class ('Incoming', 'HalfEdge', 'Vertex', 'Incoming', ''                 , 'Next'    , 'Opposite', 'Opposite', 'Prev'    )) # 2
classes.append (Class (''        , 'Face'    , 'Vertex', 'Outgoing', 'Face'             , 'Opposite', 'Next'    , 'Prev'    , 'Opposite')) # 3
classes.append (Class (''        , 'Vertex'  , 'Face'  , 'Inner'   , 'TerminatingVertex', 'Next'    , ''        , 'Prev'    , ''        )) # 4
classes.append (Class ('Inner'   , 'HalfEdge', 'Face'  , 'Inner'   , ''                 , 'Next'    , ''        , 'Prev'    , ''        )) # 5
classes.append (Class ('Outer'   , 'HalfEdge', 'Face'  , 'Inner'   , 'OppositeHalfEdge' , 'Next'    , ''        , 'Prev'    , ''        )) # 6
classes.append (Class (''        , 'Face'    , 'Face'  , 'Inner'   , 'OppositeFace'     , 'Next'    , ''        , 'Prev'    , ''        )) # 7

classes [0].docstring = 'Circulates counter-clockwise around a vertex and returns an index to the terminating vertex of the outgoing half-edge (the target).'
classes [1].docstring = 'Circulates counter-clockwise around a vertex and returns an index to the outgoing half-edge (the target).'
classes [2].docstring = 'Circulates counter-clockwise around a vertex and returns an index to the incoming half-edge (the target).'
classes [3].docstring = 'Circulates counter-clockwise around a vertex and returns an index to the face of the outgoing half-edge (the target).'
classes [4].docstring = 'Circulates clockwise around a face and returns an index to the terminating vertex of the inner half-edge (the target).'
classes [5].docstring = 'Circulates clockwise around a face and returns an index to the inner half-edge (the target).'
classes [6].docstring = 'Circulates clockwise around a face and returns an index to the outer half-edge (the target).'
classes [7].docstring = 'Circulates clockwise around a face and returns an index to the face of the outer half-edge (the target).'

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

f.write ("// NOTE: This file has been created with 'pcl_src/geometry/include/pcl/geometry/mesh_circulators.py'\n\n")

f.write ('#ifndef PCL_GEOMETRY_MESH_CIRCULATORS_H\n')
f.write ('#define PCL_GEOMETRY_MESH_CIRCULATORS_H\n\n')

f.write ('#include <pcl/geometry/mesh_indices.h>\n\n')
f.write ('#include <boost/operators.hpp>\n\n')

for c in classes:

    value_prefix    = c.value_prefix
    value_name      = c.value_prefix + c.value_type
    value_type      = c.value_type
    around_type     = c.around_type
    around_obj      = around_type.lower ()
    around_idx      = 'idx_' + around_obj
    current_he      = c.current_he
    current_he_idx  = 'idx_' + current_he.lower () + '_half_edge'
    current_he_idx_ = current_he_idx + '_'
    deref           = c.deref
    inc1            = c.inc1
    inc2            = c.inc2
    dec1            = c.dec1
    dec2            = c.dec2

    class_name      = value_name + 'Around' + around_type + 'Circulator'

    placeholder_at  = ' ' * (len (around_type) - 3)
    placeholder_cn  = ' ' *  len (class_name)
    placeholder_gt  = ' ' * (len (current_he_idx) - 5)

    f.write ('////////////////////////////////////////////////////////////////////////////////\n')
    f.write ('// ' + class_name + '\n')
    f.write ('////////////////////////////////////////////////////////////////////////////////\n\n')

    f.write ('namespace pcl\n')
    f.write ('{\n')
    f.write ('  namespace geometry\n')
    f.write ('  {\n')

    f.write ('    /** \\brief ' + c.docstring + ' The best way to declare the circulator is to use the method pcl::geometry::MeshBase::get' + class_name + ' ().\n')
    f.write ("      * \\tparam MeshT Mesh to which this circulator belongs to.\n")
    f.write ("      * \\note The circulator can't be used to change the connectivity in the mesh (only const circulators are valid).\n")
    f.write ('      * \\author Martin Saelzle\n')
    f.write ('      * \ingroup geometry\n')
    f.write ('      */\n')

    f.write ('    template <class MeshT>\n')
    f.write ('    class ' + class_name + '\n')
    f.write ('        : boost::equality_comparable <pcl::geometry::' + class_name + ' <MeshT>\n')
    f.write ('        , boost::unit_steppable      <pcl::geometry::' + class_name + ' <MeshT>\n')
    f.write ('        > >\n')
    f.write ('    {\n')
    f.write ('      public:\n\n')

    f.write ('        typedef boost::equality_comparable <pcl::geometry::' + class_name + ' <MeshT>\n')
    f.write ('              , boost::unit_steppable      <pcl::geometry::' + class_name + ' <MeshT> > > Base;\n')
    f.write ('        typedef pcl::geometry::' + class_name + ' <MeshT> Self;\n\n')

    f.write ('        typedef MeshT Mesh;\n')
    if value_type != 'HalfEdge':
        f.write ('        typedef typename Mesh::' + value_type + 'Index ' + value_type + 'Index;\n')
    if around_type != value_type:
        f.write ('        typedef typename Mesh::' + around_type + 'Index ' + around_type + 'Index;\n')
    f.write ('        typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;\n\n')

    f.write ('        /** \\brief Constructor resulting in an invalid circulator. */\n')
    f.write ('        ' + class_name + ' ()\n')
    f.write ('          : mesh_ ' + placeholder_gt + ' (NULL),\n')
    f.write ('            '       + current_he_idx_ + ' ()\n')
    f.write ('        {\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Construct from the ' + around_obj + ' around which we want to circulate. */\n')
    f.write ('        ' + class_name     + ' (const '      + around_type    + 'Index& ' + around_idx + ',\n')
    f.write ('        ' + placeholder_cn + '  Mesh*const ' + placeholder_at + '     '   + 'mesh)\n')
    f.write ('          : mesh_ ' + placeholder_gt + ' (mesh),\n')
    f.write ('            '       + current_he_idx_ + ' (mesh->get' + current_he + 'HalfEdgeIndex (' + around_idx + '))\n')
    f.write ('        {\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Construct directly from the ' + current_he.lower () + ' half-edge. */\n')
    f.write ('        ' + class_name     + ' (const HalfEdgeIndex& ' + current_he_idx  + ',\n')
    f.write ('        ' + placeholder_cn + '  Mesh*const           ' + 'mesh)\n')
    f.write ('          : mesh_ ' + placeholder_gt + ' (mesh),\n')
    f.write ('            '       + current_he_idx_ + ' (' + current_he_idx + ')\n')
    f.write ('        {\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Check if the circulator is valid.\n')
    f.write ('          * \\warning Does NOT check if the stored mesh pointer is valid. You have to ensure this yourself when constructing the circulator. */\n')
    f.write ('        inline bool\n')
    f.write ('        isValid () const\n')
    f.write ('        {\n')
    f.write ('          return (' + current_he_idx_ + '.isValid ());\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Comparison operators (with boost::operators): == !=\n')
    f.write ('          * \\warning Does NOT check if the circulators belong to the same mesh. Please check this yourself. */\n')
    f.write ('        inline bool\n')
    f.write ('        operator == (const Self& other) const\n')
    f.write ('        {\n')
    f.write ('          return (' + current_he_idx_ + ' == other.' + current_he_idx_ + ');\n')
    f.write ('        }\n\n')

    tmp = 'mesh_->get' + inc1 + 'HalfEdgeIndex (' + current_he_idx_ + ')'
    if inc2:
        tmp = 'mesh_->get' + inc2 + 'HalfEdgeIndex (' + tmp + ')'

    f.write ('        /** \\brief Increment operators (with boost::operators): ++ (pre and post) */\n')
    f.write ('        inline Self&\n')
    f.write ('        operator ++ ()\n')
    f.write ('        {\n')
    f.write ('          ' + current_he_idx_ + ' = ' + tmp + ';\n')
    f.write ('          return (*this);\n')
    f.write ('        }\n\n')

    tmp = 'mesh_->get' + dec1 + 'HalfEdgeIndex (' + current_he_idx_ + ')'
    if dec2:
        tmp = 'mesh_->get' + dec2 + 'HalfEdgeIndex (' + tmp + ')'

    f.write ('        /** \\brief Decrement operators (with boost::operators): -- (pre and post) */\n')
    f.write ('        inline Self&\n')
    f.write ('        operator -- ()\n')
    f.write ('        {\n')
    f.write ('          ' + current_he_idx_ + ' = ' + tmp + ';\n')
    f.write ('          return (*this);\n')
    f.write ('        }\n\n')

    if deref:
        tmp = 'mesh_->get' + deref + 'Index (' + current_he_idx_ + ')'
    else:
        tmp = current_he_idx_
    tgt = 'half-edge' if value_type=='HalfEdge' else value_type.lower ()
    tgt = value_prefix.lower () + (' ' if value_prefix else 'target ')  + tgt

    f.write ('        /** \\brief Get the index to the ' + tgt + '. */\n')
    f.write ('        inline ' + value_type + 'Index\n')
    f.write ('        getTargetIndex () const\n')
    f.write ('        {\n')
    f.write ('          return (' + tmp + ');\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief Get the half-edge that is currently stored in the circulator. */\n')
    f.write ('        inline HalfEdgeIndex\n')
    f.write ('        getCurrentHalfEdgeIndex () const\n')
    f.write ('        {\n')
    f.write ('          return (' + current_he_idx_ + ');\n')
    f.write ('        }\n\n')

    f.write ('        /** \\brief The mesh to which this circulator belongs to. */\n')
    f.write ('        Mesh* mesh_;\n\n')

    f.write ('        /** \\brief The ' + current_he.lower () + ' half-edge of the ' + around_obj + ' around which we want to circulate. */\n')
    f.write ('        HalfEdgeIndex ' + current_he_idx_ + ';\n')
    f.write ('    };\n')

    f.write ('  } // End namespace geometry\n')
    f.write ('} // End namespace pcl\n\n')

f.write ('#endif // PCL_GEOMETRY_MESH_CIRCULATORS_H\n')

f.close()
