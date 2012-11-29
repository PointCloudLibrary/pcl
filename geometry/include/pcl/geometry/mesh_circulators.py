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
#  $Id$
#

import os

filename  = os.path.join (os.path.dirname (__file__), 'impl', 'mesh_circulators.hpp')
namespace = 'pcl'

class Class:
    def __init__ (self, value_name, value_type, around_type, he_getter, deref, inc1, inc2, dec1, dec2):
        self.value_name  = value_name
        self.value_type  = value_type
        self.around_type = around_type
        self.he_getter   = he_getter
        self.deref       = deref
        self.inc1        = inc1
        self.inc2        = inc2
        self.dec1        = dec1
        self.dec2        = dec2

classes = []
classes.append (Class ('Vertex'          , 'Vertex'  , 'Vertex', 'Outgoing', 'TerminatingVertex', 'Prev'    , 'Opposite', 'Opposite', 'Next'    ))
classes.append (Class ('OutgoingHalfEdge', 'HalfEdge', 'Vertex', 'Outgoing', ''                 , 'Prev'    , 'Opposite', 'Opposite', 'Next'    ))
classes.append (Class ('IncomingHalfEdge', 'HalfEdge', 'Vertex', 'Incoming', ''                 , 'Opposite', 'Prev'    , 'Next'    , 'Opposite'))
classes.append (Class ('Face'            , 'Face'    , 'Vertex', 'Outgoing', 'Face'             , 'Prev'    , 'Opposite', 'Opposite', 'Next'    ))
classes.append (Class ('Vertex'          , 'Vertex'  , 'Face'  , 'Inner'   , 'TerminatingVertex', 'Next'    , ''        , 'Prev'    , ''        ))
classes.append (Class ('InnerHalfEdge'   , 'HalfEdge', 'Face'  , 'Inner'   , ''                 , 'Next'    , ''        , 'Prev'    , ''        ))
classes.append (Class ('OuterHalfEdge'   , 'HalfEdge', 'Face'  , 'Inner'   , 'OppositeHalfEdge' , 'Next'    , ''        , 'Prev'    , ''        ))
classes.append (Class ('Face'            , 'Face'    , 'Face'  , 'Inner'   , 'OppositeFace'     , 'Next'    , ''        , 'Prev'    , ''        ))

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

f.write ('#ifndef PCL_GEOMETRY_MESH_CIRCULATORS_HPP\n')
f.write ('#define PCL_GEOMETRY_MESH_CIRCULATORS_HPP\n\n')

f.write ('#include <pcl/geometry/boost.h>\n\n')

for c in classes:

    value_name      = c.value_name
    value_type      = c.value_type
    around_type     = c.around_type
    around_obj      = around_type.lower ()
    around_idx      = 'idx_' + around_obj
    he_getter       = c.he_getter
    he_getter_idx   = 'idx_' + he_getter.lower () + '_half_edge'
    he_getter_idx_  = he_getter_idx + '_'
    deref           = c.deref
    inc1            = c.inc1
    inc2            = c.inc2
    dec1            = c.dec1
    dec2            = c.dec2

    class_name      = value_name + 'Around' + around_type + 'Circulator'
    class_base_name = 'boost::iterator_facade <' + namespace + '::' + class_name + ' <MeshT>, typename boost::conditional <boost::is_const <MeshT>::value, typename boost::add_const <typename MeshT::' + value_type + '>::type, typename MeshT::' + value_type + '>::type, boost::bidirectional_traversal_tag>'

    placeholder_at  = ' ' * (len (around_type) - 3)
    placeholder_cn  = ' ' *  len (class_name)
    placeholder_gt  = ' ' * (len (he_getter_idx) - 5)

    f.write ('////////////////////////////////////////////////////////////////////////////////\n')
    f.write ('// ' + class_name + '\n')
    f.write ('////////////////////////////////////////////////////////////////////////////////\n\n')

    f.write ('namespace ' + namespace + '\n')
    f.write ('{\n\n')

    f.write ('  template <class MeshT>\n')
    f.write ('  class ' + class_name + '\n')
    f.write ('      : public ' + class_base_name + '\n')
    f.write ('  {\n')
    f.write ('    public:\n\n')

    f.write ('      typedef ' + class_base_name + ' Base;\n\n')

    f.write ('      typedef typename Base::value_type        value_type;\n')
    f.write ('      typedef typename Base::reference         reference;\n')
    f.write ('      typedef typename Base::pointer           pointer;\n')
    f.write ('      typedef typename Base::difference_type   difference_type;\n')
    f.write ('      typedef typename Base::iterator_category iterator_category;\n\n')

    f.write ('      typedef MeshT                        Mesh;\n\n')

    f.write ('      typedef typename Mesh::Vertex        Vertex;\n')
    f.write ('      typedef typename Mesh::HalfEdge      HalfEdge;\n')
    f.write ('      typedef typename Mesh::Face          Face;\n\n')

    f.write ('      typedef typename Mesh::VertexIndex   VertexIndex;\n')
    f.write ('      typedef typename Mesh::HalfEdgeIndex HalfEdgeIndex;\n')
    f.write ('      typedef typename Mesh::FaceIndex     FaceIndex;\n\n')

    f.write ('    private:\n\n')

    f.write ('      struct enabler {};\n\n')

    f.write ('    public:\n\n')

    if he_getter=='Incoming':
        tmp = '*mesh_'
    else:
        tmp = ''

    f.write ('      ' + class_name     + ' (const '      + around_type    + '& ' + around_obj + ',\n')
    f.write ('      ' + placeholder_cn + '  Mesh*const ' + placeholder_at        + 'mesh)\n')
    f.write ('        : mesh_ ' + placeholder_gt + ' (mesh),\n')
    f.write ('          '       + he_getter_idx_ + ' (' + around_obj + '.get' + he_getter + 'HalfEdgeIndex (' + tmp + '))\n')
    f.write ('      {\n')
    f.write ('      }\n\n')

    f.write ('      ' + class_name     + ' (const '      + around_type    + 'Index& ' + around_idx + ',\n')
    f.write ('      ' + placeholder_cn + '  Mesh*const ' + placeholder_at + '     '   + 'mesh)\n')
    f.write ('        : mesh_ ' + placeholder_gt + ' (mesh),\n')
    f.write ('          '       + he_getter_idx_ + ' (mesh->getElement (' + around_idx + ').get' + he_getter + 'HalfEdgeIndex (' + tmp + '))\n')
    f.write ('      {\n')
    f.write ('      }\n\n')

    f.write ('      ' + class_name     + ' (const HalfEdgeIndex& ' + he_getter_idx  + ',\n')
    f.write ('      ' + placeholder_cn + '  Mesh*const           ' + 'mesh)\n')
    f.write ('        : mesh_ ' + placeholder_gt + ' (mesh),\n')
    f.write ('          '       + he_getter_idx_ + ' (' + he_getter_idx + ')\n')
    f.write ('      {\n')
    f.write ('      }\n\n')

    f.write ('      template <class OtherMeshT>\n')
    f.write ('      ' + class_name     + ' (const ' + namespace + '::' + class_name + ' <OtherMeshT>& other,\n')
    f.write ('      ' + placeholder_cn + '  typename boost::enable_if <boost::is_convertible <OtherMeshT*, Mesh*>, enabler>::type = enabler ())\n')
    f.write ('        : mesh_ ' + placeholder_gt + ' (other.mesh_),\n')
    f.write ('          '       + he_getter_idx_ + ' (other.' + he_getter_idx_ + ')\n')
    f.write ('      {\n')
    f.write ('      }\n\n')

    f.write ('    public:\n\n')

    f.write ('      const HalfEdgeIndex&\n')
    f.write ('      getCurrentHalfEdgeIndex () const\n')
    f.write ('      {\n')
    f.write ('        return (' + he_getter_idx_ + ');\n')
    f.write ('      }\n\n')

    if deref:
        tmp = 'mesh_->getElement (' + he_getter_idx_ + ').get' + deref + 'Index ('
        if deref=='OppositeFace':
            tmp = tmp + '*mesh_)'
        else:
            tmp = tmp + ')'
    else:
        tmp = he_getter_idx_

    f.write ('      const ' + value_type + 'Index&\n')
    f.write ('      getDereferencedIndex () const\n')
    f.write ('      {\n')
    f.write ('        return (' + tmp + ');\n')
    f.write ('      }\n\n')

    f.write ('      bool\n')
    f.write ('      isValid () const\n')
    f.write ('      {\n')
    f.write ('        return (this->getCurrentHalfEdgeIndex ().isValid ());\n')
    f.write ('      }\n\n')

    f.write ('    private:\n\n')

    f.write ('      friend class boost::iterator_core_access;\n')
    f.write ('      template <class> friend class ' + namespace + '::' + class_name + ';\n\n')

    f.write ('      template <class OtherMeshT> bool\n')
    f.write ('      equal (const ' + namespace + '::' + class_name + ' <OtherMeshT>& other) const\n')
    f.write ('      {\n')
    f.write ('        return (mesh_ == other.mesh_ && ' + he_getter_idx_ + ' == other.' + he_getter_idx_ + ');\n')
    f.write ('      }\n\n')

    if inc2:
        tmp = inc1 + 'HalfEdge (*mesh_).get' + inc2
    else:
        tmp = inc1

    f.write ('      void\n')
    f.write ('      increment ()\n')
    f.write ('      {\n')
    f.write ('        ' + he_getter_idx_ + ' = mesh_->getElement (' + he_getter_idx_ + ').get' + tmp + 'HalfEdgeIndex ();\n')
    f.write ('      }\n\n')

    if dec2:
        tmp = dec1 + 'HalfEdge (*mesh_).get' + dec2
    else:
        tmp = dec1

    f.write ('      void\n')
    f.write ('      decrement ()\n')
    f.write ('      {\n')
    f.write ('        ' + he_getter_idx_ + ' = mesh_->getElement (' + he_getter_idx_ + ').get' + tmp + 'HalfEdgeIndex ();\n')
    f.write ('      }\n\n')

    f.write ('      reference\n')
    f.write ('      dereference () const\n')
    f.write ('      {\n')
    f.write ('        return (mesh_->getElement (this->getDereferencedIndex ()));\n')
    f.write ('      }\n\n')

    f.write ('    private:\n\n')

    f.write ('      Mesh*         mesh_;\n')
    f.write ('      HalfEdgeIndex ' + he_getter_idx_ + ';\n')
    f.write ('  };\n\n')

    f.write ('} // End namespace ' + namespace + '\n\n')

f.write ('#endif // PCL_GEOMETRY_MESH_CIRCULATORS_HPP\n\n')

f.close()
