/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2011 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/


////////////////////////////////////////////////////////////////
//
//   Includes all openNURBS toolkit headers required to use the
//   openNURBS toolkit library.  See readme.txt for details.
//
////////////////////////////////////////////////////////////////

#if !defined(OPENNURBS_INC_)
#define OPENNURBS_INC_

#include <pcl/surface/openNURBS/opennurbs_system.h>       /* system headers used by openNURBS code */

#include <pcl/surface/openNURBS/opennurbs_3dm.h>          /* 3DM typecode (TCODE) definitions */

#include <pcl/surface/openNURBS/opennurbs_defines.h>      /* openNURBS defines and enums */
#include <pcl/surface/openNURBS/opennurbs_error.h>        /* error handling */
#include <pcl/surface/openNURBS/opennurbs_memory.h>       /* memory managment (onmalloc(), onrealloc(), onfree(), ...) */
#include <pcl/surface/openNURBS/opennurbs_rand.h>         /* random number generator */
#include <pcl/surface/openNURBS/opennurbs_crc.h>          /* cyclic redundancy check tool */
#include <pcl/surface/openNURBS/opennurbs_uuid.h>         /* universally unique identifiers (UUID, a.k.a, GUID) */
#include <pcl/surface/openNURBS/opennurbs_unicode.h>      /* unicode string conversion */

#if defined(ON_CPLUSPLUS)

#include <pcl/surface/openNURBS/opennurbs_fsp.h>            // fixed size memory pool
#include <pcl/surface/openNURBS/opennurbs_string.h>         // dynamic string classes (single and double byte)
#include <pcl/surface/openNURBS/opennurbs_compress.h>
#include <pcl/surface/openNURBS/opennurbs_base64.h>         // base64 encodeing and decoding
#include <pcl/surface/openNURBS/opennurbs_color.h>          // R G B color
#include <pcl/surface/openNURBS/opennurbs_linestyle.h>      // line pattern, scale, and width
#include <pcl/surface/openNURBS/opennurbs_point.h>          // double precision 2d, 3d, 4d points and 2d, 3d vectors
#include <pcl/surface/openNURBS/opennurbs_fpoint.h>         // float precision 2d, 3d, 4d points and 2d, 3d vectors
#include <pcl/surface/openNURBS/opennurbs_array.h>          // dynamic array templates
#include <pcl/surface/openNURBS/opennurbs_base32.h>         // base32 encodeing and decoding
#include <pcl/surface/openNURBS/opennurbs_pluginlist.h>
#include <pcl/surface/openNURBS/opennurbs_bounding_box.h>   // simple 3d axis aligned bounding box
#include <pcl/surface/openNURBS/opennurbs_matrix.h>         // general m X n matrix
#include <pcl/surface/openNURBS/opennurbs_xform.h>          // 4 X 4 transformation matrix
#include <pcl/surface/openNURBS/opennurbs_quaternion.h>
#include <pcl/surface/openNURBS/opennurbs_workspace.h>      // workspace memory allocation
#include <pcl/surface/openNURBS/opennurbs_plane.h>          // simple 3d plane
#include <pcl/surface/openNURBS/opennurbs_circle.h>         // simple 3d circle
#include <pcl/surface/openNURBS/opennurbs_ellipse.h>        // simple 3d ellipse
#include <pcl/surface/openNURBS/opennurbs_line.h>           // simple line
#include <pcl/surface/openNURBS/opennurbs_polyline.h>       // simple polyline
#include <pcl/surface/openNURBS/opennurbs_cylinder.h>       // simple 3d elliptical cylinder
#include <pcl/surface/openNURBS/opennurbs_cone.h>           // simple 3d right circular cone
#include <pcl/surface/openNURBS/opennurbs_sphere.h>         // simple 3d sphere
#include <pcl/surface/openNURBS/opennurbs_box.h>            // simple 3d box
#include <pcl/surface/openNURBS/opennurbs_torus.h>          // simple 3d torus
#include <pcl/surface/openNURBS/opennurbs_bezier.h>         // simple bezier and polynomial curves and surfaces
#include <pcl/surface/openNURBS/opennurbs_math.h>           // utilities for performing simple calculations
#include <pcl/surface/openNURBS/opennurbs_intersect.h>      // utilities for performing simple intersections
#include <pcl/surface/openNURBS/opennurbs_optimize.h>       // utilities for finding extrema and zeros
#include <pcl/surface/openNURBS/opennurbs_knot.h>           // utilities for working with NURBS knot vectors
#include <pcl/surface/openNURBS/opennurbs_evaluate_nurbs.h> // utilities for evaluating Beziers and NURBS
#include <pcl/surface/openNURBS/opennurbs_textlog.h>        // text log for dumps, error logs, etc.
#include <pcl/surface/openNURBS/opennurbs_zlib.h>           // ON_BinaryArchive needs z_stream definition.
#include <pcl/surface/openNURBS/opennurbs_rtree.h>          // ON_RTree spatial search utility.
#include <pcl/surface/openNURBS/opennurbs_mapchan.h>
#include <pcl/surface/openNURBS/opennurbs_rendering.h>
#include <pcl/surface/openNURBS/opennurbs_archive.h>        // binary arcive objects for serialization to file, memory blocks, etc.
#include <pcl/surface/openNURBS/opennurbs_massprop.h>       // mass properties values
#include <pcl/surface/openNURBS/opennurbs_arc.h>            // simple 3d circular arc
#include <pcl/surface/openNURBS/opennurbs_object.h>         // virtual base class for all openNURBS objects
#include <pcl/surface/openNURBS/opennurbs_userdata.h>       // class for attaching persistent user information to openNURBS objects
#include <pcl/surface/openNURBS/opennurbs_geometry.h>       // virtual base class for geometric objects
#include <pcl/surface/openNURBS/opennurbs_curve.h>          // virtual parametric curve
#include <pcl/surface/openNURBS/opennurbs_surface.h>        // virtual parametric surface
#include <pcl/surface/openNURBS/opennurbs_viewport.h>       // simple renering projection
#include <pcl/surface/openNURBS/opennurbs_texture_mapping.h> // texture coordinate evaluation
#include <pcl/surface/openNURBS/opennurbs_texture.h>        // texture definition
#include <pcl/surface/openNURBS/opennurbs_material.h>       // simple rendering material
#include <pcl/surface/openNURBS/opennurbs_layer.h>          // layer definition
#include <pcl/surface/openNURBS/opennurbs_linetype.h>          // linetype definition
#include <pcl/surface/openNURBS/opennurbs_group.h>          // group name and index
#include <pcl/surface/openNURBS/opennurbs_font.h>           // font name and index
#include <pcl/surface/openNURBS/opennurbs_light.h>          // light
#include <pcl/surface/openNURBS/opennurbs_pointgeometry.h>  // single point
#include <pcl/surface/openNURBS/opennurbs_pointcloud.h>     // point set
#include <pcl/surface/openNURBS/opennurbs_curveproxy.h>     // proxy curve provides a way to use an existing curve
#include <pcl/surface/openNURBS/opennurbs_surfaceproxy.h>   // proxy surface provides a way to use another surface
#include <pcl/surface/openNURBS/opennurbs_mesh.h>           // render mesh object
#include <pcl/surface/openNURBS/opennurbs_pointgrid.h>      // point grid object
#include <pcl/surface/openNURBS/opennurbs_linecurve.h>      // line as a paramtric curve object
#include <pcl/surface/openNURBS/opennurbs_arccurve.h>       // arc/circle as a paramtric curve object
#include <pcl/surface/openNURBS/opennurbs_polylinecurve.h>  // polyline as a paramtric curve object
#include <pcl/surface/openNURBS/opennurbs_nurbscurve.h>     // NURBS curve
#include <pcl/surface/openNURBS/opennurbs_polycurve.h>      // polycurve (composite curve)
#include <pcl/surface/openNURBS/opennurbs_curveonsurface.h> // curve on surface (other kind of composite curve)
#include <pcl/surface/openNURBS/opennurbs_nurbssurface.h>   // NURBS surface
#include <pcl/surface/openNURBS/opennurbs_planesurface.h>   // plane surface
#include <pcl/surface/openNURBS/opennurbs_revsurface.h>     // surface of revolution
#include <pcl/surface/openNURBS/opennurbs_sumsurface.h>     // sum surface
#include <pcl/surface/openNURBS/opennurbs_brep.h>           // boundary rep
#include <pcl/surface/openNURBS/opennurbs_beam.h>           // lightweight extrusion object
#include <pcl/surface/openNURBS/opennurbs_bitmap.h>         // Windows and OpenGL bitmaps
#include <pcl/surface/openNURBS/opennurbs_instance.h>       // instance definitions and references
#include <pcl/surface/openNURBS/opennurbs_3dm_properties.h>
#include <pcl/surface/openNURBS/opennurbs_3dm_settings.h>
#include <pcl/surface/openNURBS/opennurbs_3dm_attributes.h>
#include <pcl/surface/openNURBS/opennurbs_annotation.h>     // used to serialize annotation objects
#include <pcl/surface/openNURBS/opennurbs_annotation2.h>    // used to serialize annotation2 objects
#include <pcl/surface/openNURBS/opennurbs_hatch.h>          // hatch geometry definitions
#include <pcl/surface/openNURBS/opennurbs_dimstyle.h>       // dimstyle name and index
#include <pcl/surface/openNURBS/opennurbs_hatch.h>          // hatch geometry definitions
#include <pcl/surface/openNURBS/opennurbs_linetype.h>       // linetype pattern definitions
#include <pcl/surface/openNURBS/opennurbs_objref.h>         // ON_ObjRef definition
#include <pcl/surface/openNURBS/opennurbs_offsetsurface.h>  // ON_OffsetSurface definition
#include <pcl/surface/openNURBS/opennurbs_detail.h>         // ON_Detail definition
#include <pcl/surface/openNURBS/opennurbs_lookup.h>         // ON_SerialNumberTable
#include <pcl/surface/openNURBS/opennurbs_object_history.h>

#include <pcl/surface/openNURBS/opennurbs_extensions.h>

#endif

#endif
