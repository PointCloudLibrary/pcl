/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
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

#ifdef __GNUC__
#pragma GCC system_header 
#endif

#include "opennurbs_system.h"       /* system headers used by openNURBS code */

#include "opennurbs_3dm.h"          /* 3DM typecode (TCODE) definitions */

#include "opennurbs_defines.h"      /* openNURBS defines and enums */
#include "opennurbs_error.h"        /* error handling */
#include "opennurbs_memory.h"       /* memory managment (onmalloc(), onrealloc(), onfree(), ...) */
#include "opennurbs_rand.h"         /* random number generator */
#include "opennurbs_crc.h"          /* cyclic redundancy check tool */
#include "opennurbs_uuid.h"         /* universally unique identifiers (UUID, a.k.a, GUID) */
#include "opennurbs_unicode.h"      /* unicode string conversion */

#if defined(ON_CPLUSPLUS)

#include "opennurbs_fsp.h"            // fixed size memory pool
#include "opennurbs_string.h"         // dynamic string classes (single and double byte)
#include "opennurbs_compress.h"
#include "opennurbs_base64.h"         // base64 encodeing and decoding
#include "opennurbs_color.h"          // R G B color
#include "opennurbs_linestyle.h"      // line pattern, scale, and width
#include "opennurbs_point.h"          // double precision 2d, 3d, 4d points and 2d, 3d vectors
#include "opennurbs_fpoint.h"         // float precision 2d, 3d, 4d points and 2d, 3d vectors
#include "opennurbs_array.h"          // dynamic array templates
#include "opennurbs_base32.h"         // base32 encodeing and decoding
#include "opennurbs_pluginlist.h"
#include "opennurbs_bounding_box.h"   // simple 3d axis aligned bounding box
#include "opennurbs_matrix.h"         // general m X n matrix
#include "opennurbs_xform.h"          // 4 X 4 transformation matrix
#include "opennurbs_workspace.h"      // workspace memory allocation
#include "opennurbs_plane.h"          // simple 3d plane
#include "opennurbs_circle.h"         // simple 3d circle
#include "opennurbs_ellipse.h"        // simple 3d ellipse
#include "opennurbs_line.h"           // simple line
#include "opennurbs_polyline.h"       // simple polyline
#include "opennurbs_cylinder.h"       // simple 3d elliptical cylinder
#include "opennurbs_cone.h"           // simple 3d right circular cone
#include "opennurbs_sphere.h"         // simple 3d sphere
#include "opennurbs_box.h"            // simple 3d box
#include "opennurbs_torus.h"          // simple 3d torus
#include "opennurbs_bezier.h"         // simple bezier and polynomial curves and surfaces
#include "opennurbs_math.h"           // utilities for performing simple calculations
#include "opennurbs_intersect.h"      // utilities for performing simple intersections
#include "opennurbs_optimize.h"       // utilities for finding extrema and zeros
#include "opennurbs_knot.h"           // utilities for working with NURBS knot vectors
#include "opennurbs_evaluate_nurbs.h" // utilities for evaluating Beziers and NURBS
#include "opennurbs_textlog.h"        // text log for dumps, error logs, etc.
#include "opennurbs_zlib.h"           // ON_BinaryArchive needs z_stream definition.
#include "opennurbs_rtree.h"          // ON_RTree spatial search utility.
#include "opennurbs_mapchan.h"
#include "opennurbs_rendering.h"
#include "opennurbs_archive.h"        // binary arcive objects for serialization to file, memory blocks, etc.
#include "opennurbs_arc.h"            // simple 3d circular arc
#include "opennurbs_object.h"         // virtual base class for all openNURBS objects
#include "opennurbs_userdata.h"       // class for attaching persistent user information to openNURBS objects
#include "opennurbs_geometry.h"       // virtual base class for geometric objects
#include "opennurbs_curve.h"          // virtual parametric curve
#include "opennurbs_surface.h"        // virtual parametric surface
#include "opennurbs_viewport.h"       // simple renering projection
#include "opennurbs_texture_mapping.h" // texture coordinate evaluation
#include "opennurbs_texture.h"        // texture definition
#include "opennurbs_material.h"       // simple rendering material
#include "opennurbs_layer.h"          // layer definition
#include "opennurbs_linetype.h"          // linetype definition
#include "opennurbs_group.h"          // group name and index
#include "opennurbs_font.h"           // font name and index
#include "opennurbs_light.h"          // light
#include "opennurbs_pointgeometry.h"  // single point
#include "opennurbs_pointcloud.h"     // point set
#include "opennurbs_curveproxy.h"     // proxy curve provides a way to use an existing curve
#include "opennurbs_surfaceproxy.h"   // proxy surface provides a way to use another surface
#include "opennurbs_mesh.h"           // render mesh object
#include "opennurbs_pointgrid.h"      // point grid object
#include "opennurbs_linecurve.h"      // line as a paramtric curve object
#include "opennurbs_arccurve.h"       // arc/circle as a paramtric curve object
#include "opennurbs_polylinecurve.h"  // polyline as a paramtric curve object
#include "opennurbs_nurbscurve.h"     // NURBS curve
#include "opennurbs_polycurve.h"      // polycurve (composite curve)
#include "opennurbs_curveonsurface.h" // curve on surface (other kind of composite curve)
#include "opennurbs_nurbssurface.h"   // NURBS surface
#include "opennurbs_planesurface.h"   // plane surface
#include "opennurbs_revsurface.h"     // surface of revolution
#include "opennurbs_sumsurface.h"     // sum surface
#include "opennurbs_brep.h"           // boundary rep
#include "opennurbs_beam.h"           // lightweight extrusion object
#include "opennurbs_bitmap.h"         // Windows and OpenGL bitmaps
#include "opennurbs_instance.h"       // instance definitions and references
#include "opennurbs_3dm_properties.h"
#include "opennurbs_3dm_settings.h"
#include "opennurbs_3dm_attributes.h"
#include "opennurbs_annotation.h"     // used to serialize annotation objects
#include "opennurbs_annotation2.h"    // used to serialize annotation2 objects
#include "opennurbs_hatch.h"          // hatch geometry definitions
#include "opennurbs_dimstyle.h"       // dimstyle name and index
#include "opennurbs_hatch.h"          // hatch geometry definitions
#include "opennurbs_linetype.h"       // linetype pattern definitions
#include "opennurbs_objref.h"         // ON_ObjRef definition
#include "opennurbs_offsetsurface.h"  // ON_OffsetSurface definition
#include "opennurbs_detail.h"         // ON_Detail definition
#include "opennurbs_lookup.h"         // ON_SerialNumberTable
#include "opennurbs_object_history.h"
#include "opennurbs_extensions.h"

#endif

#endif
