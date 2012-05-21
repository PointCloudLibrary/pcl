###############################################################

# If your make doesn't use .cpp for C++ extensions, 
# then uncomment the suffixes line.
.SUFFIXES : .c .cpp

RM = /bin/rm
# If you don't mind the risk of using the -f option and your rm 
# asks too many questions, then uncomment the next line.
# RM = /bin/rm -f

AR = ar qvl

# If your system doesn't use ranlib, uncomment the "echo" define.
RANLIB = ranlib
# RANLIB = echo

###############################################################
# Gnu tools
#
# (Tested with gcc 4.1.2 on SUSE Linux)
#
# In order to get full wide character (UNICODE) support, you
# need to define _GNU_SOURCE

# C compiler and flags
CC = gcc
CFLAGS = -g -Wall -D_GNU_SOURCE -DMY_ZCALLOC -DZ_PREFIX -I.
#CFLAGS = -O -Wall -D_GNU_SOURCE -DMY_ZCALLOC -DZ_PREFIX -I.

# C++ compiler and flags
CCC = g++
CCFLAGS = -g -Wall -D_GNU_SOURCE -I.
#CCFLAGS = -O -Wall -D_GNU_SOURCE -I.

LINK = $(CCC)
LINKFLAGS =




###############################################################
# Irix 6.5 compiler (uncomment the lines in this block)
#
#

#OPTIMIZER       = -g 
#OPTIMIZER       = -O3

#ABIISA          = -n32 -mips3
#ABIISA          = -64  -mips3

#IRIXDEF 	= -D_WCHAR_T_DEFINED -DON_COMPILER_IRIX -D_GNU_SOURCE

#IRIXFLAGS 	= -woff all -no_prelink -ptused 

#CC = cc
#CFLAGS = $(OPTIMIZER) $(ABIISA) $(IRIXDEF) $(IRIXFLAGS) -DMY_ZCALLOC -DZ_PREFIX -I.

#CCC = CC
#CCFLAGS = $(OPTIMIZER) $(ABIISA) $(IRIXDEF) $(IRIXFLAGS) -I.

#LINK = $(CCC)
#LINKFLAGS =
# uncomment for 64 bit linking
#LINKFLAGS = $CCFLAGS

# 
###############################################################





###############################################################

.cpp.o :
	$(CCC) $(CCFLAGS) -c $*.cpp -o $*.o

# If your make doesn't have a rule for .c to .o, uncomment the
# next two lines.
#.c.o :
#	$(CC) $(CFLAGS) -c $*.c -o $*.o


###############################################################

OPENNURBS_LIB_NAME = openNURBS
OPENNURBS_LIB_FILE = lib$(OPENNURBS_LIB_NAME).a

ON_INC= opennurbs.h \
		opennurbs_3dm.h \
		opennurbs_3dm_attributes.h \
		opennurbs_3dm_properties.h \
		opennurbs_3dm_settings.h \
		opennurbs_annotation.h \
		opennurbs_annotation2.h \
		opennurbs_arc.h \
		opennurbs_arccurve.h \
		opennurbs_archive.h \
		opennurbs_array.h \
		opennurbs_array_defs.h \
		opennurbs_base64.h \
		opennurbs_beam.h \
		opennurbs_bezier.h \
		opennurbs_bitmap.h \
		opennurbs_bounding_box.h \
		opennurbs_box.h \
		opennurbs_brep.h \
		opennurbs_circle.h \
		opennurbs_color.h \
		opennurbs_compress.h \
		opennurbs_cone.h \
		opennurbs_crc.h \
		opennurbs_curve.h \
		opennurbs_curveonsurface.h \
		opennurbs_curveproxy.h \
		opennurbs_cylinder.h \
		opennurbs_defines.h \
		opennurbs_detail.h \
		opennurbs_dimstyle.h \
		opennurbs_ellipse.h \
		opennurbs_error.h \
		opennurbs_evaluate_nurbs.h \
		opennurbs_extensions.h \
		opennurbs_font.h \
		opennurbs_fpoint.h \
		opennurbs_fsp.h \
		opennurbs_fsp_defs.h \
		opennurbs_geometry.h \
		opennurbs_gl.h \
		opennurbs_group.h \
		opennurbs_hatch.h \
		opennurbs_hsort_template.h \
		opennurbs_instance.h \
		opennurbs_intersect.h \
		opennurbs_knot.h \
		opennurbs_layer.h \
		opennurbs_light.h \
		opennurbs_line.h \
		opennurbs_linecurve.h \
		opennurbs_linestyle.h \
		opennurbs_linetype.h \
		opennurbs_lookup.h \
		opennurbs_mapchan.h \
		opennurbs_massprop.h \
		opennurbs_material.h \
		opennurbs_math.h \
		opennurbs_matrix.h \
		opennurbs_memory.h \
		opennurbs_mesh.h \
		opennurbs_nurbscurve.h \
		opennurbs_nurbssurface.h \
		opennurbs_object.h \
		opennurbs_object_history.h \
		opennurbs_objref.h \
		opennurbs_offsetsurface.h \
		opennurbs_optimize.h \
		opennurbs_plane.h \
		opennurbs_planesurface.h \
		opennurbs_pluginlist.h \
		opennurbs_point.h \
		opennurbs_pointcloud.h \
		opennurbs_pointgeometry.h \
		opennurbs_pointgrid.h \
		opennurbs_polycurve.h \
		opennurbs_polyedgecurve.h \
		opennurbs_polyline.h \
		opennurbs_polylinecurve.h \
		opennurbs_qsort_template.h \
		opennurbs_rand.h \
		opennurbs_rendering.h \
		opennurbs_revsurface.h \
		opennurbs_rtree.h \
		opennurbs_sphere.h \
		opennurbs_string.h \
		opennurbs_sumsurface.h \
		opennurbs_surface.h \
		opennurbs_surfaceproxy.h \
		opennurbs_system.h \
		opennurbs_textlog.h \
		opennurbs_texture.h \
		opennurbs_texture_mapping.h \
		opennurbs_torus.h \
		opennurbs_unicode.h \
		opennurbs_userdata.h \
		opennurbs_uuid.h \
		opennurbs_version.h \
		opennurbs_viewport.h \
		opennurbs_workspace.h \
		opennurbs_xform.h \
		opennurbs_zlib.h

ON_SRC= opennurbs_3dm_attributes.cpp \
		opennurbs_3dm_properties.cpp \
		opennurbs_3dm_settings.cpp \
		opennurbs_annotation.cpp \
		opennurbs_annotation2.cpp \
		opennurbs_arc.cpp \
		opennurbs_arccurve.cpp \
		opennurbs_archive.cpp \
		opennurbs_array.cpp \
		opennurbs_base64.cpp \
		opennurbs_basic.cpp \
		opennurbs_beam.cpp \
		opennurbs_bezier.cpp \
		opennurbs_beziervolume.cpp \
		opennurbs_bitmap.cpp \
		opennurbs_bounding_box.cpp \
		opennurbs_box.cpp \
		opennurbs_brep.cpp \
		opennurbs_brep_extrude.cpp \
		opennurbs_brep_io.cpp \
		opennurbs_brep_isvalid.cpp \
		opennurbs_brep_region.cpp \
		opennurbs_brep_tools.cpp \
		opennurbs_brep_v2valid.cpp \
		opennurbs_circle.cpp \
		opennurbs_color.cpp \
		opennurbs_compress.cpp \
		opennurbs_cone.cpp \
		opennurbs_crc.cpp \
		opennurbs_curve.cpp \
		opennurbs_curveonsurface.cpp \
		opennurbs_curveproxy.cpp \
		opennurbs_cylinder.cpp \
		opennurbs_defines.cpp \
		opennurbs_detail.cpp \
		opennurbs_dimstyle.cpp \
		opennurbs_ellipse.cpp \
		opennurbs_embedded_file.cpp \
		opennurbs_error.cpp \
		opennurbs_error_message.cpp \
		opennurbs_evaluate_nurbs.cpp \
		opennurbs_extensions.cpp \
		opennurbs_font.cpp \
		opennurbs_fsp.cpp \
		opennurbs_geometry.cpp \
		opennurbs_group.cpp \
		opennurbs_hatch.cpp \
		opennurbs_instance.cpp \
		opennurbs_intersect.cpp \
		opennurbs_knot.cpp \
		opennurbs_layer.cpp \
		opennurbs_light.cpp \
		opennurbs_line.cpp \
		opennurbs_linecurve.cpp \
		opennurbs_linetype.cpp \
		opennurbs_lookup.cpp \
		opennurbs_massprop.cpp \
		opennurbs_material.cpp \
		opennurbs_math.cpp \
		opennurbs_matrix.cpp \
		opennurbs_memory.c \
		opennurbs_memory_util.c \
		opennurbs_mesh.cpp \
		opennurbs_mesh_ngon.cpp \
		opennurbs_mesh_tools.cpp \
		opennurbs_morph.cpp \
		opennurbs_nurbscurve.cpp \
		opennurbs_nurbssurface.cpp \
		opennurbs_nurbsvolume.cpp \
		opennurbs_object.cpp \
		opennurbs_object_history.cpp \
		opennurbs_objref.cpp \
		opennurbs_offsetsurface.cpp \
		opennurbs_optimize.cpp \
		opennurbs_plane.cpp \
		opennurbs_planesurface.cpp \
		opennurbs_pluginlist.cpp \
		opennurbs_point.cpp \
		opennurbs_pointcloud.cpp \
		opennurbs_pointgeometry.cpp \
		opennurbs_pointgrid.cpp \
		opennurbs_polycurve.cpp \
		opennurbs_polyedgecurve.cpp \
		opennurbs_polyline.cpp \
		opennurbs_polylinecurve.cpp \
		opennurbs_rand.cpp \
		opennurbs_revsurface.cpp \
		opennurbs_rtree.cpp \
		opennurbs_sort.cpp \
		opennurbs_sphere.cpp \
		opennurbs_string.cpp \
		opennurbs_sum.cpp \
		opennurbs_sumsurface.cpp \
		opennurbs_surface.cpp \
		opennurbs_surfaceproxy.cpp \
		opennurbs_textlog.cpp \
		opennurbs_torus.cpp \
		opennurbs_unicode.cpp \
		opennurbs_userdata.cpp \
		opennurbs_uuid.cpp \
		opennurbs_viewport.cpp \
		opennurbs_workspace.cpp \
		opennurbs_wstring.cpp \
		opennurbs_xform.cpp \
		opennurbs_zlib.cpp \
		opennurbs_zlib_memory.cpp

ON_OBJ= opennurbs_3dm_attributes.o \
		opennurbs_3dm_properties.o \
		opennurbs_3dm_settings.o \
		opennurbs_annotation.o \
		opennurbs_annotation2.o \
		opennurbs_arc.o \
		opennurbs_arccurve.o \
		opennurbs_archive.o \
		opennurbs_array.o \
		opennurbs_base64.o \
		opennurbs_basic.o \
		opennurbs_beam.o \
		opennurbs_bezier.o \
		opennurbs_beziervolume.o \
		opennurbs_bitmap.o \
		opennurbs_bounding_box.o \
		opennurbs_box.o \
		opennurbs_brep.o \
		opennurbs_brep_extrude.o \
		opennurbs_brep_io.o \
		opennurbs_brep_isvalid.o \
		opennurbs_brep_region.o \
		opennurbs_brep_tools.o \
		opennurbs_brep_v2valid.o \
		opennurbs_circle.o \
		opennurbs_color.o \
		opennurbs_compress.o \
		opennurbs_cone.o \
		opennurbs_crc.o \
		opennurbs_curve.o \
		opennurbs_curveonsurface.o \
		opennurbs_curveproxy.o \
		opennurbs_cylinder.o \
		opennurbs_defines.o \
		opennurbs_detail.o \
		opennurbs_dimstyle.o \
		opennurbs_ellipse.o \
		opennurbs_embedded_file.o \
		opennurbs_error.o \
		opennurbs_error_message.o \
		opennurbs_evaluate_nurbs.o \
		opennurbs_extensions.o \
		opennurbs_font.o \
		opennurbs_fsp.o \
		opennurbs_geometry.o \
		opennurbs_group.o \
		opennurbs_hatch.o \
		opennurbs_instance.o \
		opennurbs_intersect.o \
		opennurbs_knot.o \
		opennurbs_layer.o \
		opennurbs_light.o \
		opennurbs_line.o \
		opennurbs_linecurve.o \
		opennurbs_linetype.o \
		opennurbs_lookup.o \
		opennurbs_massprop.o \
		opennurbs_material.o \
		opennurbs_math.o \
		opennurbs_matrix.o \
		opennurbs_memory.o \
		opennurbs_memory_util.o \
		opennurbs_mesh.o \
		opennurbs_mesh_ngon.o \
		opennurbs_mesh_tools.o \
		opennurbs_morph.o \
		opennurbs_nurbscurve.o \
		opennurbs_nurbssurface.o \
		opennurbs_nurbsvolume.o \
		opennurbs_object.o \
		opennurbs_object_history.o \
		opennurbs_objref.o \
		opennurbs_offsetsurface.o \
		opennurbs_optimize.o \
		opennurbs_plane.o \
		opennurbs_planesurface.o \
		opennurbs_pluginlist.o \
		opennurbs_point.o \
		opennurbs_pointcloud.o \
		opennurbs_pointgeometry.o \
		opennurbs_pointgrid.o \
		opennurbs_polycurve.o \
		opennurbs_polyedgecurve.o \
		opennurbs_polyline.o \
		opennurbs_polylinecurve.o \
		opennurbs_rand.o \
		opennurbs_revsurface.o \
		opennurbs_rtree.o \
		opennurbs_sort.o \
		opennurbs_sphere.o \
		opennurbs_string.o \
		opennurbs_sum.o \
		opennurbs_sumsurface.o \
		opennurbs_surface.o \
		opennurbs_surfaceproxy.o \
		opennurbs_textlog.o \
		opennurbs_torus.o \
		opennurbs_unicode.o \
		opennurbs_userdata.o \
		opennurbs_uuid.o \
		opennurbs_viewport.o \
		opennurbs_workspace.o \
		opennurbs_wstring.o \
		opennurbs_xform.o \
		opennurbs_zlib.o \
		opennurbs_zlib_memory.o

ZLIB_INC= zlib/crc32.h \
		zlib/deflate.h \
		zlib/inffast.h \
		zlib/inffixed.h \
		zlib/inflate.h \
		zlib/inftrees.h \
		zlib/trees.h \
		zlib/zconf.h \
		zlib/zlib.h \
		zlib/zutil.h \

ZLIB_SRC= zlib/adler32.c \
		zlib/compress.c \
		zlib/crc32.c \
		zlib/deflate.c \
		zlib/infback.c \
		zlib/inffast.c \
		zlib/inflate.c \
		zlib/inftrees.c \
		zlib/trees.c \
		zlib/uncompr.c \
		zlib/zutil.c


ZLIB_OBJ= zlib/adler32.o \
		zlib/compress.o \
		zlib/crc32.o \
		zlib/deflate.o \
		zlib/infback.o \
		zlib/inffast.o \
		zlib/inflate.o \
		zlib/inftrees.o \
		zlib/trees.o \
		zlib/uncompr.o \
		zlib/zutil.o


EXAMPLE_OBJ = example_read/example_read.o \
      example_write/example_write.o \
      example_dump/example_dump.o \
      example_brep/example_brep.o \
      example_fileit/example_fileit.o \
      example_userdata/example_ud.o \
      example_userdata/example_userdata.o \
      example_roundtrip/example_roundtrip.o

EXAMPLES = example_read/example_read \
      example_write/example_write \
      example_dump/example_dump \
      example_brep/example_brep \
      example_fileit/example_fileit \
      example_roundtrip/example_roundtrip \
      example_userdata/example_userdata

EXAMPLE_GL_OBJ = example_gl/example_gl.o \
      opennurbs_gl.o
      
EXAMPLE_GL = example_gl/example_gl

all : $(OPENNURBS_LIB_FILE) $(EXAMPLES)

gl : $(OPENNURBS_LIB_FILE) $(EXAMPLE_GL)

opennurbs_gl.o : opennurbs_gl.h $(ON_INC)

example_userdata/example_ud.o : example_userdata/example_ud.h $(ON_INC)

$(ON_OBJ) : $(ON_INC)

$(ZLIB_OBJ) : $(ZLIB_INC)

$(OPENNURBS_LIB_FILE) : $(ON_OBJ) $(ZLIB_OBJ)
	-$(RM) $@
	$(AR) $@ $(ON_OBJ) $(ZLIB_OBJ)  
	$(RANLIB) $@


example_read/example_read : example_read/example_read.o example_userdata/example_ud.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_read/example_read.o example_userdata/example_ud.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

example_write/example_write : example_write/example_write.o example_userdata/example_ud.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_write/example_write.o example_userdata/example_ud.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

example_brep/example_brep : example_brep/example_brep.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_brep/example_brep.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

example_fileit/example_fileit : example_fileit/example_fileit.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_fileit/example_fileit.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

example_dump/example_dump : example_dump/example_dump.o
	$(LINK) $(LINKFLAGS) example_dump/example_dump.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

example_userdata/example_userdata : example_userdata/example_userdata.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_userdata/example_userdata.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

example_roundtrip/example_roundtrip : example_roundtrip/example_roundtrip.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_roundtrip/example_roundtrip.o -L. -l$(OPENNURBS_LIB_NAME) -lm -o $@

LINK_GL = -lglaux -lglu -lgl
LINK_XWIN = -lXmu -lXi -lXext -lX11

$(EXAMPLE_GL) : example_gl/example_gl.o ./opennurbs_gl.o $(OPENNURBS_LIB_FILE)
	$(LINK) $(LINKFLAGS) example_gl/example_gl.o ./opennurbs_gl.o -L. -l$(OPENNURBS_LIB_NAME) $(LINK_GL) $(LINK_XWIN) -lm -o $@

clean :
	-$(RM) $(OPENNURBS_LIB_FILE)
	-$(RM) $(ON_OBJ)
	-$(RM) $(ZLIB_OBJ)
	-$(RM) $(EXAMPLE_OBJ)
	-$(RM) $(EXAMPLE_GL_OBJ)
	-$(RM) $(EXAMPLES)
	-$(RM) $(EXAMPLE_GL)
