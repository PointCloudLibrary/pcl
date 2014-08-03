/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <pcl/apps/in_hand_scanner/opengl_viewer.h>

#include <cmath>
#include <typeinfo>
#include <cstdlib>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif

#include <QtOpenGL>

#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp> // TODO: PointIHS is not registered
#include <pcl/apps/in_hand_scanner/visibility_confidence.h>

////////////////////////////////////////////////////////////////////////////////
// FaceVertexMesh
////////////////////////////////////////////////////////////////////////////////

pcl::ihs::detail::FaceVertexMesh::FaceVertexMesh ()
  : vertices       (),
    triangles      (),
    transformation (Eigen::Isometry3d::Identity ())
{
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::detail::FaceVertexMesh::FaceVertexMesh (const Mesh& mesh, const Eigen::Isometry3d& T)
  : vertices       (mesh.getVertexDataCloud ()),
    triangles      (),
    transformation (T)
{
  if (typeid (Mesh::MeshTag) != typeid (pcl::geometry::TriangleMeshTag))
  {
    std::cerr << "In opengl_viewer.cpp: Only triangle meshes are currently supported!\n";
    exit (EXIT_FAILURE);
  }

  for (CloudIHS::iterator it=vertices.begin (); it!=vertices.end (); ++it)
  {
    std::swap (it->r, it->b);
  }

  triangles.reserve (mesh.sizeFaces ());
  pcl::ihs::detail::FaceVertexMesh::Triangle triangle;

  for (unsigned int i=0; i<mesh.sizeFaces (); ++i)
  {
    Mesh::VertexAroundFaceCirculator circ = mesh.getVertexAroundFaceCirculator (Mesh::FaceIndex (i));
    triangle.first  = (circ++).getTargetIndex ().get ();
    triangle.second = (circ++).getTargetIndex ().get ();
    triangle.third  = (circ  ).getTargetIndex ().get ();

    triangles.push_back (triangle);
  }
}

////////////////////////////////////////////////////////////////////////////////
// OpenGLViewer
////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OpenGLViewer::OpenGLViewer (QWidget* parent)
  : QGLWidget            (parent),
    mutex_vis_           (),
    timer_vis_           (new QTimer (this)),
    colormap_            (Colormap::Constant (255)),
    vis_conf_norm_       (1),
    drawn_meshes_        (),
    mesh_representation_ (MR_POINTS),
    coloring_            (COL_RGB),
    draw_box_            (false),
    box_coefficients_    (),
    scaling_factor_      (1.),
    R_cam_               (1., 0., 0., 0.),
    t_cam_               (0., 0., 0.),
    cam_pivot_           (0., 0., 0.),
    cam_pivot_id_        (""),
    mouse_pressed_begin_ (false),
    x_prev_              (0),
    y_prev_              (0)
{
  // Timer: Defines the update rate for the visualization
  connect (timer_vis_.get (), SIGNAL (timeout ()), this, SLOT (timerCallback ()));
  timer_vis_->start (33);

  // http://doc.qt.digia.com/qt/opengl-overpainting.html
  QWidget::setAutoFillBackground (false);

  // http://doc.qt.digia.com/qt/qwidget.html#keyPressEvent
  this->setFocusPolicy (Qt::StrongFocus);

  // http://doc.qt.digia.com/qt/qmetatype.html#qRegisterMetaType
  qRegisterMetaType <pcl::ihs::OpenGLViewer::MeshRepresentation> ("MeshRepresentation");
  qRegisterMetaType <pcl::ihs::OpenGLViewer::Coloring>           ("Coloring");

  //////////////////////////////////////////////////////////////////////////////
  // Code to generate the colormap (I don't want to link against vtk just for the colormap).
  //////////////////////////////////////////////////////////////////////////////

  //#include <cstdlib>
  //#include <iomanip>

  //#include <vtkColorTransferFunction.h>
  //#include <vtkSmartPointer.h>

  //int
  //main ()
  //{
  //  static const unsigned int n = 256;
  //  // double rgb_1 [] = { 59./255., 76./255., 192./255.};
  //  // double rgb_2 [] = {180./255.,  4./255.,  38./255.};
  //  double rgb_1 [] = {180./255.,   0./255.,  0./255.};
  //  double rgb_2 [] = {  0./255., 180./255.,  0./255.};

  //  vtkSmartPointer <vtkColorTransferFunction> ctf = vtkColorTransferFunction::New ();
  //  ctf->SetColorSpaceToDiverging ();
  //  ctf->AddRGBPoint (  0., rgb_1 [0], rgb_1 [1], rgb_1 [2]);
  //  ctf->AddRGBPoint (  1., rgb_2 [0], rgb_2 [1], rgb_2 [2]);
  //  ctf->Build ();

  //  const unsigned char* colormap = ctf->GetTable (0., 1., n);

  //  for (unsigned int i=0; i<n; ++i)
  //  {
  //    const unsigned int r = static_cast <unsigned int> (colormap [3 * i    ]);
  //    const unsigned int g = static_cast <unsigned int> (colormap [3 * i + 1]);
  //    const unsigned int b = static_cast <unsigned int> (colormap [3 * i + 2]);

  //    std::cerr << "colormap_.col ("
  //              << std::setw (3) << i << ") = Color ("
  //              << std::setw (3) << r << ", "
  //              << std::setw (3) << g << ", "
  //              << std::setw (3) << b << ");\n";
  //  }

  //  return (EXIT_SUCCESS);
  //}

  colormap_.col (  0) = Color (180,   0,   0);
  colormap_.col (  1) = Color (182,   9,   1);
  colormap_.col (  2) = Color (184,  17,   1);
  colormap_.col (  3) = Color (186,  24,   2);
  colormap_.col (  4) = Color (188,  29,   2);
  colormap_.col (  5) = Color (190,  33,   3);
  colormap_.col (  6) = Color (192,  38,   4);
  colormap_.col (  7) = Color (194,  42,   5);
  colormap_.col (  8) = Color (196,  46,   6);
  colormap_.col (  9) = Color (197,  49,   7);
  colormap_.col ( 10) = Color (199,  53,   9);
  colormap_.col ( 11) = Color (201,  56,  10);
  colormap_.col ( 12) = Color (203,  59,  12);
  colormap_.col ( 13) = Color (205,  63,  13);
  colormap_.col ( 14) = Color (207,  66,  15);
  colormap_.col ( 15) = Color (208,  69,  17);
  colormap_.col ( 16) = Color (210,  72,  18);
  colormap_.col ( 17) = Color (212,  75,  20);
  colormap_.col ( 18) = Color (214,  78,  21);
  colormap_.col ( 19) = Color (215,  81,  23);
  colormap_.col ( 20) = Color (217,  84,  25);
  colormap_.col ( 21) = Color (219,  87,  26);
  colormap_.col ( 22) = Color (221,  89,  28);
  colormap_.col ( 23) = Color (222,  92,  30);
  colormap_.col ( 24) = Color (224,  95,  32);
  colormap_.col ( 25) = Color (225,  98,  33);
  colormap_.col ( 26) = Color (227, 101,  35);
  colormap_.col ( 27) = Color (229, 103,  37);
  colormap_.col ( 28) = Color (230, 106,  39);
  colormap_.col ( 29) = Color (232, 109,  40);
  colormap_.col ( 30) = Color (233, 112,  42);
  colormap_.col ( 31) = Color (235, 114,  44);
  colormap_.col ( 32) = Color (236, 117,  46);
  colormap_.col ( 33) = Color (238, 120,  48);
  colormap_.col ( 34) = Color (239, 122,  50);
  colormap_.col ( 35) = Color (241, 125,  52);
  colormap_.col ( 36) = Color (242, 127,  54);
  colormap_.col ( 37) = Color (244, 130,  56);
  colormap_.col ( 38) = Color (245, 133,  58);
  colormap_.col ( 39) = Color (246, 135,  60);
  colormap_.col ( 40) = Color (248, 138,  62);
  colormap_.col ( 41) = Color (249, 140,  64);
  colormap_.col ( 42) = Color (250, 143,  66);
  colormap_.col ( 43) = Color (252, 145,  68);
  colormap_.col ( 44) = Color (253, 148,  70);
  colormap_.col ( 45) = Color (254, 150,  73);
  colormap_.col ( 46) = Color (255, 153,  75);
  colormap_.col ( 47) = Color (255, 154,  76);
  colormap_.col ( 48) = Color (255, 156,  78);
  colormap_.col ( 49) = Color (255, 158,  80);
  colormap_.col ( 50) = Color (255, 159,  82);
  colormap_.col ( 51) = Color (255, 161,  84);
  colormap_.col ( 52) = Color (255, 163,  86);
  colormap_.col ( 53) = Color (255, 164,  88);
  colormap_.col ( 54) = Color (255, 166,  90);
  colormap_.col ( 55) = Color (255, 168,  92);
  colormap_.col ( 56) = Color (255, 169,  93);
  colormap_.col ( 57) = Color (255, 171,  95);
  colormap_.col ( 58) = Color (255, 172,  97);
  colormap_.col ( 59) = Color (255, 174,  99);
  colormap_.col ( 60) = Color (255, 176, 101);
  colormap_.col ( 61) = Color (255, 177, 103);
  colormap_.col ( 62) = Color (255, 179, 105);
  colormap_.col ( 63) = Color (255, 180, 107);
  colormap_.col ( 64) = Color (255, 182, 109);
  colormap_.col ( 65) = Color (255, 183, 111);
  colormap_.col ( 66) = Color (255, 185, 113);
  colormap_.col ( 67) = Color (255, 186, 115);
  colormap_.col ( 68) = Color (255, 188, 117);
  colormap_.col ( 69) = Color (255, 189, 119);
  colormap_.col ( 70) = Color (255, 191, 122);
  colormap_.col ( 71) = Color (255, 192, 124);
  colormap_.col ( 72) = Color (255, 194, 126);
  colormap_.col ( 73) = Color (255, 195, 128);
  colormap_.col ( 74) = Color (255, 196, 130);
  colormap_.col ( 75) = Color (255, 198, 132);
  colormap_.col ( 76) = Color (255, 199, 134);
  colormap_.col ( 77) = Color (255, 201, 136);
  colormap_.col ( 78) = Color (255, 202, 139);
  colormap_.col ( 79) = Color (255, 203, 141);
  colormap_.col ( 80) = Color (255, 205, 143);
  colormap_.col ( 81) = Color (255, 206, 145);
  colormap_.col ( 82) = Color (255, 207, 147);
  colormap_.col ( 83) = Color (255, 209, 149);
  colormap_.col ( 84) = Color (255, 210, 152);
  colormap_.col ( 85) = Color (255, 211, 154);
  colormap_.col ( 86) = Color (255, 213, 156);
  colormap_.col ( 87) = Color (255, 214, 158);
  colormap_.col ( 88) = Color (255, 215, 161);
  colormap_.col ( 89) = Color (255, 216, 163);
  colormap_.col ( 90) = Color (255, 218, 165);
  colormap_.col ( 91) = Color (255, 219, 168);
  colormap_.col ( 92) = Color (255, 220, 170);
  colormap_.col ( 93) = Color (255, 221, 172);
  colormap_.col ( 94) = Color (255, 223, 175);
  colormap_.col ( 95) = Color (255, 224, 177);
  colormap_.col ( 96) = Color (255, 225, 179);
  colormap_.col ( 97) = Color (255, 226, 182);
  colormap_.col ( 98) = Color (255, 227, 184);
  colormap_.col ( 99) = Color (255, 228, 186);
  colormap_.col (100) = Color (255, 230, 189);
  colormap_.col (101) = Color (255, 231, 191);
  colormap_.col (102) = Color (255, 232, 193);
  colormap_.col (103) = Color (255, 233, 196);
  colormap_.col (104) = Color (255, 234, 198);
  colormap_.col (105) = Color (255, 235, 201);
  colormap_.col (106) = Color (255, 236, 203);
  colormap_.col (107) = Color (255, 237, 205);
  colormap_.col (108) = Color (255, 238, 208);
  colormap_.col (109) = Color (255, 239, 210);
  colormap_.col (110) = Color (255, 240, 213);
  colormap_.col (111) = Color (255, 241, 215);
  colormap_.col (112) = Color (255, 242, 218);
  colormap_.col (113) = Color (255, 243, 220);
  colormap_.col (114) = Color (255, 244, 222);
  colormap_.col (115) = Color (255, 245, 225);
  colormap_.col (116) = Color (255, 246, 227);
  colormap_.col (117) = Color (255, 247, 230);
  colormap_.col (118) = Color (255, 248, 232);
  colormap_.col (119) = Color (255, 249, 235);
  colormap_.col (120) = Color (255, 249, 237);
  colormap_.col (121) = Color (255, 250, 239);
  colormap_.col (122) = Color (255, 251, 242);
  colormap_.col (123) = Color (255, 252, 244);
  colormap_.col (124) = Color (255, 253, 247);
  colormap_.col (125) = Color (255, 253, 249);
  colormap_.col (126) = Color (255, 254, 251);
  colormap_.col (127) = Color (255, 255, 254);
  colormap_.col (128) = Color (255, 255, 254);
  colormap_.col (129) = Color (254, 255, 253);
  colormap_.col (130) = Color (253, 255, 252);
  colormap_.col (131) = Color (252, 255, 250);
  colormap_.col (132) = Color (251, 255, 249);
  colormap_.col (133) = Color (250, 255, 248);
  colormap_.col (134) = Color (249, 255, 246);
  colormap_.col (135) = Color (248, 255, 245);
  colormap_.col (136) = Color (247, 255, 244);
  colormap_.col (137) = Color (246, 255, 242);
  colormap_.col (138) = Color (245, 255, 241);
  colormap_.col (139) = Color (244, 255, 240);
  colormap_.col (140) = Color (243, 255, 238);
  colormap_.col (141) = Color (242, 255, 237);
  colormap_.col (142) = Color (241, 255, 236);
  colormap_.col (143) = Color (240, 255, 235);
  colormap_.col (144) = Color (239, 255, 233);
  colormap_.col (145) = Color (238, 255, 232);
  colormap_.col (146) = Color (237, 255, 231);
  colormap_.col (147) = Color (236, 255, 229);
  colormap_.col (148) = Color (235, 255, 228);
  colormap_.col (149) = Color (234, 255, 227);
  colormap_.col (150) = Color (234, 255, 225);
  colormap_.col (151) = Color (233, 255, 224);
  colormap_.col (152) = Color (232, 255, 223);
  colormap_.col (153) = Color (231, 255, 221);
  colormap_.col (154) = Color (230, 255, 220);
  colormap_.col (155) = Color (229, 255, 219);
  colormap_.col (156) = Color (228, 255, 218);
  colormap_.col (157) = Color (227, 255, 216);
  colormap_.col (158) = Color (226, 255, 215);
  colormap_.col (159) = Color (225, 255, 214);
  colormap_.col (160) = Color (224, 255, 212);
  colormap_.col (161) = Color (223, 255, 211);
  colormap_.col (162) = Color (222, 255, 210);
  colormap_.col (163) = Color (221, 255, 208);
  colormap_.col (164) = Color (220, 255, 207);
  colormap_.col (165) = Color (219, 255, 206);
  colormap_.col (166) = Color (218, 255, 204);
  colormap_.col (167) = Color (217, 255, 203);
  colormap_.col (168) = Color (216, 255, 202);
  colormap_.col (169) = Color (215, 255, 201);
  colormap_.col (170) = Color (214, 255, 199);
  colormap_.col (171) = Color (213, 255, 198);
  colormap_.col (172) = Color (211, 255, 197);
  colormap_.col (173) = Color (210, 255, 195);
  colormap_.col (174) = Color (209, 255, 194);
  colormap_.col (175) = Color (208, 255, 193);
  colormap_.col (176) = Color (207, 255, 191);
  colormap_.col (177) = Color (206, 255, 190);
  colormap_.col (178) = Color (205, 255, 188);
  colormap_.col (179) = Color (204, 255, 187);
  colormap_.col (180) = Color (203, 255, 186);
  colormap_.col (181) = Color (202, 255, 184);
  colormap_.col (182) = Color (201, 255, 183);
  colormap_.col (183) = Color (199, 255, 182);
  colormap_.col (184) = Color (198, 255, 180);
  colormap_.col (185) = Color (197, 255, 179);
  colormap_.col (186) = Color (196, 255, 177);
  colormap_.col (187) = Color (195, 255, 176);
  colormap_.col (188) = Color (194, 255, 174);
  colormap_.col (189) = Color (192, 255, 173);
  colormap_.col (190) = Color (191, 255, 172);
  colormap_.col (191) = Color (190, 255, 170);
  colormap_.col (192) = Color (189, 255, 169);
  colormap_.col (193) = Color (188, 255, 167);
  colormap_.col (194) = Color (186, 255, 166);
  colormap_.col (195) = Color (185, 255, 164);
  colormap_.col (196) = Color (184, 255, 163);
  colormap_.col (197) = Color (183, 255, 161);
  colormap_.col (198) = Color (181, 255, 160);
  colormap_.col (199) = Color (180, 255, 158);
  colormap_.col (200) = Color (179, 255, 157);
  colormap_.col (201) = Color (177, 255, 155);
  colormap_.col (202) = Color (176, 255, 154);
  colormap_.col (203) = Color (175, 255, 152);
  colormap_.col (204) = Color (173, 255, 150);
  colormap_.col (205) = Color (172, 255, 149);
  colormap_.col (206) = Color (170, 255, 147);
  colormap_.col (207) = Color (169, 255, 145);
  colormap_.col (208) = Color (166, 253, 143);
  colormap_.col (209) = Color (164, 252, 141);
  colormap_.col (210) = Color (162, 251, 138);
  colormap_.col (211) = Color (159, 250, 136);
  colormap_.col (212) = Color (157, 248, 134);
  colormap_.col (213) = Color (155, 247, 131);
  colormap_.col (214) = Color (152, 246, 129);
  colormap_.col (215) = Color (150, 245, 127);
  colormap_.col (216) = Color (148, 243, 124);
  colormap_.col (217) = Color (145, 242, 122);
  colormap_.col (218) = Color (143, 240, 119);
  colormap_.col (219) = Color (140, 239, 117);
  colormap_.col (220) = Color (138, 238, 114);
  colormap_.col (221) = Color (135, 236, 112);
  colormap_.col (222) = Color (133, 235, 110);
  colormap_.col (223) = Color (130, 233, 107);
  colormap_.col (224) = Color (128, 232, 105);
  colormap_.col (225) = Color (125, 230, 102);
  colormap_.col (226) = Color (122, 229, 100);
  colormap_.col (227) = Color (120, 227,  97);
  colormap_.col (228) = Color (117, 226,  94);
  colormap_.col (229) = Color (114, 224,  92);
  colormap_.col (230) = Color (111, 223,  89);
  colormap_.col (231) = Color (109, 221,  87);
  colormap_.col (232) = Color (106, 220,  84);
  colormap_.col (233) = Color (103, 218,  82);
  colormap_.col (234) = Color (100, 217,  79);
  colormap_.col (235) = Color ( 97, 215,  76);
  colormap_.col (236) = Color ( 94, 213,  73);
  colormap_.col (237) = Color ( 91, 212,  71);
  colormap_.col (238) = Color ( 88, 210,  68);
  colormap_.col (239) = Color ( 85, 208,  65);
  colormap_.col (240) = Color ( 82, 207,  62);
  colormap_.col (241) = Color ( 78, 205,  59);
  colormap_.col (242) = Color ( 75, 203,  57);
  colormap_.col (243) = Color ( 71, 201,  54);
  colormap_.col (244) = Color ( 68, 200,  50);
  colormap_.col (245) = Color ( 64, 198,  47);
  colormap_.col (246) = Color ( 60, 196,  44);
  colormap_.col (247) = Color ( 56, 195,  41);
  colormap_.col (248) = Color ( 52, 193,  37);
  colormap_.col (249) = Color ( 47, 191,  33);
  colormap_.col (250) = Color ( 42, 189,  29);
  colormap_.col (251) = Color ( 37, 187,  25);
  colormap_.col (252) = Color ( 31, 186,  20);
  colormap_.col (253) = Color ( 24, 184,  15);
  colormap_.col (254) = Color ( 14, 182,   7);
  colormap_.col (255) = Color (  0, 180,   0);
}

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::OpenGLViewer::~OpenGLViewer ()
{
  this->stopTimer ();
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::addMesh (const MeshConstPtr& mesh, const std::string& id, const Eigen::Isometry3d& T)
{
  if (!mesh)
  {
    std::cerr << "ERROR in opengl_viewer.cpp: Input mesh is not valid.\n";
    return (false);
  }

  boost::mutex::scoped_lock lock (mutex_vis_);

  if (this->getMeshIsAdded (id))
    drawn_meshes_ [id] = FaceVertexMeshPtr (new FaceVertexMesh (*mesh, T));
  else
    drawn_meshes_.insert (std::make_pair (id, FaceVertexMeshPtr (new FaceVertexMesh (*mesh, T))));

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::addMesh (const CloudXYZRGBNormalConstPtr& cloud, const std::string& id, const Eigen::Isometry3d& T)
{
  if (!cloud)
  {
    std::cerr << "ERROR in opengl_viewer.cpp: Input cloud is not valid.\n";
    return (false);
  }
  if (!cloud->isOrganized ())
  {
    std::cerr << "ERROR in opengl_viewer.cpp: Input cloud is not organized.\n";
    return (false);
  }

  // Convert the cloud to a mesh using the following pattern
  // 2 - 1 //
  // | / | //
  // 3 - 0 //
  const int w        = cloud->width;
  const int h        = cloud->height;
  const int offset_1 = -w;
  const int offset_2 = -w - 1;
  const int offset_3 =    - 1;

  FaceVertexMeshPtr mesh (new FaceVertexMesh ());
  mesh->transformation = T;

  std::vector <int> indices (w * h, -1); // Map the original indices to the vertex indices.
  CloudIHS& vertices = mesh->vertices;
  std::vector <FaceVertexMesh::Triangle>& triangles = mesh->triangles;
  vertices.reserve (cloud->size ());
  triangles.reserve (2 * (w-1) * (h-1));

  // Helper functor
  struct AddVertex
  {
    inline int operator () (const PointXYZRGBNormal& pt, CloudIHS& vertices, int& ind_o) const
    {
      if (ind_o == -1)
      {
        ind_o = vertices.size ();
        vertices.push_back (PointIHS (pt, -pt.normal_z));
        std::swap (vertices.back ().r, vertices.back ().b);
      }
      return (ind_o);
    }
  };
  AddVertex addVertex;

  int ind_o_0, ind_o_1, ind_o_2, ind_o_3; // Index into the organized cloud.
  int ind_v_0, ind_v_1, ind_v_2, ind_v_3; // Index to the new vertices.

  for (int r=1; r<h; ++r)
  {
    for (int c=1; c<w; ++c)
    {
      ind_o_0 = r*w + c;
      ind_o_1 = ind_o_0 + offset_1;
      ind_o_2 = ind_o_0 + offset_2;
      ind_o_3 = ind_o_0 + offset_3;

      const PointXYZRGBNormal& pt_0 = cloud->operator [] (ind_o_0);
      const PointXYZRGBNormal& pt_1 = cloud->operator [] (ind_o_1);
      const PointXYZRGBNormal& pt_2 = cloud->operator [] (ind_o_2);
      const PointXYZRGBNormal& pt_3 = cloud->operator [] (ind_o_3);

      if (!boost::math::isnan (pt_1.x) && !boost::math::isnan (pt_3.x))
      {
        if (!boost::math::isnan (pt_2.x)) // 1-2-3 is valid
        {
          if (std::abs (pt_1.z - pt_2.z) < 1 &&
              std::abs (pt_1.z - pt_3.z) < 1 &&
              std::abs (pt_2.z - pt_3.z) < 1) // distance threshold
          {
            ind_v_1 = addVertex (pt_1, vertices, indices [ind_o_1]);
            ind_v_2 = addVertex (pt_2, vertices, indices [ind_o_2]);
            ind_v_3 = addVertex (pt_3, vertices, indices [ind_o_3]);

            triangles.push_back (FaceVertexMesh::Triangle (ind_v_1, ind_v_2, ind_v_3));
          }
        }
        if (!boost::math::isnan (pt_0.x)) // 0-1-3 is valid
        {
          if (std::abs (pt_0.z - pt_1.z) < 1 &&
              std::abs (pt_0.z - pt_3.z) < 1 &&
              std::abs (pt_1.z - pt_3.z) < 1) // distance threshold
          {
            ind_v_1 = addVertex (pt_1, vertices, indices [ind_o_1]);
            ind_v_3 = addVertex (pt_3, vertices, indices [ind_o_3]);
            ind_v_0 = addVertex (pt_0, vertices, indices [ind_o_0]);

            triangles.push_back (FaceVertexMesh::Triangle (ind_v_1, ind_v_3, ind_v_0));
          }
        }
      }
    }
  }

  // Finally add the mesh.
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (this->getMeshIsAdded (id))
    drawn_meshes_ [id] = mesh;
  else
    drawn_meshes_.insert (std::make_pair (id, mesh));

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::removeMesh (const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  if (!this->getMeshIsAdded (id)) return (false);

  drawn_meshes_.erase (id);

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::removeAllMeshes ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  drawn_meshes_.clear ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setBoxCoefficients (const BoxCoefficients& coeffs)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  box_coefficients_ = coeffs;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setDrawBox (const bool enabled)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  draw_box_ = enabled;
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::getDrawBox () const
{
  return (draw_box_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setPivot (const Eigen::Vector3d& pivot)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  cam_pivot_ = pivot;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setPivot (const std::string& id)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  cam_pivot_id_ = id;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::stopTimer ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  if (timer_vis_)
  {
    timer_vis_->stop ();
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setVisibilityConfidenceNormalization (const float vis_conf_norm)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  vis_conf_norm_ = vis_conf_norm < 1 ? 1 : vis_conf_norm;
}

////////////////////////////////////////////////////////////////////////////////

QSize
pcl::ihs::OpenGLViewer::minimumSizeHint () const
{
  return (QSize (160, 120));
}

////////////////////////////////////////////////////////////////////////////////

QSize
pcl::ihs::OpenGLViewer::sizeHint () const
{
  return (QSize (640, 480));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setScalingFactor (const double scale)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  scaling_factor_ = scale;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::timerCallback ()
{
  QWidget::update ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::resetCamera ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  R_cam_ = Eigen::Quaterniond (1., 0., 0., 0.);
  t_cam_ = Eigen::Vector3d    (0., 0., 0.);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::toggleMeshRepresentation ()
{
  switch (mesh_representation_)
  {
    case MR_POINTS: this->setMeshRepresentation (MR_EDGES);  break;
    case MR_EDGES:  this->setMeshRepresentation (MR_FACES);  break;
    case MR_FACES:  this->setMeshRepresentation (MR_POINTS); break;
    default: std::cerr << "ERROR in opengl_viewer.cpp: Unknown mesh representation\n"; exit (EXIT_FAILURE);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setMeshRepresentation (const MeshRepresentation& representation)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  switch (mesh_representation_)
  {
    case MR_POINTS: std::cerr << "Drawing the points.\n";   break;
    case MR_EDGES:  std::cerr << "Drawing the edges.\n";  break;
    case MR_FACES:  std::cerr << "Drawing the faces.\n"; break;
    default: std::cerr << "ERROR in opengl_viewer.cpp: Unknown mesh representation\n"; exit (EXIT_FAILURE);
  }

  mesh_representation_ = representation;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::toggleColoring ()
{
  switch (coloring_)
  {
    case COL_RGB:       this->setColoring (COL_ONE_COLOR); break;
    case COL_ONE_COLOR: this->setColoring (COL_VISCONF);   break;
    case COL_VISCONF:   this->setColoring (COL_RGB);       break;
    default: std::cerr << "ERROR in opengl_viewer.cpp: Unknown coloring\n"; exit (EXIT_FAILURE);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setColoring (const Coloring& coloring)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  switch (coloring)
  {
    case COL_RGB:       std::cerr << "Coloring according to the rgb values.\n";            break;
    case COL_ONE_COLOR: std::cerr << "Use one color for all points.\n";                    break;
    case COL_VISCONF:   std::cerr << "Coloring according to the visibility confidence.\n"; break;
    default: std::cerr << "ERROR in opengl_viewer.cpp: Unknown coloring\n"; exit (EXIT_FAILURE);
  }
  coloring_ = coloring;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::paintEvent (QPaintEvent* /*event*/)
{
  this->calcPivot ();
  this->makeCurrent ();

  // Clear information from the last draw
  glClearColor (0.f, 0.f, 0.f, 0.f);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  this->setupViewport (this->width (), this->height ());

  // Move light with camera (see example 5-7)
  // http://www.glprogramming.com/red/chapter05.html
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // light 0 (directional)
  glLightfv (GL_LIGHT0, GL_AMBIENT , Eigen::Vector4f (0.1f, 0.1f, 0.1f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT0, GL_DIFFUSE , Eigen::Vector4f (0.6f, 0.6f, 0.6f, 1.0f).eval ().data () );
  glLightfv (GL_LIGHT0, GL_SPECULAR, Eigen::Vector4f (0.2f, 0.2f, 0.2f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT0, GL_POSITION, Eigen::Vector4f (0.3f, 0.5f, 0.8f, 0.0f).normalized ().eval ().data ());

  // light 1 (directional)
  glLightfv (GL_LIGHT1, GL_AMBIENT , Eigen::Vector4f ( 0.0f, 0.0f, 0.0f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT1, GL_DIFFUSE , Eigen::Vector4f ( 0.3f, 0.3f, 0.3f, 1.0f).eval ().data () );
  glLightfv (GL_LIGHT1, GL_SPECULAR, Eigen::Vector4f ( 0.1f, 0.1f, 0.1f, 1.0f).eval ().data ());
  glLightfv (GL_LIGHT1, GL_POSITION, Eigen::Vector4f (-0.3f, 0.5f, 0.8f, 0.0f).normalized ().eval ().data ());

  // Material
  glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glMaterialfv    (GL_FRONT, GL_SPECULAR , Eigen::Vector4f (0.1f, 0.1f, 0.1f, 1.0f).eval ().data ());
  glMaterialf     (GL_FRONT, GL_SHININESS, 25.f);

  glEnable (GL_DEPTH_TEST);
  glEnable (GL_NORMALIZE);
  glEnable (GL_COLOR_MATERIAL);
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  glEnable (GL_LIGHT1);

  // Projection matrix
  glMatrixMode   (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (43., 4./3., 0.01 / scaling_factor_, 10. / scaling_factor_);
  glMatrixMode   (GL_MODELVIEW);

  // ModelView matrix
  Eigen::Quaterniond R_cam;
  Eigen::Vector3d    t_cam;
  {
    boost::mutex::scoped_lock lock (mutex_vis_);
    R_cam = R_cam_;
    t_cam = t_cam_;
  }

  const Eigen::Vector3d o  = Eigen::Vector3d::Zero  ();
  const Eigen::Vector3d ey = Eigen::Vector3d::UnitY ();
  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ ();

  const Eigen::Vector3d eye    =  R_cam * o  + t_cam;
  const Eigen::Vector3d center =  R_cam * ez + t_cam;
  const Eigen::Vector3d up     = (R_cam * -ey).normalized ();

  glMatrixMode (GL_MODELVIEW);
  gluLookAt (eye.   x (), eye.   y (), eye.   z (),
             center.x (), center.y (), center.z (),
             up.    x (), up.    y (), up.    z ());

  // Draw everything
  this->drawMeshes ();

  glDisable (GL_LIGHTING); // This is needed so the color is right.
  this->drawBox ();
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::OpenGLViewer::getMeshIsAdded (const std::string& id)
{
  // boost::mutex::scoped_lock lock (mutex_vis_);
  return (drawn_meshes_.find (id) != drawn_meshes_.end ());
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::calcPivot ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  if (this->getMeshIsAdded (cam_pivot_id_))
  {
    Eigen::Vector4f pivot;
    const FaceVertexMeshConstPtr mesh = drawn_meshes_ [cam_pivot_id_];

    if (pcl::compute3DCentroid (mesh->vertices, pivot))
    {
      const Eigen::Vector3d p = mesh->transformation * pivot.head <3> ().cast <double> ();
      lock.unlock ();
      this->setPivot (p);
    }
  }
  cam_pivot_id_.clear ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::drawMeshes ()
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  glEnableClientState (GL_VERTEX_ARRAY);
  glEnableClientState (GL_NORMAL_ARRAY);
  switch (coloring_)
  {
    case COL_RGB:       glEnableClientState  (GL_COLOR_ARRAY); break;
    case COL_ONE_COLOR: glDisableClientState (GL_COLOR_ARRAY); break;
    case COL_VISCONF:   glEnableClientState  (GL_COLOR_ARRAY); break;
    default: std::cerr << "ERROR in opengl_viewer.cpp: Unknown coloring\n"; exit (EXIT_FAILURE);
  }
  switch (mesh_representation_)
  {
    case MR_POINTS:                                             break;
    case MR_EDGES:  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE); break;
    case MR_FACES:  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL); break;
    default: std::cerr << "ERROR in opengl_viewer.cpp: Unknown mesh representation\n"; exit (EXIT_FAILURE);
  }

  for (FaceVertexMeshMap::const_iterator it=drawn_meshes_.begin (); it!=drawn_meshes_.end (); ++it)
  {
    if (it->second && !it->second->vertices.empty ())
    {
      const FaceVertexMesh& mesh = *it->second;

      glVertexPointer (3, GL_FLOAT, sizeof (PointIHS), &(mesh.vertices [0].x       ));
      glNormalPointer (   GL_FLOAT, sizeof (PointIHS), &(mesh.vertices [0].normal_x));

      Colors colors (3, mesh.vertices.size ());

      switch (coloring_)
      {
        case COL_RGB:
        {
          glColorPointer (3, GL_UNSIGNED_BYTE, sizeof (PointIHS), &(mesh.vertices [0].b));
          break;
        }
        case COL_ONE_COLOR:
        {
          glColor3f (.7f, .7f, .7f);
          break;
        }
        case COL_VISCONF:
        {
          for (unsigned int i=0; i<mesh.vertices.size (); ++i)
          {
            const unsigned int n = pcl::ihs::countDirections (mesh.vertices [i].directions);
            const unsigned int index = static_cast <unsigned int> (
                                         static_cast <float> (colormap_.cols ()) *
                                         static_cast <float> (n) / vis_conf_norm_);

            colors.col (i) = colormap_.col (index < 256 ? index : 255);
          }

          glColorPointer (3, GL_UNSIGNED_BYTE, 0, colors.data ());
        }
      }

      glPushMatrix ();
      {
        glMultMatrixd (mesh.transformation.matrix ().data ());

        switch (mesh_representation_)
        {
          case MR_POINTS:
          {
            glDrawArrays (GL_POINTS, 0, mesh.vertices.size ());
            break;
          }
          case MR_EDGES: case MR_FACES:
          {
            glDrawElements (GL_TRIANGLES, 3*mesh.triangles.size (), GL_UNSIGNED_INT, &mesh.triangles [0]);
            break;
          }
        }
      }
      glPopMatrix ();
    }
  }

  glDisableClientState (GL_VERTEX_ARRAY);
  glDisableClientState (GL_NORMAL_ARRAY);
  glDisableClientState (GL_COLOR_ARRAY);
  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::drawBox ()
{
  BoxCoefficients coeffs;
  {
    boost::mutex::scoped_lock lock (mutex_vis_);
    if (draw_box_) coeffs = box_coefficients_;
    else           return;
  }

  glColor3f (1.f, 1.f, 1.f);

  glPushMatrix ();
  {
    glMultMatrixd (coeffs.transformation.matrix ().data ());

    // Front
    glBegin(GL_LINE_STRIP);
    {
      glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_min);
      glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_min);
      glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_min);
      glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_min);
      glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_min);
    }
    glEnd();

    // Back
    glBegin (GL_LINE_STRIP);
    {
      glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_max);
      glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_max);
      glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_max);
      glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_max);
      glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_max);
    }
    glEnd();

    // Sides
    glBegin (GL_LINES);
    {
      glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_min);
      glVertex3f (coeffs.x_min, coeffs.y_min, coeffs.z_max);

      glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_min);
      glVertex3f (coeffs.x_max, coeffs.y_min, coeffs.z_max);

      glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_min);
      glVertex3f (coeffs.x_max, coeffs.y_max, coeffs.z_max);

      glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_min);
      glVertex3f (coeffs.x_min, coeffs.y_max, coeffs.z_max);
    }
    glEnd();
  }
  glPopMatrix ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::initializeGL ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::setupViewport (const int w, const int h)
{
  const float aspect_ratio = 4./3.;

  // Use the biggest possible area of the window to draw to
  //    case 1 (w < w_scaled):        case 2 (w >= w_scaled):
  //      w
  //    |---|         ^               |-------------|  ^
  //    |---| ^       |               |    |   |    |  | h
  //    |   | | h_sc  | h             |-------------|  v
  //    |---| v       |                    <---> w_sc
  //    |---|         v               <----- w ----->
  const float w_scaled = h * aspect_ratio;
  const float h_scaled = w / aspect_ratio;

  if (w < w_scaled)
  {
    glViewport (0, static_cast <GLint> ((h - h_scaled) / 2.f), w, static_cast <GLsizei> (h_scaled));
  }
  else
  {
    glViewport (static_cast <GLint> ((w - w_scaled) / 2.f), 0, static_cast <GLsizei> (w_scaled), h);
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::resizeGL (int w, int h)
{
  this->setupViewport (w, h);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::mousePressEvent (QMouseEvent* /*event*/)
{
  boost::mutex::scoped_lock lock (mutex_vis_);
  mouse_pressed_begin_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::mouseMoveEvent (QMouseEvent* event)
{
  boost::mutex::scoped_lock lock (mutex_vis_);

  if (mouse_pressed_begin_)
  {
    x_prev_ = event->pos ().x ();
    y_prev_ = event->pos ().y ();
    mouse_pressed_begin_ = false;
    return;
  }
  if (event->pos ().x () == x_prev_ && event->pos ().y () == y_prev_) return;
  if (this->width () == 0 || this->height () == 0)                    return;

  const double dx = static_cast <double> (event->pos ().x ()) - static_cast <double> (x_prev_);
  const double dy = static_cast <double> (event->pos ().y ()) - static_cast <double> (y_prev_);
  const double w  = static_cast <double> (this->width ());
  const double h  = static_cast <double> (this->height ());
  const double d  = std::sqrt (w*w + h*h);

  const Eigen::Vector3d o  = Eigen::Vector3d::Zero  ();
  const Eigen::Vector3d ex = Eigen::Vector3d::UnitX ();
  const Eigen::Vector3d ey = Eigen::Vector3d::UnitY ();
  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ ();

  // Scale with the distance between the pivot and camera eye.
  const double scale = std::max ((cam_pivot_ - R_cam_ * o - t_cam_).norm (), .1 / scaling_factor_) / d;

  if (QApplication::mouseButtons () == Qt::LeftButton)
  {
    const double          rot_angle = -7. * std::atan (std::sqrt ((dx*dx + dy*dy)) / d);
    const Eigen::Vector3d rot_axis  = (R_cam_ * ex * dy - R_cam_ * ey * dx).normalized ();

    const Eigen::Quaterniond dR (Eigen::AngleAxisd (rot_angle, rot_axis));
    t_cam_ = dR * (t_cam_ - cam_pivot_) + cam_pivot_;
    R_cam_ = (dR * R_cam_).normalized ();
  }
  else if (QApplication::mouseButtons () == Qt::MiddleButton)
  {
    t_cam_ += 1.3 * scale * Eigen::Vector3d (R_cam_ * (ey * -dy + ex * -dx));
  }
  else if (QApplication::mouseButtons () == Qt::RightButton)
  {
    t_cam_ += 2.6 * scale * Eigen::Vector3d (R_cam_ * (ez * -dy));
  }

  x_prev_ = event->pos ().x ();
  y_prev_ = event->pos ().y ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::OpenGLViewer::wheelEvent (QWheelEvent* event)
{
  if (QApplication::mouseButtons () == Qt::NoButton)
  {
    boost::mutex::scoped_lock lock (mutex_vis_);

    // Scale with the distance between the pivot and camera eye.
    const Eigen::Vector3d o     = Eigen::Vector3d::Zero  ();
    const Eigen::Vector3d ez    = Eigen::Vector3d::UnitZ ();
    const double          w     = static_cast <double> (this->width ());
    const double          h     = static_cast <double> (this->height ());
    const double          d     = std::sqrt (w*w + h*h);
    const double          scale = std::max ((cam_pivot_ - R_cam_ * o - t_cam_).norm (), .1 / scaling_factor_) / d;

    // http://doc.qt.digia.com/qt/qwheelevent.html#delta
    t_cam_ += scale * Eigen::Vector3d (R_cam_ * (ez * static_cast <double> (event->delta ())));
  }
}

////////////////////////////////////////////////////////////////////////////////
