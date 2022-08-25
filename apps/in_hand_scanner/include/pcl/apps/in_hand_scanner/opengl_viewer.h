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

#pragma once

#include <pcl/apps/in_hand_scanner/common_types.h>
#include <pcl/common/time.h>
#include <pcl/memory.h>
#include <pcl/pcl_exports.h>
#include <pcl/pcl_macros.h>

#include <QGLWidget>

#include <iomanip>
#include <mutex>
#include <string>
#include <unordered_map>

namespace pcl {
namespace ihs {
namespace detail {
/** \brief Mesh format more efficient for visualization than the half-edge data
 * structure. \see http://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes
 *
 * \note Only triangles are currently supported.
 */
class FaceVertexMesh {
public:
  class Triangle {
  public:
    Triangle() : first(0), second(0), third(0) {}
    Triangle(const unsigned int first,
             const unsigned int second,
             const unsigned int third)
    : first(first), second(second), third(third)
    {}

    unsigned int first;
    unsigned int second;
    unsigned int third;
  };

  /** \brief Constructor */
  FaceVertexMesh();

  /** \brief Constructor. Converts the input mesh into a face vertex mesh. */
  FaceVertexMesh(const Mesh& mesh, const Eigen::Isometry3d& T);

  using PointIHS = pcl::ihs::PointIHS;
  using CloudIHS = pcl::ihs::CloudIHS;
  using CloudIHSPtr = pcl::ihs::CloudIHSPtr;
  using CloudIHSConstPtr = pcl::ihs::CloudIHSConstPtr;

  CloudIHS vertices;
  std::vector<Triangle> triangles;
  Eigen::Isometry3d transformation;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // End namespace detail

/** \brief Viewer for the in-hand scanner based on Qt and OpenGL.
 *
 * \note Currently you have to derive from this class to use it. Implement the
 * paintEvent: Call the paint event of this class and declare a QPainter.
 */
class PCL_EXPORTS OpenGLViewer : public QGLWidget {
  Q_OBJECT

public:
  using PointXYZRGBNormal = pcl::PointXYZRGBNormal;
  using CloudXYZRGBNormal = pcl::PointCloud<PointXYZRGBNormal>;
  using CloudXYZRGBNormalPtr = CloudXYZRGBNormal::Ptr;
  using CloudXYZRGBNormalConstPtr = CloudXYZRGBNormal::ConstPtr;

  using Mesh = pcl::ihs::Mesh;
  using MeshPtr = pcl::ihs::MeshPtr;
  using MeshConstPtr = pcl::ihs::MeshConstPtr;

  /** \brief How to draw the mesh. */
  enum MeshRepresentation {
    MR_POINTS, /**< Draw the points. */
    MR_EDGES,  /**< Wireframe represen of the mesh. */
    MR_FACES   /**< Draw the faces of the mesh without edges. */
  };

  /** \brief How to color the shapes. */
  enum Coloring {
    COL_RGB,       /**< Coloring according to the rgb values. */
    COL_ONE_COLOR, /**< Use one color for all points. */
    COL_VISCONF    /**< Coloring according to the visibility confidence. */
  };

  /** \brief Coefficients for the wireframe box. */
  class BoxCoefficients {
  public:
    BoxCoefficients()
    : x_min(0)
    , x_max(0)
    , y_min(0)
    , y_max(0)
    , z_min(0)
    , z_max(0)
    , transformation(Eigen::Isometry3d::Identity())
    {}

    BoxCoefficients(const float x_min,
                    const float x_max,
                    const float y_min,
                    const float y_max,
                    const float z_min,
                    const float z_max,
                    const Eigen::Isometry3d& T)
    : x_min(x_min)
    , x_max(x_max)
    , y_min(y_min)
    , y_max(y_max)
    , z_min(z_min)
    , z_max(z_max)
    , transformation(T)
    {}

    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    Eigen::Isometry3d transformation;

  public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief Constructor. */
  explicit OpenGLViewer(QWidget* parent = nullptr);

  /** \brief Destructor. */
  ~OpenGLViewer() override;

  /** \brief Add a mesh to be drawn.
   *
   * \param[in] mesh The input mesh.
   * \param[in] id Unique identifier for the mesh. The internal mesh is replaced by the
   * input mesh if the id already exists.
   * \param[in] T Transformation applied to the mesh. Defaults to an identity
   * transformation.
   *
   * \return true if success.
   *
   * \note Converts the mesh to the internal representation better suited for
   * visualization. Therefore this method takes some time.
   */
  bool
  addMesh(const MeshConstPtr& mesh,
          const std::string& id,
          const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity());

  /** \brief Convert an organized cloud to a mesh and draw it.
   *
   * \param[in] cloud Organized input cloud.
   * \param[in] id Unique identifier for the mesh. The internal mesh is replaced by the
   * converted input mesh if the id already exists.
   * \param[in] T Transformation applied to the mesh. Defaults to an identity
   * transformation.
   *
   * \return true if success.
   *
   * \note This method takes some time for the conversion).
   */
  bool
  addMesh(const CloudXYZRGBNormalConstPtr& cloud,
          const std::string& id,
          const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity());

  /** \brief Remove the mesh with the given id.
   *
   * \param[in] id Identifier of the mesh (results in a failure if the id does not
   * exist).
   *
   * \return true if success.
   */
  bool
  removeMesh(const std::string& id);

  /** \brief Remove all meshes that are currently drawn. */
  void
  removeAllMeshes();

  /** \brief Set the coefficients for the box. */
  void
  setBoxCoefficients(const BoxCoefficients& coeffs);

  /** \brief Enable / disable drawing the box. */
  void
  setDrawBox(const bool enabled);

  /** \brief Check if the box is drawn. */
  bool
  getDrawBox() const;

  /** \brief Set the point around which the camera rotates during mouse navigation. */
  void
  setPivot(const Eigen::Vector3d& pivot);

  /** \brief Searches the given id in the drawn meshes and calculates the pivot as the
   * centroid of the found geometry.
   *
   * \note Returns immediately and computes the pivot in
   * another thread.
   */
  void
  setPivot(const std::string& id);

  /** \brief Stop the visualization timer. */
  void
  stopTimer();

  /** \brief The visibility confidence is normalized with this value (must be greater
   * than 1). */
  void
  setVisibilityConfidenceNormalization(const float vis_conf_norm);

  /** \see http://doc.qt.digia.com/qt/qwidget.html#minimumSizeHint-prop */
  QSize
  minimumSizeHint() const override;

  /** \see http://doc.qt.digia.com/qt/qwidget.html#sizeHint-prop */
  QSize
  sizeHint() const override;

  /** \brief Set the scaling factor to convert from meters to the unit of the drawn
   * files. */
  void
  setScalingFactor(const double scale);

public Q_SLOTS:

  /** \brief Requests the scene to be re-drawn (called periodically from a timer). */
  void
  timerCallback();

  /** \brief Reset the virtual camera position and orientation. */
  void
  resetCamera();

  /** \brief Toggle the mesh representation. */
  void
  toggleMeshRepresentation();

  /** \brief Set the mesh representation. */
  void
  setMeshRepresentation(const MeshRepresentation& representation);

  /** \brief Toggle the coloring mode. */
  void
  toggleColoring();

  /** \brief Set the coloring mode. */
  void
  setColoring(const Coloring& coloring);

protected:
  /** \brief Please have a look at the documentation of calcFPS. */
  class FPS {
  public:
    FPS() : fps_(0.) {}

    inline double&
    value()
    {
      return (fps_);
    }
    inline double
    value() const
    {
      return (fps_);
    }

    inline std::string
    str() const
    {
      std::stringstream ss;
      ss << std::setprecision(1) << std::fixed << fps_;
      return (ss.str());
    }

  protected:
    ~FPS() = default;

  private:
    double fps_;
  };

  /** Measures the performance of the current thread (selected by passing the
   * corresponding 'fps' helper object). The resulting value is stored in the fps
   * object. */
  template <class FPS>
  void
  calcFPS(FPS& fps) const
  {
    static pcl::StopWatch sw;
    static unsigned int count = 0;

    ++count;
    if (sw.getTimeSeconds() >= .2) {
      fps.value() = static_cast<double>(count) / sw.getTimeSeconds();
      count = 0;
      sw.reset();
    }
  }

  /** \see http://doc.qt.digia.com/qt/qwidget.html#paintEvent
   * \see http://doc.qt.digia.com/qt/opengl-overpainting.html
   */
  void
  paintEvent(QPaintEvent* event) override;

private:
  using Color = Eigen::Matrix<unsigned char, 3, 1>;
  using Colors = Eigen::Matrix<unsigned char, 3, Eigen::Dynamic>;
  using Colormap = Eigen::Matrix<unsigned char, 3, 256>;

  using CloudXYZRGBNormalMap = std::unordered_map<std::string, CloudXYZRGBNormalPtr>;

  using PointIHS = pcl::ihs::PointIHS;
  using CloudIHS = pcl::ihs::CloudIHS;
  using CloudIHSPtr = pcl::ihs::CloudIHSPtr;
  using CloudIHSConstPtr = pcl::ihs::CloudIHSConstPtr;

  using FaceVertexMesh = pcl::ihs::detail::FaceVertexMesh;
  using FaceVertexMeshPtr = std::shared_ptr<FaceVertexMesh>;
  using FaceVertexMeshConstPtr = std::shared_ptr<const FaceVertexMesh>;
  using FaceVertexMeshMap = std::unordered_map<std::string, FaceVertexMeshPtr>;

  /** \brief Check if the mesh with the given id is added.
   *
   * \note Must lock the mutex before calling this method.
   */
  bool
  getMeshIsAdded(const std::string& id);

  /** \brief Calculate the pivot for the stored id. */
  void
  calcPivot();

  /** \brief Draw all meshes.
   *
   * \note Only triangle meshes are currently supported.
   */
  void
  drawMeshes();

  /** \brief Draw a wireframe box. */
  void
  drawBox();

  /** \see http://doc.qt.digia.com/qt/qglwidget.html#initializeGL */
  void
  initializeGL() override;

  /** \see http://www.opengl.org/sdk/docs/man/xhtml/glViewport.xml */
  void
  setupViewport(const int w, const int h);

  /** \see http://doc.qt.digia.com/qt/qglwidget.html#resizeGL */
  void
  resizeGL(int w, int h) override;

  /** \see http://doc.qt.digia.com/qt/qwidget.html#mousePressEvent */
  void
  mousePressEvent(QMouseEvent* event) override;

  /** \see http://doc.qt.digia.com/qt/qwidget.html#mouseMoveEvent */
  void
  mouseMoveEvent(QMouseEvent* event) override;

  /** \see http://doc.qt.digia.com/qt/qwidget.html#wheelEvent */
  void
  wheelEvent(QWheelEvent* event) override;

  ////////////////////////////////////////////////////////////////////////
  // Members
  ////////////////////////////////////////////////////////////////////////

  /** \brief Synchronization. */
  std::mutex mutex_vis_;

  /** \brief Visualization timer. */
  std::shared_ptr<QTimer> timer_vis_;

  /** \brief Colormap. */
  Colormap colormap_;

  /** \brief The visibility confidence is normalized with this value. */
  float vis_conf_norm_;

  /** \brief Meshes stored for visualization. */
  FaceVertexMeshMap drawn_meshes_;

  /** \brief How to draw the mesh. */
  MeshRepresentation mesh_representation_;

  /** \brief How to color the shapes. */
  Coloring coloring_;

  /** \brief A box is drawn if this value is true. */
  bool draw_box_;

  /** \brief Coefficients of the drawn box. */
  BoxCoefficients box_coefficients_;

  /** \brief Scaling factor to convert from meters to the unit of the drawn files. */
  double scaling_factor_;

  /** \brief Rotation of the camera. */
  Eigen::Quaterniond R_cam_;

  /** \brief Translation of the camera. */
  Eigen::Vector3d t_cam_;

  /** \brief Center of rotation during mouse navigation. */
  Eigen::Vector3d cam_pivot_;

  /** \brief Compute the pivot for the mesh with the given id. */
  std::string cam_pivot_id_;

  /** \brief Set to true right after the mouse got pressed and false if the mouse got
   * moved. */
  bool mouse_pressed_begin_;

  /** \brief Mouse x-position of the previous mouse move event. */
  int x_prev_;

  /** \brief Mouse y-position of the previous mouse move event. */
  int y_prev_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // End namespace ihs
} // End namespace pcl

// http://doc.qt.digia.com/qt/qmetatype.html#Q_DECLARE_METATYPE
Q_DECLARE_METATYPE(pcl::ihs::OpenGLViewer::MeshRepresentation)
Q_DECLARE_METATYPE(pcl::ihs::OpenGLViewer::Coloring)
