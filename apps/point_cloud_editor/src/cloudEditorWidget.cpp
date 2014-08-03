///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///
///
/// @file   cloudEditorWidget.cpp
/// @details the implementation of class CloudEditorWidget.
/// @author  Yue Li and Matthew Hielsberg

#include <ctype.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QMouseEvent>
#include <qgl.h>

#include <pcl/pcl_config.h>

#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/glu.h>
#else
# include <GL/glu.h>
#endif

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/apps/point_cloud_editor/cloudEditorWidget.h>
#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/cloudTransformTool.h>
#include <pcl/apps/point_cloud_editor/selectionTransformTool.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/select1DTool.h>
#include <pcl/apps/point_cloud_editor/select2DTool.h>
//#include <pcl/apps/point_cloud_editor/select3DTool.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/copyCommand.h>
#include <pcl/apps/point_cloud_editor/pasteCommand.h>
#include <pcl/apps/point_cloud_editor/deleteCommand.h>
#include <pcl/apps/point_cloud_editor/denoiseCommand.h>
#include <pcl/apps/point_cloud_editor/cutCommand.h>
#include <pcl/apps/point_cloud_editor/mainWindow.h>

CloudEditorWidget::CloudEditorWidget (QWidget *parent)
  : QGLWidget(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer |
                        QGL::Rgba | QGL::StencilBuffer), parent),
    point_size_(2.0f), selected_point_size_(4.0f),
    cam_fov_(60.0), cam_aspect_(1.0), cam_near_(0.0001), cam_far_(100.0),
    color_scheme_(COLOR_BY_PURE), is_colored_(false)
{
  setFocusPolicy(Qt::StrongFocus);
  command_queue_ptr_ = CommandQueuePtr(new CommandQueue());
  initFileLoadMap();
  initKeyMap();
}

CloudEditorWidget::~CloudEditorWidget ()
{
}

void
CloudEditorWidget::loadFile(const std::string &filename)
{
  std::string ext = filename.substr(filename.find_last_of(".")+1);
  FileLoadMap::iterator it = cloud_load_func_map_.find(ext);
  if (it != cloud_load_func_map_.end())
    (it->second)(this, filename);
  else
    loadFilePCD(filename);
}

void
CloudEditorWidget::load ()
{
  QString file_path = QFileDialog::getOpenFileName(this, tr("Open File"));
    
  if (file_path.isEmpty())
    return;

  try
  {
    loadFile(file_path.toStdString());
  }
  catch (...)
  {
    QMessageBox::information(this, tr("Point Cloud Editor"),
                             tr("Can not load %1.").arg(file_path));
  }
  update();
  updateGL();
}

void
CloudEditorWidget::save ()
{
  if (!cloud_ptr_)
  {
    QMessageBox::information(this, tr("Point Cloud Editor"),
                             tr("No cloud is loaded."));
    return;
  }

  QString file_path = QFileDialog::getSaveFileName(this,tr("Save point cloud"));
  
  std::string file_path_std = file_path.toStdString();
  if ( (file_path_std == "") || (!cloud_ptr_) )
    return;

  if (is_colored_)
  {
    // the swapping is due to the strange alignment of r,g,b values used by PCL..
    swapRBValues();
    try
    {
      pcl::io::savePCDFile(file_path_std, cloud_ptr_->getInternalCloud());
    }
    catch (...)
    {
      QMessageBox::information(this, tr("Point Cloud Editor"),
                               tr("Can not save %1.").arg(file_path));
    }
    swapRBValues();
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ> uncolored_cloud;
    pcl::copyPointCloud(cloud_ptr_->getInternalCloud(), uncolored_cloud);
    try
    {
      pcl::io::savePCDFile(file_path_std, uncolored_cloud);
    }
    catch (...)
    {
      QMessageBox::information(this, tr("Point Cloud Editor"),
                               tr("Can not save %1.").arg(file_path));
    }
  }
}

void
CloudEditorWidget::toggleBlendMode ()
{
  if (!cloud_ptr_)
    return;
  GLint blend_src = 0;
  glGetIntegerv(GL_BLEND_SRC, &blend_src);
  if (blend_src == GL_SRC_ALPHA)
    glBlendFunc( GL_ONE, GL_ZERO );
  else
    glBlendFunc( GL_SRC_ALPHA, GL_ZERO );
  update();
}

void
CloudEditorWidget::view ()
{
  if (!cloud_ptr_)
    return;
  tool_ptr_ = boost::shared_ptr<CloudTransformTool>(
              new CloudTransformTool(cloud_ptr_));
}

void
CloudEditorWidget::select1D ()
{
  if (!cloud_ptr_)
    return;
  tool_ptr_ = boost::shared_ptr<Select1DTool>(new Select1DTool(selection_ptr_,
                                                               cloud_ptr_));
  update();
}

void
CloudEditorWidget::select2D ()
{
  if (!cloud_ptr_)
    return;
  tool_ptr_ = boost::shared_ptr<Select2DTool>(new Select2DTool(selection_ptr_,
                                                               cloud_ptr_));
  update();
}

void
CloudEditorWidget::select3D ()
{
  if (!cloud_ptr_)
    return;
  //tool_ptr_ = boost::shared_ptr<Select3DTool>(new Select3DTool(selection_ptr_,
  //                                                             cloud_ptr_));
  update();
}

void
CloudEditorWidget::invertSelect ()
{
  if (!selection_ptr_)
    return;
  selection_ptr_ -> invertSelect();
  cloud_ptr_->setSelection(selection_ptr_);
  update();
}

void
CloudEditorWidget::cancelSelect ()
{
  if (!selection_ptr_)
    return;
  selection_ptr_ -> clear();
  update();
}

void
CloudEditorWidget::copy ()
{
  if (!cloud_ptr_)
    return;
  if (!selection_ptr_ || selection_ptr_->empty())
    return;
  boost::shared_ptr<CopyCommand> c(new CopyCommand(copy_buffer_ptr_,
    selection_ptr_, cloud_ptr_));
  command_queue_ptr_->execute(c);
}

void
CloudEditorWidget::paste ()
{
  if (!cloud_ptr_)
    return;
  if (!copy_buffer_ptr_ || copy_buffer_ptr_->empty())
    return;
  boost::shared_ptr<PasteCommand> c(new PasteCommand(copy_buffer_ptr_,
    selection_ptr_, cloud_ptr_));
  command_queue_ptr_->execute(c);
  update();
}

void
CloudEditorWidget::remove ()
{
  if (!cloud_ptr_)
    return;
  if (!selection_ptr_ || selection_ptr_->empty())
    return;
  boost::shared_ptr<DeleteCommand> c(new DeleteCommand(selection_ptr_,
                                                       cloud_ptr_));
  command_queue_ptr_->execute(c);
  update();
}

void
CloudEditorWidget::cut ()
{
  if (!cloud_ptr_)
    return;
  if (!selection_ptr_ || selection_ptr_->empty())
    return;
  boost::shared_ptr<CutCommand> c(new CutCommand(copy_buffer_ptr_,
    selection_ptr_, cloud_ptr_));
  command_queue_ptr_->execute(c);
  update();
}

void
CloudEditorWidget::transform ()
{
  if (!cloud_ptr_ || !selection_ptr_ || selection_ptr_->empty())
    return;
  tool_ptr_ = boost::shared_ptr<SelectionTransformTool>(
    new SelectionTransformTool(selection_ptr_, cloud_ptr_, command_queue_ptr_));
  update();
}

void
CloudEditorWidget::denoise ()
{
  if (!cloud_ptr_)
    return;
  DenoiseParameterForm form;
  form.exec();
  boost::shared_ptr<DenoiseCommand> c(new DenoiseCommand(selection_ptr_,
    cloud_ptr_, form.getMeanK(), form.getStdDevThresh()));
  command_queue_ptr_->execute(c);
  update();
}

void
CloudEditorWidget::undo ()
{
  if (!cloud_ptr_)
    return;
  command_queue_ptr_ -> undo();
  update();
}

void
CloudEditorWidget::increasePointSize ()
{
  ((MainWindow*) parentWidget()) -> increaseSpinBoxValue();
  point_size_ = ((MainWindow*) parentWidget()) -> getSpinBoxValue();
  if (!cloud_ptr_)
    return;
  cloud_ptr_->setPointSize(point_size_);
  update();
}

void
CloudEditorWidget::decreasePointSize ()
{
  ((MainWindow*) parentWidget()) -> decreaseSpinBoxValue();
  point_size_ = ((MainWindow*) parentWidget()) -> getSpinBoxValue();
  if (!cloud_ptr_)
    return;
  cloud_ptr_->setPointSize(point_size_);
  update();
}

void
CloudEditorWidget::increaseSelectedPointSize ()
{
  ((MainWindow*) parentWidget()) -> increaseSelectedSpinBoxValue();
  selected_point_size_ =
    ((MainWindow*) parentWidget()) -> getSelectedSpinBoxValue();
  if (!cloud_ptr_)
    return;
  cloud_ptr_->setHighlightPointSize(selected_point_size_);
  update();
}

void
CloudEditorWidget::decreaseSelectedPointSize ()
{
  ((MainWindow*) parentWidget()) -> decreaseSelectedSpinBoxValue();
  selected_point_size_ =
    ((MainWindow*) parentWidget()) -> getSelectedSpinBoxValue();
  if (!cloud_ptr_)
    return;
  cloud_ptr_->setHighlightPointSize(selected_point_size_);
  update();
}

void
CloudEditorWidget::setPointSize (int size)
{
  point_size_ = size;
  if (!cloud_ptr_)
    return;
  cloud_ptr_->setPointSize(size);
  update();
}

void
CloudEditorWidget::setSelectedPointSize (int size)
{
  selected_point_size_ = size;
  if (!cloud_ptr_)
    return;
  cloud_ptr_ -> setHighlightPointSize(size);
  update();
}

void
CloudEditorWidget::colorByRGB ()
{
  if(is_colored_)
    color_scheme_ = COLOR_BY_RGB;
}

void
CloudEditorWidget::colorByX ()
{
  color_scheme_ = COLOR_BY_X;
}

void
CloudEditorWidget::colorByY ()
{
  color_scheme_ = COLOR_BY_Y;
}

void
CloudEditorWidget::colorByZ ()
{
  color_scheme_ = COLOR_BY_Z;
}

void
CloudEditorWidget::colorByPure ()
{
  color_scheme_ = COLOR_BY_PURE;
}

void
CloudEditorWidget::showStat ()
{
  stat_dialog_.update();
  stat_dialog_.show();
  stat_dialog_.raise();
}

void
CloudEditorWidget::initializeGL ()
{
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_FLAT);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glDisable(GL_FOG);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable( GL_BLEND );
  glBlendFunc( GL_ONE, GL_ZERO );
  glHint(GL_POINT_SMOOTH_HINT & GL_LINE_SMOOTH_HINT, GL_NICEST);
  initTexture();
}

void
CloudEditorWidget::paintGL ()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if (!cloud_ptr_)
    return;
  tool_ptr_ -> draw();


  if (color_scheme_ == COLOR_BY_RGB)
    cloud_ptr_->drawWithRGB();
  else if (color_scheme_ == COLOR_BY_PURE)
    cloud_ptr_->drawWithPureColor();
  else
  {
    // Assumes that color_scheme_ contains COLOR_BY_[X,Y,Z] and the values
    // match Axis::[X,Y,Z]
    cloud_ptr_ -> setColorRampAxis(Axis(color_scheme_));
    cloud_ptr_ -> drawWithTexture();
  }


}

void
CloudEditorWidget::resizeGL (int width, int height)
{
  glViewport(0, 0, width, height);
  cam_aspect_ = double(width) / double(height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(cam_fov_, cam_aspect_, cam_near_, cam_far_);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void
CloudEditorWidget::mousePressEvent (QMouseEvent *event)
{
  if (!tool_ptr_)
    return;
  tool_ptr_ -> start(event -> x(), event -> y(),
                     event -> modifiers(), event -> buttons());
  update();
}

void
CloudEditorWidget::mouseMoveEvent (QMouseEvent *event)
{
  if (!tool_ptr_)
    return;
  tool_ptr_ -> update(event -> x(), event -> y(),
                      event -> modifiers(), event -> buttons());
  update();
}

void
CloudEditorWidget::mouseReleaseEvent (QMouseEvent *event)
{
  if (!tool_ptr_)
    return;
  tool_ptr_ -> end(event -> x(), event -> y(),
                   event -> modifiers(), event -> button());
  update();
}

void
CloudEditorWidget::keyPressEvent (QKeyEvent *event)
{
  int key = event->key() + static_cast<int>(event->modifiers());
  std::map<int, KeyMapFunc>::iterator it = key_map_.find(key);
  if (it != key_map_.end())
  {
    (it->second)(this);
    update();
  }
}

void
CloudEditorWidget::loadFilePCD(const std::string &filename)
{   
  PclCloudPtr pcl_cloud_ptr;
  Cloud3D tmp;
  if (pcl::io::loadPCDFile<Point3D>(filename, tmp) == -1)
    throw;
  pcl_cloud_ptr = PclCloudPtr(new Cloud3D(tmp));
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, index);
  Statistics::clear();
  cloud_ptr_ = CloudPtr(new Cloud(*pcl_cloud_ptr, true));
  selection_ptr_ = SelectionPtr(new Selection(cloud_ptr_, true));
  copy_buffer_ptr_ = CopyBufferPtr(new CopyBuffer(true));
  cloud_ptr_->setPointSize(point_size_);
  cloud_ptr_->setHighlightPointSize(selected_point_size_);
  tool_ptr_ =
    boost::shared_ptr<CloudTransformTool>(new CloudTransformTool(cloud_ptr_));

  if (isColored(filename))
  {
    swapRBValues();
    color_scheme_ = COLOR_BY_RGB;
    is_colored_ = true;
  }
  else
  {
    color_scheme_ = COLOR_BY_Z;
    is_colored_ = false;
  }
}

void
CloudEditorWidget::initFileLoadMap()
{
  cloud_load_func_map_.clear();
  cloud_load_func_map_["pcd"] = &CloudEditorWidget::loadFilePCD;
}

bool
CloudEditorWidget::isColored (const std::string &fileName) const
{
  pcl::PCLPointCloud2 cloud2;
  pcl::PCDReader reader;
  reader.readHeader(fileName, cloud2);
  std::vector< pcl::PCLPointField > fs = cloud2.fields;
  for(unsigned int i = 0; i < fs.size(); ++i)
  {
    std::string name(fs[i].name);
    stringToLower(name);
    if ((name.compare("rgb") == 0) || (name.compare("rgba") == 0))
      return true;
  }
  return false;
}

void
CloudEditorWidget::swapRBValues ()
{
  if (!cloud_ptr_)
    return;
  for (unsigned int i = 0; i < cloud_ptr_ -> size(); i++)
  {
    uint8_t cc = (*cloud_ptr_)[i].r;
    (*cloud_ptr_)[i].r = (*cloud_ptr_)[i].b;
    (*cloud_ptr_)[i].b = cc;
  }
}

void
CloudEditorWidget::initKeyMap ()
{   
  key_map_[Qt::Key_1] = &CloudEditorWidget::colorByPure;
  key_map_[Qt::Key_2] = &CloudEditorWidget::colorByX;
  key_map_[Qt::Key_3] = &CloudEditorWidget::colorByY;
  key_map_[Qt::Key_4] = &CloudEditorWidget::colorByZ;
  key_map_[Qt::Key_5] = &CloudEditorWidget::colorByRGB;
  key_map_[Qt::Key_C + (int) Qt::ControlModifier] = &CloudEditorWidget::copy;
  key_map_[Qt::Key_X + (int) Qt::ControlModifier] = &CloudEditorWidget::cut;
  key_map_[Qt::Key_V + (int) Qt::ControlModifier] = &CloudEditorWidget::paste;
  key_map_[Qt::Key_S] = &CloudEditorWidget::select2D;
  key_map_[Qt::Key_E] = &CloudEditorWidget::select1D;
  key_map_[Qt::Key_T] = &CloudEditorWidget::transform;
  key_map_[Qt::Key_V] = &CloudEditorWidget::view;
  key_map_[Qt::Key_Delete] = &CloudEditorWidget::remove;
  key_map_[Qt::Key_Z + (int) Qt::ControlModifier] = &CloudEditorWidget::undo;
  key_map_[Qt::Key_Equal] = &CloudEditorWidget::increasePointSize;
  key_map_[Qt::Key_Plus] = &CloudEditorWidget::increasePointSize;
  key_map_[Qt::Key_Minus] = &CloudEditorWidget::decreasePointSize;
  key_map_[Qt::Key_Equal + (int) Qt::ControlModifier] =
    &CloudEditorWidget::increaseSelectedPointSize;
  key_map_[Qt::Key_Plus + (int) Qt::ControlModifier] =
    &CloudEditorWidget::increaseSelectedPointSize;
  key_map_[Qt::Key_Minus + (int) Qt::ControlModifier] =
    &CloudEditorWidget::decreaseSelectedPointSize;
  key_map_[Qt::Key_Escape] = &CloudEditorWidget::cancelSelect;
}

void
CloudEditorWidget::initTexture ()
{
  static GLfloat colorWheel[14][3] =
  {
    {      0.0f,    0.0f,  1.0000f},
    {      0.0f, 0.2500f,  1.0000f},
    {      0.0f, 0.5000f,  1.0000f},
    {      0.0f, 0.7500f,  1.0000f},
    {      0.0f, 1.0000f,  1.0000f},
    {   0.2500f, 1.0000f,  1.0000f},
    {   0.5000f, 1.0000f,  0.7500f},
    {   0.7500f, 1.0000f,  0.5000f},
    {   1.0000f, 1.0000f,  0.2500f},
    {   1.0000f, 1.0000f,     0.0f},
    {   1.0000f, 0.7500f,     0.0f},
    {   1.0000f, 0.5000f,     0.0f},
    {   1.0000f, 0.2500f,     0.0f},
    {   1.0000f,    0.0f,     0.0f},
  };
  GLuint textures;
  glGenTextures(1,&textures);
  glBindTexture(GL_TEXTURE_1D,textures);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, 14, 0, GL_RGB , GL_FLOAT, colorWheel);
}
