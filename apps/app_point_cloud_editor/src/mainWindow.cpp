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

/// @file mainWindow.cpp
/// @details the implementation of the MainWindow class
/// @author Yue Li and Matthew Hielsberg

#include <algorithm>
#include <pcl/apps/point_cloud_editor/mainWindow.h>
#include <pcl/apps/point_cloud_editor/cloudEditorWidget.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

MainWindow::MainWindow () :
  window_width_(WINDOW_WIDTH), window_height_(WINDOW_HEIGHT)
{
  initWindow();
}

MainWindow::MainWindow (int argc, char **argv) :
window_width_(WINDOW_WIDTH), window_height_(WINDOW_HEIGHT)
{
  initWindow();
  if (argc > 1)
    cloud_editor_widget_->loadFile(argv[1]);
}

MainWindow::~MainWindow()
{

}

void
MainWindow::about ()
{
  QMessageBox::about(this, tr("Point Cloud Editor"),
    tr("PCL 3D Editor\n\nAuthors: \n  Matthew Hielsberg (hielsber@tamu.edu) and\n"
       "  Yue Li (yli@cse.tamu.edu)\n  Texas A&M University\n\n"
       "This software was written as part of a collaboration with the "
       "University of South Carolina, Interdisciplinary Mathematics Institute."));
}

void
MainWindow::help ()
{
  QMessageBox::about(this, tr("Point Cloud Editor"),
                     tr("View Mode\n"
                        "  Drag:\t\tRotate about origin\n"
                        "  Alt Drag:\t\tTranslate Z\n"
                        "  Ctrl Drag:\t\tPan\n"
                        "  Shift Drag:\t\tZoom\n"
                        "\n"
                        "Selection Transform Mode\n"
                        "  Drag:\t\tRotate about centeroid\n"
                        "  Alt Drag:\t\tTranslate Z\n"
                        "  Ctrl Drag:\t\tPan\n"
                        "\n"
                        "Mouse Picking\n"
                        "  Left Click:\t\tSelect Point\n"
                        "  Ctrl Left Click:\tDeselect Point\n"
                        "  Shift Left Click:\tAppend to Selection\n"
                        "\n"
                        "2D Picking (Rubberband)\n"
                        "  Drag:\t\tSelect Region\n"
                        "  Ctrl Drag:\t\tDeselect Region\n"
                        "  Shift Drag:\t\tAppend to Selection\n"
                        "\n"
                        "Shortcut Keys\n"
                        "  1:\t\tColor Points White\n"
                        "  2:\t\tUse ColorWheel X\n"
                        "  3:\t\tUse ColorWheel Y\n"
                        "  4:\t\tUse ColorWheel Z\n"
                        "  5:\t\tUse RGB Color\n"
                        "  Ctrl C:\t\tCopy Selection\n"
                        "  Ctrl X:\t\tCut Selection\n"
                        "  Ctrl V:\t\tPaste Selection\n"
                        "  Ctrl Z:\t\tUndo\n"
                        "  V:\t\tView Mode\n"
                        "  T:\t\tSelection Transform Mode\n"
                        "  E:\t\tPoint Selection Mode\n"
                        "  S:\t\t2D Selection Mode\n"
                        "  Del:\t\tDelete Selection\n"
                        "  +:\t\tIncrease Point Size\n"
                        "  -:\t\tDecrease Point Size\n"
                        "  Ctrl +:\t\tInc. Selection Point Size\n"
                        "  Ctrl -:\t\tDec. Selection Point Size\n"
                        "  Esc:\t\tCancel Selection\n"
                        ));
}

void
MainWindow::initWindow ()
{
  cloud_editor_widget_ = new CloudEditorWidget(this);
  cloud_editor_widget_->setAttribute(Qt::WA_DeleteOnClose);
  setCentralWidget(cloud_editor_widget_);
  createActions();
  createMenus();
  createToolBars();
  setWindowTitle(tr("PCL 3D Editor (TAMU)"));
  resize(window_width_, window_height_);
}

void
MainWindow::createActions ()
{
  action_group_ = new QActionGroup(this);

  QString icon_path(":/");

  open_action_ = new QAction(QIcon(icon_path+"open.png"),tr("Open..."),this);
  open_action_ -> setShortcut(tr("Ctrl+O"));
  connect(open_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(load()));

  save_action_ = new QAction(QIcon(icon_path+"save.png"), tr("Save as.."),
                             this);
  save_action_ -> setShortcut(tr("Ctrl+S"));
  connect(save_action_,SIGNAL(triggered()),cloud_editor_widget_,SLOT(save()));

  exit_action_ = new QAction(tr("Exit..."), this);
  exit_action_ -> setShortcut(tr("Ctrl+Q"));
  connect(exit_action_, SIGNAL(triggered()), this, SLOT(close()));

  about_action_ = new QAction(tr("About"), this);
  connect(about_action_, SIGNAL(triggered()), this, SLOT(about()));

  help_action_ = new QAction(tr("Keyboard/Mouse Control"), this);
  connect(help_action_, SIGNAL(triggered()), this, SLOT(help()));

  copy_action_ = new QAction(QIcon(icon_path+"copy.png"), tr("Copy"),
                             action_group_);
  copy_action_ -> setShortcut(tr("Ctrl+C"));
  connect(copy_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(copy()));
  copy_action_->setCheckable(false);

  delete_action_ = new QAction(QIcon(icon_path+"delete.png"), tr("Delete"),
                               action_group_);
  delete_action_ -> setShortcut(tr("D"));
  connect(delete_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(remove()));
  delete_action_->setCheckable(false);

  cut_action_ = new QAction(QIcon(icon_path+"cut.png"), tr("Cut"),
                            action_group_);
  cut_action_ -> setShortcut(tr("Ctrl+X"));
  connect(cut_action_, SIGNAL(triggered()), cloud_editor_widget_,SLOT(cut()));
  cut_action_ -> setCheckable(false);

  paste_action_ = new QAction(QIcon(icon_path+"paste.png"), tr("Paste"),
                              action_group_);
  paste_action_ -> setShortcut(tr("Ctrl+V"));
  connect(paste_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(paste()));
  paste_action_ -> setCheckable(false);

  toggle_blend_action_ = new QAction(tr("Outline Points"), this);
  connect(toggle_blend_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(toggleBlendMode()));
  toggle_blend_action_->setCheckable(true);
  toggle_blend_action_->setChecked(false);

  view_action_ = new QAction(QIcon(icon_path+"view.png"), tr("View"),
                             action_group_);
  view_action_ -> setShortcut(tr("V"));
  connect(view_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(view()));
  view_action_->setCheckable(true);
  view_action_->setChecked(true);

  undo_action_ = new QAction(QIcon(icon_path+"undo.png"),
                                   tr("Undo"), action_group_);
  undo_action_ -> setShortcut(tr("Ctrl+Z"));
  connect(undo_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(undo()));
  undo_action_->setCheckable(false);

  transform_action_ = new QAction(QIcon(icon_path+"move.png"),
                                  tr("Transform Selection"), action_group_);
  transform_action_ -> setShortcut(tr("T"));
  connect(transform_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(transform()));
  transform_action_->setCheckable(true);


  denoise_action_ = new QAction(tr("Denoise"), this);
  connect(denoise_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(denoise()));

  select_action_ = new QAction(QIcon(icon_path+"click.png"),
                               tr("Point Selection"), action_group_);
  select_action_->setShortcut(tr("E"));
  connect(select_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(select1D()));
  select_action_->setCheckable(true);

  invert_select_action_ = new QAction(QIcon(icon_path+"invert.png"),
                                      tr("Invert Selection"), action_group_);
  connect(invert_select_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(invertSelect()));
  invert_select_action_->setCheckable(false);

  select_2D_action_ = new QAction(QIcon(icon_path+"select.png"),
                                  tr("Rubberband Selection"), action_group_);
  select_2D_action_->setShortcut(tr("S"));
  connect(select_2D_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(select2D()));
  select_2D_action_->setCheckable(true);

  //select_3D_action_ = new QAction(QIcon(icon_path+"cube.png"),
  //                                tr("3D Selection"), action_group_);
  //select_3D_action_->setShortcut(tr("A"));
  //connect(select_3D_action_, SIGNAL(triggered()), cloud_editor_widget_,
  //        SLOT(select3D()));
  //select_3D_action_->setCheckable(true);

  show_stat_action_ = new QAction(QIcon(icon_path+"info.png"),
                      tr("Show statistics"), action_group_);
  connect(show_stat_action_, SIGNAL(triggered()), cloud_editor_widget_,
          SLOT(showStat()));
  show_stat_action_->setCheckable(false);
}

void
MainWindow::createMenus ()
{
  file_menu_ = new QMenu(tr("&File"), this);
  file_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  file_menu_ -> addAction(open_action_);
  file_menu_ -> addSeparator();
  file_menu_ -> addAction(save_action_);
  file_menu_ -> addSeparator();
  file_menu_ -> addAction(exit_action_);

  edit_menu_ = new QMenu(tr("&Edit"), this);
  edit_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  edit_menu_ -> addAction(undo_action_);
  edit_menu_ -> addSeparator();
  edit_menu_ -> addAction(cut_action_);
  edit_menu_ -> addAction(copy_action_);
  edit_menu_ -> addAction(paste_action_);
  edit_menu_ -> addAction(delete_action_);
  edit_menu_ -> addSeparator();
  edit_menu_ -> addAction(transform_action_);

  select_menu_ = new QMenu(tr("&Select"), this);
  select_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  select_menu_ -> addAction(select_action_);
  select_menu_ -> addAction(select_2D_action_);
  //select_menu_ -> addAction(select_3D_action_);

  display_menu_ = new QMenu(tr("&Display"), this);
  display_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  display_menu_ -> addAction(toggle_blend_action_);

  view_menu_ = new QMenu(tr("&View"), this);
  view_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  view_menu_ -> addAction(view_action_);
  view_menu_ -> addAction(show_stat_action_);

  tool_menu_ = new QMenu(tr("&Algorithm"), this);
  tool_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  tool_menu_ -> addAction(denoise_action_);

  help_menu_ = new QMenu(tr("&Help"), this);
  help_menu_ -> setAttribute(Qt::WA_DeleteOnClose);
  help_menu_ -> addAction(about_action_);
  help_menu_ -> addAction(help_action_);

  menuBar() -> addMenu(file_menu_);
  menuBar() -> addMenu(edit_menu_);
  menuBar() -> addMenu(select_menu_);
  menuBar() -> addMenu(display_menu_);
  menuBar() -> addMenu(tool_menu_);
  menuBar() -> addMenu(help_menu_);
}

void
MainWindow::createToolBars ()
{
  createSpinBoxes();
  createSliders();
  view_tool_bar_ = addToolBar(tr("File"));
  view_tool_bar_ -> addAction(open_action_);
  view_tool_bar_ -> addAction(save_action_);
  view_tool_bar_ = addToolBar(tr("View"));
  view_tool_bar_ -> addAction(view_action_);
  view_tool_bar_ -> addAction(select_action_);
  view_tool_bar_ -> addAction(select_2D_action_);
  //view_tool_bar_ -> addAction(select_3D_action_);
  view_tool_bar_ -> addAction(invert_select_action_);
  QLabel *ptSizeLabel = new QLabel(tr("Point Size:"));
  ptSizeLabel -> setAttribute(Qt::WA_DeleteOnClose);
  view_tool_bar_ -> addWidget(ptSizeLabel);
  view_tool_bar_ -> addWidget(point_size_spin_box_);
  QLabel *selectedPtSizeLabel = new QLabel(tr("Selected Point Size:"));
  selectedPtSizeLabel -> setAttribute(Qt::WA_DeleteOnClose);
  view_tool_bar_ -> addWidget(selectedPtSizeLabel);
  view_tool_bar_ -> addWidget(selected_point_size_spin_box_);
  edit_tool_bar_ = addToolBar(tr("Edit"));
  edit_tool_bar_ -> addAction(undo_action_);
  edit_tool_bar_ -> addAction(copy_action_);
  edit_tool_bar_ -> addAction(cut_action_);
  edit_tool_bar_ -> addAction(delete_action_);
  edit_tool_bar_ -> addAction(paste_action_);
  edit_tool_bar_ -> addAction(transform_action_);
  edit_tool_bar_ -> addAction(show_stat_action_);
}

void
MainWindow::increaseSpinBoxValue()
{
  point_size_spin_box_ -> setValue(point_size_spin_box_->value() + 1);
}

void
MainWindow::decreaseSpinBoxValue()
{
  point_size_spin_box_ -> setValue(point_size_spin_box_->value() - 1);
}

int
MainWindow::getSpinBoxValue()
{
  return (point_size_spin_box_->value());
}

void
MainWindow::increaseSelectedSpinBoxValue()
{
  selected_point_size_spin_box_ -> setValue(selected_point_size_spin_box_->value() + 1);
}

void
MainWindow::decreaseSelectedSpinBoxValue()
{
  selected_point_size_spin_box_ -> setValue(selected_point_size_spin_box_->value() - 1);
}

int
MainWindow::getSelectedSpinBoxValue()
{
  return (selected_point_size_spin_box_->value());
}

void
MainWindow::createSpinBoxes ()
{
  point_size_spin_box_ = new QSpinBox;
  point_size_spin_box_ -> setAttribute(Qt::WA_DeleteOnClose);
  point_size_spin_box_ -> setRange(1, 20);
  point_size_spin_box_ -> setSingleStep(1);
  point_size_spin_box_ -> setValue(2);
  connect(point_size_spin_box_, SIGNAL(valueChanged(int)),
          cloud_editor_widget_, SLOT(setPointSize(int)));
  selected_point_size_spin_box_ = new QSpinBox;
  selected_point_size_spin_box_ -> setAttribute(Qt::WA_DeleteOnClose);
  selected_point_size_spin_box_ -> setRange(1, 20);
  selected_point_size_spin_box_ -> setSingleStep(1);
  selected_point_size_spin_box_ -> setValue(4);
  connect(selected_point_size_spin_box_, SIGNAL(valueChanged(int)),
          cloud_editor_widget_, SLOT(setSelectedPointSize(int)));
}

void
MainWindow::createSliders ()
{
  move_speed_slider_ = new QSlider(Qt::Horizontal);
  move_speed_slider_ -> setAttribute(Qt::WA_DeleteOnClose);
  move_speed_slider_ -> setFocusPolicy(Qt::StrongFocus);
  move_speed_slider_ -> setTickPosition(QSlider::TicksBothSides);
  move_speed_slider_ -> setTickInterval(10);
  move_speed_slider_ -> setSingleStep(1);
}
