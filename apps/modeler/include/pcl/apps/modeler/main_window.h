/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <ui_main_window.h>

#include <memory>

namespace pcl {
namespace modeler {

class SceneTree;
class AbstractItem;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  static MainWindow&
  getInstance()
  {
    static MainWindow theSingleton;
    return theSingleton;
  }

  QString
  getRecentFolder();

  RenderWindowItem*
  createRenderWindow();

public Q_SLOTS:
  // slots for file menu
  void
  slotOpenProject();
  void
  slotSaveProject();
  void
  slotCloseProject();
  void
  slotExit();
  void
  slotUpdateRecentFile(const QString& filename);

  // slots for view menu
  void
  slotCreateRenderWindow();

  void
  slotOnWorkerStarted();

  void
  slotOnWorkerFinished();

private:
  // methods for file Menu
  void
  connectFileMenuActions();
  void
  createRecentPointCloudActions();
  void
  updateRecentPointCloudActions();
  void
  createRecentProjectActions();
  void
  updateRecentProjectActions();
  bool
  openProjectImpl(const QString& filename);
  static void
  updateRecentActions(std::vector<std::shared_ptr<QAction>>& recent_actions,
                      QStringList& recent_items);

  // methods for view menu
  void
  connectViewMenuActions();

  // methods for edit menu
  void
  connectEditMenuActions();

  // methods for global settings
  void
  loadGlobalSettings();
  void
  saveGlobalSettings();

private Q_SLOTS:
  void
  slotOpenRecentPointCloud();
  void
  slotOpenRecentProject();

private:
  friend class AbstractItem;

  MainWindow();
  MainWindow(const MainWindow&) = delete;
  MainWindow&
  operator=(const MainWindow&) = delete;
  ~MainWindow();

  Ui::MainWindow* ui_; // Designer form

  // shortcuts for recent point clouds/projects
  QStringList recent_files_;
  QStringList recent_projects_;
  static const std::size_t MAX_RECENT_NUMBER = 8;
  std::vector<std::shared_ptr<QAction>> recent_pointcloud_actions_;
  std::vector<std::shared_ptr<QAction>> recent_project_actions_;
};

} // namespace modeler
} // namespace pcl
