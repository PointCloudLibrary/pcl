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
/// @file   mainWindow.h
/// @details the declaration of the class representing the main window of the
/// point cloud editor
/// @author  Yue Li and Matthew Hielsberg
///

#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <QtGui>
#include <QMainWindow>
#include <QActionGroup>
#include <QSpinBox>
#include <QSlider>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <pcl/apps/point_cloud_editor/localTypes.h>

// Forward declaration to prevent circular inclusion
class CloudEditorWidget;

/// @brief the class for point cloud editor
class MainWindow : public QMainWindow
{
   Q_OBJECT

  public:
    /// @brief Constructor
    MainWindow ();
    
    /// @brief Constructor
    /// @param argc The number of c-strings to be expected in argv
    /// @param argv An array of c-strings.  The zero entry is expected to be
    /// the name of the appliation.  Any additional strings will be interpreted
    /// as filenames designating point clouds to be loaded.
    MainWindow (int argc, char **argv);

    /// @brief Destructor
    ~MainWindow ();

    /// @brief Increase the value of the spinbox by 1.
    void
    increaseSpinBoxValue ();

    /// @brief Decrease the value of the spinbox by 1.
    void
    decreaseSpinBoxValue ();

    /// @brief Gets the value of the spinbox.
    int
    getSpinBoxValue();

    /// @brief Increase the value of the selected pts size spinbox by 1.
    void
    increaseSelectedSpinBoxValue ();

    /// @brief Decrease the value of the selected pts size  spinbox by 1.
    void
    decreaseSelectedSpinBoxValue ();

    /// @brief Gets the value of the selected pts size  spinbox.
    int
    getSelectedSpinBoxValue ();

  private slots:
    void
    about ();

    void
    help ();

  private:
    /// Initialization function.  This handles the initialization of the widget,
    /// menus, actions, etc.
    void
    initWindow ();
    
    /// create actions which are connected to file menus
    void 
    createActions ();

    /// create menus such as file and help
    void
    createMenus ();

    /// create buttons in the tool bar
    void
    createToolBars ();

    /// create spin boxes used in the tool bar.
    void
    createSpinBoxes ();

    /// create sliders used in the tool bar.
    void
    createSliders ();

    /// the cloud editor GL widget
    CloudEditorWidget *cloud_editor_widget_;

    /// the action group for making actions checkable.
    QActionGroup* action_group_;

    /// action for exit the cloud editor
    QAction *exit_action_;

    /// action for opening file
    QAction *open_action_;

    /// action for saving file
    QAction *save_action_;

    /// action for copying selected points
    QAction *copy_action_;

    /// action for pasting copied points
    QAction *paste_action_;

    /// action for cutting selected points
    QAction *cut_action_;

    /// action for deleting selected points
    QAction *delete_action_;

    /// action for viewing the software information
    QAction *about_action_;

    /// action for viewing the software use/control information
    QAction *help_action_;

    /// action for toggling the pseudo distance display
    QAction *toggle_blend_action_;

    /// action for switching to view mode
    QAction *view_action_;

    /// action for undo
    QAction *undo_action_;

    /// action for point selection
    QAction *select_action_;

    /// action for 2D point selection
    QAction *select_2D_action_;

    /// action for 3D point selection
    //QAction *select_3D_action_;

    /// action for box edit
    QAction *box_edit_action_;

    /// action for invert selection
    QAction *invert_select_action_;

    /// action for transforming the cloud
    QAction *transform_action_;

    /// action for denoising the cloud
    QAction *denoise_action_;

    /// action for showing the statistics of the editor
    QAction *show_stat_action_;

    /// the file menu
    QMenu *file_menu_;

    /// the menu for editing tools
    QMenu *edit_menu_;

    /// the menu for display options
    QMenu *display_menu_;

    /// the menu for visualization tools
    QMenu *view_menu_;

    /// the menu for select tools
    QMenu *select_menu_;

    /// the menu for other algorithmic tools
    QMenu *tool_menu_;

    /// the help menu
    QMenu *help_menu_;

    /// the spin box for adjusting point size.
    QSpinBox *point_size_spin_box_;

    /// the spin box for adjusting the size of the selected point.
    QSpinBox *selected_point_size_spin_box_;

    /// the tool bar containing all the cloud editing buttons.
    QToolBar *edit_tool_bar_;

    /// the tool bar containing all the visualization function buttons
    QToolBar *view_tool_bar_;

    /// the width of the main window.
    int window_width_;

    /// the height of the main window.
    int window_height_;

    /// the slider used for adjusting moving speed.
     QSlider *move_speed_slider_;
};
#endif //MAIN_WINDOW_H_
