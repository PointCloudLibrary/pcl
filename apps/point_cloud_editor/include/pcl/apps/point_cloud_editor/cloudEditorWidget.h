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
/// @file   cloudEditorWidget.h
/// @details the declaration of the class representing the widgets
/// which are used for viewing and editing point clouds
/// @author  Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/commandQueue.h>
#include <pcl/apps/point_cloud_editor/denoiseParameterForm.h>
#include <pcl/apps/point_cloud_editor/statisticsDialog.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>

#include <pcl/memory.h>  // for pcl::shared_ptr

#include <QGLWidget>

#include <functional>

class Selection;

/// @brief class declaration for the widget for editing and viewing
/// point clouds.
class CloudEditorWidget : public QGLWidget
{
  Q_OBJECT
  public:
    /// The type for shared pointer pointing to a selection buffer
    using SelectionPtr = pcl::shared_ptr<Selection>;

    /// @brief Constructor
    /// @param parent a pointer which points to the parent widget
    CloudEditorWidget (QWidget *parent = nullptr);

    /// @brief Destructor
    ~CloudEditorWidget () override;

    /// @brief Attempts to load the point cloud designated by the passed file
    /// name.
    /// @param filename The name of the point cloud file to be loaded.
    /// @remarks throws if the passed file can not be loaded.
    void
    loadFile(const std::string &filename);

  public Q_SLOTS:
    /// @brief Loads a new cloud.
    void
    load ();

    /// @brief Saves a cloud to a .pcd file. The current format is ASCII.
    void
    save ();

    /// @brief Toggles the blend mode used to render the non-selected points
    void
    toggleBlendMode ();

    /// @brief Switches to the view mode.
    void
    view ();

    /// @brief Enters click selection mode.
    void
    select1D ();

    /// @brief Enters 2D selection mode.
    void
    select2D ();

    /// @brief Enters 3D selection mode.
    void
    select3D ();

    /// @brief Inverts the current selection.
    void
    invertSelect ();

    /// @brief Cancels the current selection.
    void
    cancelSelect ();

    /// @brief Copies the selected points.
    void
    copy ();

    /// @brief Pastes the copied points to the cloud.
    void
    paste ();

    /// @brief Removes the selected points.
    void
    remove ();

    /// @brief Copies and then removes the selected points.
    void
    cut ();

    /// @brief Enters the mode where users are able to translate the selected
    /// points.
    void
    transform ();

    /// @brief Denoises the current cloud.
    void
    denoise ();

    /// @brief Undoes last change.
    void
    undo ();

    /// @brief Increases the size of the unselected points.
    void
    increasePointSize ();

    /// @brief Decreases the size of the unselected points.
    void
    decreasePointSize ();

    /// @brief Increases the size of the selected points.
    void
    increaseSelectedPointSize ();

    /// @brief Decreases the size of the selected points.
    void
    decreaseSelectedPointSize ();

    /// @brief Sets the size of the unselected points.
    void
    setPointSize (int size);

    /// @brief Sets the size of the selected points.
    void
    setSelectedPointSize (int size);

    /// @brief Colors the unselected points by its native color.
    void
    colorByRGB ();

    /// @brief Colors the unselected points with a color ramp based on the X
    /// values of the points
    void
    colorByX ();

    /// @brief Colors the unselected points with a color ramp based on the Y
    /// values of the points
    void
    colorByY ();

    /// @brief Colors the unselected points with a color ramp based on the Z
    /// values of the points
    void
    colorByZ ();

    /// @brief Colors the unselected points using an single color.
    void
    colorByPure ();

    /// @brief Turn on the dialog box showing the statistics of the cloud.
    void
    showStat ();

  protected:  
    /// initializes GL
    void
    initializeGL () override;

    /// the rendering function.
    void
    paintGL () override;

    /// resizes widget
    void
    resizeGL (int width, int height) override;

    /// mouse press control
    void
    mousePressEvent (QMouseEvent *event) override;

    /// mouse move control
    void
    mouseMoveEvent (QMouseEvent *event) override;

    /// mouse release control
    void
    mouseReleaseEvent (QMouseEvent *event) override;

    /// key press control
    void
    keyPressEvent (QKeyEvent *event) override;

  private:
    
    /// @brief Attempts to load a pcd file
    /// @param filename The name of the pcd file to be loaded.
    /// @remarks throws if the passed file can not be loaded.
    void
    loadFilePCD(const std::string &filename);
    
    /// @brief Adds all of our file loader functions to the extension map
    void
    initFileLoadMap();
    
    /// @brief Returns true if the cloud stored in a file is colored
    /// @param fileName a reference to a string storing the path of a cloud
    bool
    isColored (const std::string &fileName) const;

    /// @brief swap the values of r and b in each point of the cloud.
    void
    swapRBValues ();

    /// @brief initializes the map between key press events and the
    /// corresponding functors.
    void
    initKeyMap();

    struct ExtCompare
    {
      bool
      operator()(std::string lhs, std::string rhs) const
      {
        stringToLower(lhs);
        stringToLower(rhs);
        return lhs.compare(rhs) < 0;
      }
    };

    using FileLoadFunc = std::function<void (CloudEditorWidget*, const std::string&)>;
    using FileLoadMap = std::map<std::string, FileLoadFunc, ExtCompare>;

    /// a map of file type extensions to loader functions.
    FileLoadMap cloud_load_func_map_;
    
    /// a pointer to the cloud being edited.
    CloudPtr cloud_ptr_;

    /// The display size, in pixels, of the cloud points
    unsigned int point_size_;

    /// The display size, in pixels, of the selected cloud points
    unsigned int selected_point_size_;

    /// The transformation tool being used. Either a cloud transform tool or
    /// a selection transform tool is activated at a time.
    std::shared_ptr<ToolInterface> tool_ptr_;

    /// a pointer to the selection object
    SelectionPtr selection_ptr_;

    /// a pointer to the copy buffer object.
    CopyBufferPtr copy_buffer_ptr_;

    /// a pointer to the command queue object
    CommandQueuePtr command_queue_ptr_;

    /// The camera field of view
    double cam_fov_;

    /// The camera aspect ratio
    double cam_aspect_;

    /// The camera near clipping plane
    double cam_near_;

    /// The camera far clipping plane
    double cam_far_;

    /// @brief Initialize the texture used for rendering the cloud
    void
    initTexture ();

    /// The current scheme used for coloring the whole cloud
    ColorScheme color_scheme_;

    /// A flag indicates whether the cloud is initially colored or not.
    bool is_colored_;

    using KeyMapFunc = std::function<void (CloudEditorWidget*)>;

    /// map between pressed key and the corresponding functor
    std::map<int, KeyMapFunc> key_map_;

    /// a dialog displaying the statistics of the cloud editor
    StatisticsDialog stat_dialog_;


};
