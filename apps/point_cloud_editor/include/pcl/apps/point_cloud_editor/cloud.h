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
/// @file   cloud.h
/// @details The declaration of the class representing the cloud data structure.
/// A cloud represents a single 3D set of points and associated attributes.
/// This class allows for the basic manipulation of the cloud as well as its
/// display.
/// @author  Yue Li and Matthew Hielsberg

#ifndef CLOUD_H_
#define CLOUD_H_

#include <QtGui/QColor>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/statistics.h>

/// @brief A wrapper which allows to use any implementation of cloud provided by
/// a third-party library.
/// @details This wrapper attempts to create a simple interface for displaying
/// and modifying point clouds.  It is common for point clouds to have
/// coordinate values that are exceptionally large, especially when dealing
/// with the GIS community.  As such this class shifts the stored point cloud
/// according to the minimum in each of the coordinate directions.  The
/// interface presented by this class then serves the shifted points in order to
/// reduce any precision errors that may occur due to sums of large values.
/// Functions also exist for accessing the unshifted versions of the points,
/// however most operations performed on the cloud will expect it to live near
/// the origin.
// XXX - handle shifting upon setting of a Cloud3D
// XXX - add functions for retrieving an unshifted Cloud3D
// XXX - add functions for retrieveing unshifted points by index
// XXX - mark access functions below as returning shifted values
class Cloud : public Statistics
{
  public:
    /// @brief Default Constructor
    Cloud ();

    /// @brief Copy Constructor
    /// @details This constructor creates a copy of the passed cloud. The
    /// values of the member variables of the passed cloud are deep copied.
    /// @param copy The cloud object to be used to initialize this cloud object.
    Cloud (const Cloud& copy);

    /// @brief Construct a cloud from a Cloud3D.
    /// @details This constructor creates a cloud object with the passed 
    /// cloud object stored with the internal representation. The member
    /// variables of this object are initialized but not set.
    Cloud (const Cloud3D& cloud, bool register_stats=false);

    /// @brief Destructor
    ~Cloud ();

    /// @brief Equal Operator
    /// @details Deep copies all the state of the passed cloud to this cloud.
    /// @param cloud The cloud object whose status to be copied to this object
    /// @return A reference to this cloud containing the new values.
    Cloud&
    operator= (const Cloud& cloud);

    /// @brief Subscript Operator
    /// @details This operator returns a reference to the point with the
    /// passed index in this cloud object.
    /// @pre The index passed is expected to be within the limits of the cloud.
    /// For debugging this is currently checked by an assert.
    /// @param index The index of the point to be returned.
    /// @return A reference to the indexed point.
    Point3D&
    operator[] (unsigned int index);

    /// @brief Subscript Operator
    /// @details This operator returns a const reference to the point with the
    /// passed index in this cloud object.
    /// @pre The index passed is expected to be within the limits of the cloud.
    /// For debugging this is currently checked by an assert.
    /// @param index The index of the point to be returned.
    /// @return A const reference to the indexed point.
    const Point3D&
    operator[] (unsigned int index) const;

    /// @brief Returns the center of the point cloud
    /// @param x The x coordinate of the center (computed as the average point).
    /// @param y The y coordinate of the center (computed as the average point).
    /// @param z The z coordinate of the center (computed as the average point).
    inline
    void
    getCenter (float &x, float &y, float &z) const
    {
      x = center_xyz_[X]; y = center_xyz_[Y]; z = center_xyz_[Z];
    }

    /// @brief Returns the scaling factor for the point cloud
    /// @return The scaling factor
    inline
    float
    getScalingFactor() const
    {
      return (display_scale_);
    }

    /// @brief Gets the transform matrix.
    /// @details The returned matrix is used to transform the cloud for
    /// rendering only and does not affect the values of the points stored.
    /// @return A 1-D array representing (4 x 4) matrix in
    /// using OpenGL's column-major format.
    inline
    const float*
    getMatrix () const
    {
      return (cloud_matrix_);
    }

    /// @brief Sets the transform matrix for the cloud.
    /// @details The passed matrix is used to transform the cloud for
    /// rendering only and does not affect the values of the points stored.
    /// @pre The passed pointer represents a matrix having valid memory of at
    /// least MATRIX_SIZE elements.
    /// @param matrix a 1-D array representing (4 x 4) matrix in
    /// using OpenGL's column-major format.
    void
    loadMatrix (const float* matrix);

    /// @brief Right multiplies the cloud matrix with the passed matrix
    /// @details The application of this matrix effectively transforms the
    /// cloud from its current state.  The passed matrix is used for display
    /// only and does not affect the values of the points stored.
    /// @pre The passed pointer represents a matrix having valid memory of at
    /// least MATRIX_SIZE elements.
    /// @param matrix a 1-D array representing (4 x 4) matrix in
    /// using OpenGL's column-major format.
    void
    multMatrix (const float* matrix);

    /// @brief Sets the selection transform matrix to the one passed.
    /// @details The selection matrix represents the local transformations
    /// applied to the selected points. The matrix is used relative to the
    /// cloud's state after the application of its own matrices  which can be
    /// modified by loadMatrix and multMatrix functions.
    /// @pre The passed pointer represents a matrix having valid memory of at
    /// least MATRIX_SIZE elements.
    /// @param matrix a 1-D array representing (4 x 4) matrix in
    /// using OpenGL's column-major format.
    /// @sa loadMatrix multMatrix
    void
    setSelectionRotation (const float* matrix);

    void
    setSelectionTranslation (float dx, float dy, float dz);

    /// @brief Sets the selected points.
    /// @details The cloud object is responsible for its display. As we have
    /// tried to adopt a lazy approach in the application of modifications to
    /// the cloud, the cloud must be notified of the selected points.  This is
    /// required as the user may move the selected points and we do not wish for
    /// the selected points to be drawn twice, once in the user-updated position
    /// and another in the points original location.
    /// @pre Assumes that the selection stores the selected indices of the
    /// points sorted.
    /// @param A pointer pointing to a selection object.
    /// @remarks This has been implemented using a weak pointer to allow a lazy
    /// update to occur.  When a selection is destroyed we can switch to
    /// a faster rendering mode; this also occurs if the selection object is
    /// empty.
    void
    setSelection (SelectionPtr selection_ptr);

    /// @brief Sets the RGB values for coloring points in COLOR_BY_PURE mode.
    /// @param r the value for red color
    /// @param g the value for the green color
    /// @param b the value for the blue color
    void
    setRGB (float r, float g, float b);

    /// @brief Sets the RGB values used for highlighting the selected points.
    /// @param r the value for red color
    /// @param g the value for the green color
    /// @param b the value for the blue color
    void
    setHighlightColor (float r, float g, float b);

    /// @brief Renders the cloud and highlights any selected points.
    /// @param disableHighlight Defaults to false.  If true the selected points
    /// will not be drawn.
    /// @sa setColorRampAxis, setColorRamp
    void
    draw (bool disable_highlight = false) const;

    /// @brief Renders the cloud and highlights any selected points.
    /// @details The colors of the non-selected points come from a 1D texture
    /// which is implemented by a color ramp.
    void
    drawWithTexture () const;

    /// @brief Renders the cloud and highlights any selected points.
    /// @details The colors of the non-selected points uses the native color
    /// of the original points
    /// @pre The cloud should be originally colored.
    void
    drawWithRGB () const;

    /// @brief Renders the cloud and highlights any selected points.
    /// @details The non-selected points are in a single color
    void
    drawWithPureColor () const;

    /// @brief Renders the cloud with the color used for highlighting the
    /// selected points.
    void
    drawWithHighlightColor () const;

    /// @brief Sets the axis along which the displyed points should have the
    /// color ramp applied.
    /// @param a The axis id describing which direction the ramp should be
    /// applied.
    inline
    void
    setColorRampAxis(Axis a)
    {
      color_ramp_axis_ = a;
    }

    /// @brief Enables/Disables the use of the color ramp in display.
    /// @details The color ramp aids in the visualization of the displayed
    /// points by varying the color according to a linear ramp along one of the
    /// axes.
    /// @param onOff True enables the use of the color ramp and false disables.
    inline
    void
    setColorRamp(bool on_off)
    {
      use_color_ramp_ = on_off;
    }

    /// @brief Appends a new 3D point to the cloud.
    /// @param point the new point to be added.
    void
    append (const Point3D& point);
    
    /// @brief Appends the points of the passed cloud to this cloud.
    /// @param cloud the cloud to be appended to this cloud.
    void
    append (const Cloud& cloud);
    
    /// @brief Removes the points in selection from the cloud.
    /// @details Each indexed point in the selection will be removed from this
    /// container.
    /// @pre The index of each point in the selection is expected to be within
    /// the limits of the cloud.  For debugging this is currently checked by an
    /// assert.  Also, it is expected that the selection indices are sorted.
    /// @param selection a selection object which stores the indices of the
    /// selected points.
    /// @remarks This function requires the use of Selection::isSelected and its
    /// complexity can vary based on the implementation of that function.
    void
    remove (const Selection& selection);

    /// @brief Gets the size of the cloud
    inline
    unsigned int
    size () const
    {
      return (cloud_.size());
    }

    /// @brief Sets the size of the cloud of this object to the passed new size
    /// @details If the size is smaller than the current size, only the first
    /// new_size points will be kept, the rest being dropped. If new_size is
    /// larger than the current size, the new points required to fill the
    /// extended region are created with its default constructor.
    /// @param new_size the new size of the cloud.
    void
    resize (unsigned int new_size);
    
    /// @brief Removes all points from the cloud and resets the object
    void
    clear ();

    /// @brief Set the sizes used for rendering the unselected points.
    /// @param size The size, in pixels, used for rendering the points.
    void
    setPointSize (int size);

    /// @brief Set the sizes used for rendering the selected points.
    /// @param size The size, in pixels, used for rendering the points.
    void
    setHighlightPointSize (int size);

    /// @brief Compute the transformed coordinates of the indexed point in the
    /// cloud according to the object transform.
    /// @details This applies the object rotation and translation of the
    /// indexed point according to the user transforms.
    /// @param index The index of the point whose coordinates are
    /// transformed.
    /// @return The transformed point.
    Point3D
    getObjectSpacePoint (unsigned int index) const;

    /// @brief Compute the transformed coordinates of the indexed point in the
    /// cloud to match the display.
    /// @details To save on computation, the points in the display are not
    /// transformed on the cpu side, instead the gpu is allowed to manipulate
    /// them for display.  This function performs the same manipulation and
    /// returns the transformed point.
    /// @param index The index of the point whose coordinates are
    /// transformed according to the display.
    /// @return The transformed point.
    Point3D
    getDisplaySpacePoint (unsigned int index) const;

    /// @brief Compute the transformed coordinates of the all the points in the
    /// cloud to match the display.
    /// @details To save on computation, the points in the display are not
    /// transformed on the cpu side, instead the gpu is allowed to manipulate
    /// them for display.  This function performs the same manipulation and
    /// returns the transformed points.
    /// @param pts a vector used to store the points whose coordinates are
    /// transformed.
    void
    getDisplaySpacePoints (std::vector<Point3D>& pts) const;

    /// @brief Returns a const reference to the internal representation of this
    /// object.
    const Cloud3D&
    getInternalCloud () const;

    /// @brief Places the points in the copy buffer into the cloud according
    /// to the indices in the selection.
    void
    restore (const CopyBuffer& copy_buffer, const Selection& selection);

    /// @brief Get statistics of the selected points in string.
    std::string
    getStat () const;

    /// Default Point Size
    static const float DEFAULT_POINT_DISPLAY_SIZE_;
    /// Default Highlight Point Size
    static const float DEFAULT_POINT_HIGHLIGHT_SIZE_;
    /// Default Point Color - Red componenet
    static const float DEFAULT_POINT_DISPLAY_COLOR_RED_;
    /// Default Point Color - Green componenet
    static const float DEFAULT_POINT_DISPLAY_COLOR_GREEN_;
    /// Default Point Color - Blue componenet
    static const float DEFAULT_POINT_DISPLAY_COLOR_BLUE_;
    /// Default Point Highlight Color - Red componenet
    static const float DEFAULT_POINT_HIGHLIGHT_COLOR_RED_;
    /// Default Point Highlight Color - Green componenet
    static const float DEFAULT_POINT_HIGHLIGHT_COLOR_GREEN_;
    /// Default Point Highlight Color - Blue componenet
    static const float DEFAULT_POINT_HIGHLIGHT_COLOR_BLUE_;

  private:
    /// @brief Computes the point cloud related members.
    /// @details The cloud keeps track of certain values related to the points
    /// in the cloud.  These include the minimum coordinates and the ranges in
    /// the coordinate directions.
    /// @pre Assumes that there is at least one dimension of the point cloud
    /// that has non-zero range.
    void
    updateCloudMembers ();

    /// @brief Enable the texture used for rendering the cloud
    void
    enableTexture () const;

    /// @brief Disable the texture used for rendering the cloud
    void
    disableTexture() const;

    /// The internal representation of the cloud
    Cloud3D cloud_;

    /// @breif A weak pointer pointing to the selection object.
    /// @details This implementation uses the weak pointer to allow for a lazy
    /// update of the cloud if the selection object is destroyed.
    boost::weak_ptr<Selection> selection_wk_ptr_;

    /// Flag that indicates whether a color ramp should be used (true) or not
    /// (false) when displaying the cloud
    bool use_color_ramp_;

    /// Flag that indicates whether the cloud should be colored with its own
    /// color
    bool use_native_color_;

    /// The axis which the color ramp is to be applied when drawing the cloud
    Axis color_ramp_axis_;

    /// A scale value used to normalize the display of clouds.  This is simply
    /// one over the maximum of the range in each coordinate direction
    float display_scale_;

    /// The center coordinate values in the point cloud.  This is used for
    /// display.
    float center_xyz_[XYZ_SIZE];

    /// The minimum coordinate values in the point cloud.  This is used for
    /// display.
    float min_xyz_[XYZ_SIZE];

    /// The maximum coordinate values in the point cloud.  This is used for
    /// display.
    float max_xyz_[XYZ_SIZE];

    /// A (4x4) OpenGL transform matrix for rendering the cloud
    float cloud_matrix_[MATRIX_SIZE];

    /// A (4x4) OpenGL transform matrix specifying the relative transformations
    /// that are applied to the selected points in the cloud when drawing them
    /// as highlighted.
    float select_matrix_[MATRIX_SIZE];

    /// A vector of indices for every point in the cloud.  This vector is used
    /// when a selection is set and sorted such that the selected indices
    /// appear first in the vector.  This is used during display to allow for
    /// separate indexed drawing of the selection and the point cloud.  Note
    /// that this vector is partitioned according to selected and not-selected.
    IndexVector partitioned_indices_;

    /// The size used for rendering the unselected points in the cloud
    float point_size_;

    /// The size used for rendering the selected points in the cloud
    float selected_point_size_;

    /// The R, G, B values used for coloring each points when the current
    /// color scheme is COLOR_BY_PURE.
    float color_[RGB];

    /// The R, G, B values used for highlighting the selected points.
    float highlight_color_[RGB];

    /// The translations on x, y, and z axis on the selected points.
    float select_translate_x_, select_translate_y_, select_translate_z_;
};
#endif // CLOUD_H_




