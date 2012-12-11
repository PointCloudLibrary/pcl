

#include "pcl/pcl_config.h"

#ifndef __PCL_IO_IMAGE_GRABBER__
#define __PCL_IO_IMAGE_GRABBER__

#include <pcl/io/grabber.h>
#include <pcl/common/time_trigger.h>
#include <string>
#include <vector>
#include <pcl/ros/conversions.h>

namespace pcl
{
  /** \brief Base class for Image file grabber.
   * \ingroup io
   */
  class PCL_EXPORTS ImageGrabberBase : public Grabber
  {
    public:
    /** \brief Constructor taking a folder of depth+[rgb] images.
     * \param[in] directory Directory which contains an ordered set of images corresponding to an [RGB]D video, stored as TIFF, PNG, JPG, or PPM files. The naming convention is: frame_[timestamp]_["depth"/"rgb"].[extension]
     * \param[in] frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
     * \param[in] repeat whether to play PCD file in an endless loop or not.
     */
    ImageGrabberBase (const std::string& directory, float frames_per_second, bool repeat, bool pclzf_mode);

    /** \brief Constructor taking a list of paths to PCD files, that are played in the order they appear in the list.
     * \param[in] depth_image_files Path to the depth image files files.
     * \param[in] frames_per_second frames per second. If 0, start() functions like a trigger, publishing the next PCD in the list.
     * \param[in] repeat whether to play PCD file in an endless loop or not.
     */
    ImageGrabberBase (const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat);

    /** \brief Copy constructor.
     * \param[in] src the Image Grabber base object to copy into this
     */
    ImageGrabberBase (const ImageGrabberBase &src) : Grabber (), impl_ ()
    {
      *this = src;
    }

    /** \brief Copy operator.
     * \param[in] src the Image Grabber base object to copy into this
     */
    ImageGrabberBase&
    operator = (const ImageGrabberBase &src)
    {
      impl_ = src.impl_;
      return (*this);
    }

    /** \brief Virtual destructor. */
    virtual ~ImageGrabberBase () throw ();

    /** \brief Starts playing the list of PCD files if frames_per_second is > 0. Otherwise it works as a trigger: publishes only the next PCD file in the list. */
    virtual void 
    start ();
      
    /** \brief Stops playing the list of PCD files if frames_per_second is > 0. Otherwise the method has no effect. */
    virtual void 
    stop ();
      
    /** \brief Triggers a callback with new data */
    virtual void 
    trigger ();

    /** \brief whether the grabber is started (publishing) or not.
     * \return true only if publishing.
     */
    virtual bool 
    isRunning () const;
      
    /** \return The name of the grabber */
    virtual std::string 
    getName () const;
      
    /** \brief Rewinds to the first PCD file in the list.*/
    virtual void 
    rewind ();

    /** \brief Returns the frames_per_second. 0 if grabber is trigger-based */
    virtual float 
    getFramesPerSecond () const;

    /** \brief Returns whether the repeat flag is on */
    bool 
    isRepeatOn () const;

    /** \brief Manually set RGB image files.
     * \param[in] rgb_image_files A vector of [tiff/png/jpg/ppm] files to use as input. There must be a 1-to-1 correspondence between these and the depth images you set
     */
    void
    setRGBImageFiles (const std::vector<std::string>& rgb_image_files);

    /** \brief Define custom focal length and center pixel. This will override ANY other setting of parameters for the duration of the grabber's life, whether by factory defaults or explicitly read from a frame_[timestamp].xml file. 
     *  \param[in] fx Horizontal focal length
     *  \param[in] fy Vertical focal length
     *  \param[in] cx Horizontal coordinates of the principal point
     *  \param[in] cy Vertical coordinates of the principal point
     */
    virtual void
    setCameraIntrinsics (float fx, float fy, float cx, float cy);
    
    /** \brief Get the current focal length and center pixel. If the intrinsics have been manually set with @setCameraIntrinsics@, this will return those values. Else, if start () has been called and the grabber has found a frame_[timestamp].xml file, this will return the most recent values read. Else, returns factory defaults.
     *  \param[out] fx Horizontal focal length
     *  \param[out] fy Vertical focal length
     *  \param[out] cx Horizontal coordinates of the principal point
     *  \param[out] cy Vertical coordinates of the principal point
     */
    virtual void
    getCameraIntrinsics (float &fx, float &fy, float &cx, float &cy) const;

    /** \brief Define the units the depth data is stored in.
     *  Defaults to mm (0.001), meaning a brightness of 1000 corresponds to 1 m*/
    void
    setDepthImageUnits (float units);

    /** \brief Convenience function to see how many frames this consists of */
    size_t
    numFrames () const;

    private:
    virtual void 
    publish (const sensor_msgs::PointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) const = 0;

    // to separate and hide the implementation from interface: PIMPL
    struct ImageGrabberImpl;
    ImageGrabberImpl* impl_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename T> class PointCloud;
  template <typename PointT> class ImageGrabber : public ImageGrabberBase
  {
    public:
    ImageGrabber (const std::string& dir, float frames_per_second = 0, bool repeat = false, bool pclzf_mode = false);
    ImageGrabber (const std::vector<std::string>& depth_image_files, float frames_per_second = 0, bool repeat = false);
    

    boost::shared_ptr<pcl::PointCloud<PointT> >
    getCloud (size_t i ) const;

    protected:
    virtual void 
      publish (const sensor_msgs::PointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) const;
    boost::signals2::signal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>* signal_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    ImageGrabber<PointT>::ImageGrabber (const std::string& dir, float frames_per_second, bool repeat, bool pclzf_mode)
    : ImageGrabberBase (dir, frames_per_second, repeat, pclzf_mode)
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    ImageGrabber<PointT>::ImageGrabber (const std::vector<std::string>& depth_image_files, float frames_per_second, bool repeat)
    : ImageGrabberBase (depth_image_files, frames_per_second, repeat), signal_ ()
  {
    signal_ = createSignal<void (const boost::shared_ptr<const pcl::PointCloud<PointT> >&)>();
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
    void ImageGrabber<PointT>::publish (const sensor_msgs::PointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) const
  {
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    pcl::fromROSMsg (blob, *cloud);
    cloud->sensor_origin_ = origin;
    cloud->sensor_orientation_ = orientation;

    signal_->operator () (cloud);
  }
}
#endif
