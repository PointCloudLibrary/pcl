/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI

#ifndef __OPENNI_DEVICE_ONI__
#define __OPENNI_DEVICE_ONI__

#include "openni_device.h"
#include "openni_driver.h"

namespace openni_wrapper
{

  /**
   * @brief Concrete implementation of the interface OpenNIDevice for a virtual device playing back an ONI file.
   * @author Suat Gedikli
   * @date 19. june 2011
   * @ingroup io
   */
  class DeviceONI : public OpenNIDevice
  {
    friend class OpenNIDriver;
  public:
    DeviceONI (xn::Context& context, const std::string& file_name, bool repeat = false, bool streaming = true);
    virtual ~DeviceONI () throw ();

    virtual void startImageStream ();
    virtual void stopImageStream ();

    virtual void startDepthStream ();
    virtual void stopDepthStream ();

    virtual void startIRStream ();
    virtual void stopIRStream ();

    virtual bool isImageStreamRunning () const throw ();
    virtual bool isDepthStreamRunning () const throw ();
    virtual bool isIRStreamRunning () const throw ();

    virtual bool isImageResizeSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const throw ();

    /** \brief Trigger a new frame in the ONI stream.
      * \param[in] relative_offset the relative offset in case we want to seek in the file
      */
    bool 
    trigger (int relative_offset = 0);

    bool isStreaming () const throw ();

    /** \brief Check if there is any data left in the ONI file to process. */
    inline bool
    hasDataLeft ()
    {
      return (!player_.IsEOF ());
    }

  protected:
    virtual boost::shared_ptr<Image> getCurrentImage (boost::shared_ptr<xn::ImageMetaData> image_meta_data) const throw ();

    void PlayerThreadFunction ();
    static void __stdcall NewONIDepthDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
    static void __stdcall NewONIImageDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
    static void __stdcall NewONIIRDataAvailable (xn::ProductionNode& node, void* cookie) throw ();

    xn::Player player_;
    boost::thread player_thread_;
    mutable boost::mutex player_mutex_;
    boost::condition_variable player_condition_;
    bool streaming_;
    bool depth_stream_running_;
    bool image_stream_running_;
    bool ir_stream_running_;
  };
} //namespace openni_wrapper
#endif //__OPENNI_DEVICE_ONI__
#endif //HAVE_OPENNI

