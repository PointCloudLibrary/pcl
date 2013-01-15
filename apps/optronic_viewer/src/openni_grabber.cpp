/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/apps/optronic_viewer/openni_grabber.h>

#include <pcl/io/openni_grabber.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
OpenNIGrabber::
OpenNIGrabber (pcl::Grabber * grabber)
  : QThread ()
  , grabber_ (grabber)
{
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> ("pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr");
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
OpenNIGrabber::
~OpenNIGrabber ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
OpenNIGrabber::
run ()
{
  std::cerr << "run grabber thread..." << std::endl;
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud) > f = 
    boost::bind (&pcl::apps::optronic_viewer::OpenNIGrabber::cloudCallback, this, _1);
  boost::signals2::connection c1 = grabber_->registerCallback (f);
  grabber_->start ();
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
OpenNIGrabber::
cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
{
  //std::cerr << "[grabber thread] cloud received.." << std::endl;
  emit cloudReceived (cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
FotonicGrabber::
FotonicGrabber (FZ_Device_Handle_t * fotonic_device_handle)
  : QThread ()
  , fotonic_device_handle_ (fotonic_device_handle)
{
  qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> ("pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr");
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
FotonicGrabber::
~FotonicGrabber ()
{
}

//static const uint16_t kRed5     = 0x001f;
//static const uint16_t kGreen6   = 0x07e0;
//static const uint16_t kBlue5    = 0xf800;

static const uint16_t kRed5     = 0xf800;
static const uint16_t kGreen6   = 0x07e0;
static const uint16_t kBlue5    = 0x001f;

#define R16(rgb)    (uint8_t)((rgb) & kRed5)
#define G16(rgb)    (uint8_t)(((rgb) & kGreen6) >> 5)
#define B16(rgb)    (uint8_t)(((rgb) & kBlue5) >> 11)
/* Make 8 bits red, green, and blue, extracted from RGB565 word. */
#define R16_32(rgb) (uint8_t)((((rgb) & kRed5) << 3) | (((rgb) & kRed5) >> 2))
#define G16_32(rgb) (uint8_t)((((rgb) & kGreen6) >> 3) | (((rgb) & kGreen6) >> 9))
#define B16_32(rgb) (uint8_t)((((rgb) & kBlue5) >> 8) | (((rgb) & kBlue5) >> 14))

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FotonicGrabber::
run ()
{
  std::cerr << "run grabber thread..." << std::endl;

  char * frame_buffer = new char [640*480*8];

  while (true)
  {
    FZ_Result result = FZ_FrameAvailable (*fotonic_device_handle_);
    //std::cerr << "FZ_FrameAvailable: " << result << std::endl;

    if (result == FZ_Success)
    {
      size_t length_in_byte = 640*480*10;
      FZ_FRAME_HEADER frame_header;

      result = FZ_GetFrame (*fotonic_device_handle_, &frame_header, frame_buffer, &length_in_byte);
      std::cerr << "FZ_GetFrame: " << result << std::endl;
      std::cerr << "length_in_byte: " << length_in_byte << std::endl;

      if (result == FZ_Success)
      {
        std::cerr << "frame: " << frame_header.framecounter << std::endl;
        std::cerr << "  timestamp: " << frame_header.timestamp << std::endl;
        std::cerr << "  format: " << frame_header.format << std::endl;
        std::cerr << "  cols: " << frame_header.ncols << std::endl;
        std::cerr << "  rows: " << frame_header.nrows << std::endl;
        std::cerr << "  reportedframerate: " << frame_header.reportedframerate << std::endl;
        std::cerr << "  bytesperpixel: " << frame_header.bytesperpixel << std::endl;

        const int width = frame_header.ncols;
        const int height = frame_header.nrows;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        cloud->resize (width*height);
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;

        short * ptr = (short*) frame_buffer;

        for (int row_index = 0; row_index < height; ++row_index)
        {
          //if(pixelformat == FZ_PIXELFORMAT_YUV422) 
          {
            // YUV422
            FZ_YUV422_DOUBLE_PIXEL *p = (FZ_YUV422_DOUBLE_PIXEL*)ptr;
            int col = 0;
            for (int col_index = 0; col_index < width/2; ++col_index)
            {
              pcl::PointXYZRGBA & point0 = (*cloud) (col, row_index);
              ++col;
              pcl::PointXYZRGBA & point1 = (*cloud) (col, row_index);
              ++col;

              float r,g,b,u,v,u1,v1,uv1;

              u = p[col_index].u - 128.0f;
              v = p[col_index].v - 128.0f;
              v1 = 1.13983f*v;
              uv1 = -0.39465f*u - 0.58060f*v;
              u1 = 0.03211f*u;

              r = p[col_index].y1 + v1;
              g = p[col_index].y1 + uv1;
              b = p[col_index].y1 + u1;

              r = std::min (255.0f, std::max (0.0f, r));
              g = std::min (255.0f, std::max (0.0f, g));
              b = std::min (255.0f, std::max (0.0f, b));

              //*output++ = D3DCOLOR_XRGB(unsigned(r), unsigned(g), unsigned(b));
              point0.r = unsigned(r);
              point0.g = unsigned(g);
              point0.b = unsigned(b);
               
              r = p[col_index].y2 + v1;
              g = p[col_index].y2 + uv1;
              b = p[col_index].y2 + u1;

              r = std::min (255.0f, std::max (0.0f, r));
              g = std::min (255.0f, std::max (0.0f, g));
              b = std::min (255.0f, std::max (0.0f, b));
               
              //*output++ = D3DCOLOR_XRGB(unsigned(r), unsigned(g), unsigned(b));
              point1.r = unsigned(r);
              point1.g = unsigned(g);
              point1.b = unsigned(b);
            }

            ptr += width;
          } 
          //else 
          //{
          //  // Greyscale - FIXME: ignore for now
          //}          

          //{
          //  pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

          //  unsigned short c = (unsigned short) *ptr;
          //  ++ptr;

          //  //point.r = ((c >> 10) & 31) << 3;
          //  //point.g = ((c >> 5) & 31) << 3;
          //  //point.b = (c & 31) << 3;

          //  //if (row_index%50 == 0 && col_index%50 == 0)
          //  //{
          //  //  for (int i = 0; i < 16; ++i)
          //  //    if (((0x1<<i) & c) != 0)
          //  //      std::cerr << "1";
          //  //    else
          //  //      std::cerr << "0";

          //  //  std::cerr << " ";
          //  //}

          //  //point.r = R16_32(c);
          //  //point.g = G16_32(c);
          //  //point.b = B16_32(c);

          //  point.r = R16_32(c);
          //  point.g = G16_32(c);
          //  point.b = B16_32(c);
          //}

          //std::cerr << std::endl;

          for (int col_index = 0; col_index < width; ++col_index)
          {
            pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

            short z = *ptr;

            point.z = static_cast<float> (z) / 1000.0f;

            ++ptr;
          }

          //for (int col_index = 0; col_index < width; ++col_index)
          //{
          //  pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

          //  short x = *ptr;
          //  ++ptr;
          //  short y = *ptr;

          //  point.x = static_cast<float> (x) / 1000.0f;
          //  point.y = static_cast<float> (y) / 1000.0f;
          //  //point.x = static_cast<float> (col_index - width/2) / 575;
          //  //point.y = static_cast<float> (row_index - height/2) / 575;

          //  ++ptr;
          //}

          for (int col_index = 0; col_index < width; ++col_index)
          {
            pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

            short x = *ptr;
            ++ptr;

            point.x = -static_cast<float> (x) / 1000.0f;
          }

          for (int col_index = 0; col_index < width; ++col_index)
          {
            pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

            short y = *ptr;
            ++ptr;

            point.y = static_cast<float> (y) / 1000.0f;
          }

          //for (int col_index = 0; col_index < width; ++col_index)
          //{
          //  pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

          //  short z = ptr[(row_index*width + col_index)];

          //  std::cerr << z << " ";

          //  //short x = ptr[(row_index*width + col_index)*4 + 0];
          //  //short y = ptr[(row_index*width + col_index)*4 + 1];
          //  //short z = ptr[(row_index*width + col_index)*4 + 2];
          //  //short b = ptr[(row_index*width + col_index)*4 + 3];

          //  //std::cerr << "(" << col_index << ", " << row_index << "): "
          //  //  << x << ", "
          //  //  << y << ", "
          //  //  << z << ", "
          //  //  << b << std::endl;

          //  //point.x = static_cast<float> (x) / 1000.0f;
          //  //point.y = static_cast<float> (y) / 1000.0f;
          //  point.z = static_cast<float> (z) / 1000.0f;
          //  point.r = 255;
          //  point.g = 255;
          //  point.b = 255;
          //  point.a = 255;

          //  point.x = static_cast<float> (col_index - width/2) / 512;
          //  point.y = static_cast<float> (row_index - height/2) / 512;
          //  //point.z = 1.0f;
          //  //point.r = (uint8_t) (frame_buffer[(row_index*width + col_index)*4 + 0]);
          //  //point.g = (uint8_t) (frame_buffer[(row_index*width + col_index)*4 + 1]);
          //  //point.b = (uint8_t) (frame_buffer[(row_index*width + col_index)*4 + 2]);
          //  //point.a = (uint8_t) (frame_buffer[(row_index*width + col_index)*4 + 3]);
          //}
        }
        //std::cerr << std::endl;

        //for (int row_index = 0; row_index < height; row_index+=50)
        //{
        //  for (int col_index = 0; col_index < width; col_index+=50)
        //  {
        //    pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);

        //    short z = ptr[(row_index*width + col_index)];

        //    std::cerr << "(" << col_index << ", " << row_index << "): "
        //      << z << std::endl;

        //    //short x = ptr[(row_index*width + col_index)*4 + 0];
        //    //short y = ptr[(row_index*width + col_index)*4 + 1];
        //    //short z = ptr[(row_index*width + col_index)*4 + 2];
        //    //short b = ptr[(row_index*width + col_index)*4 + 3];

        //    //std::cerr << "(" << col_index << ", " << row_index << "): "
        //    //  << x << ", "
        //    //  << y << ", "
        //    //  << z << ", "
        //    //  << b << std::endl;

        ////    std::cerr << point.x << ", "
        ////      << point.y << ", "
        ////      << point.z << ", "
        ////      << static_cast<int> (point.r) << ", "
        ////      << static_cast<int> (point.g) << ", "
        ////      << static_cast<int> (point.b) << ", "
        ////      << static_cast<int> (point.a) << std::endl;
        //  }
        //}

        emit cloudReceived (cloud);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//void
//pcl::apps::optronic_viewer::
//FotonicGrabber::
//cloudCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
//{
//  //std::cerr << "[grabber thread] cloud received.." << std::endl;
//  emit cloudReceived (cloud);
//}

