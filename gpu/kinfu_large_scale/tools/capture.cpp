/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#ifdef HAVE_OPENNI

#include <pcl/io/openni_camera/openni.h>

#include "openni_capture.h"
#include <pcl/gpu/containers/initialization.h>

using namespace pcl;
using namespace pcl::gpu;
using namespace xn;

//const std::string XMLConfig =
//"<OpenNI>"
//        "<Licenses>"
//        "<License vendor=\"PrimeSense\" key=\"0KOIk2JeIBYClPWVnMoRKn5cdY4=\"/>"
//        "</Licenses>"
//        "<Log writeToConsole=\"false\" writeToFile=\"false\">"
//                "<LogLevel value=\"3\"/>"
//                "<Masks>"
//                        "<Mask name=\"ALL\" on=\"true\"/>"
//                "</Masks>"
//                "<Dumps>"
//                "</Dumps>"
//        "</Log>"
//        "<ProductionNodes>"
//                "<Node type=\"Image\" name=\"Image1\">"
//                        "<Configuration>"
//                                "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
//                                "<Mirror on=\"false\"/>"
//                        "</Configuration>"
//                "</Node> "
//                "<Node type=\"Depth\" name=\"Depth1\">"
//                        "<Configuration>"
//                                "<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>"
//                                "<Mirror on=\"false\"/>"
//                        "</Configuration>"
//                "</Node>"
//        "</ProductionNodes>"
//"</OpenNI>";

#define REPORT_ERROR(msg) pcl::gpu::error ((msg), __FILE__, __LINE__)

struct pcl::gpu::kinfuLS::CaptureOpenNI::Impl
{
  Context context;
  ScriptNode scriptNode;
  DepthGenerator depth;
  ImageGenerator image;
  ProductionNode node;
  DepthMetaData depthMD;
  ImageMetaData imageMD;
  XnChar strError[1024];

  bool has_depth;
  bool has_image;
};

pcl::gpu::kinfuLS::CaptureOpenNI::CaptureOpenNI() : depth_focal_length_VGA (0.f), baseline (0.f), shadow_value (0), no_sample_value (0), pixelSize (0.0), max_depth (0) {}
pcl::gpu::kinfuLS::CaptureOpenNI::CaptureOpenNI(int device) {open (device); }
pcl::gpu::kinfuLS::CaptureOpenNI::CaptureOpenNI(const std::string& filename) {open (filename); }
pcl::gpu::kinfuLS::CaptureOpenNI::~CaptureOpenNI() { release (); }

void
pcl::gpu::kinfuLS::CaptureOpenNI::open (int device)
{
  impl_.reset ( new Impl () );

  XnMapOutputMode mode;
  mode.nXRes = XN_VGA_X_RES;
  mode.nYRes = XN_VGA_Y_RES;
  mode.nFPS = 30;

  XnStatus rc;
  rc = impl_->context.Init ();
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  xn::NodeInfoList devicesList;
  rc = impl_->context.EnumerateProductionTrees ( XN_NODE_TYPE_DEVICE, nullptr, devicesList, nullptr );
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  xn::NodeInfoList::Iterator it = devicesList.Begin ();
  for (int i = 0; i < device; ++i)
    it++;

  NodeInfo node = *it;
  rc = impl_->context.CreateProductionTree ( node, impl_->node );
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  XnLicense license;
  const char* vendor = "PrimeSense";
  const char* key = "0KOIk2JeIBYClPWVnMoRKn5cdY4=";
  strncpy (license.strKey, key, sizeof (license.strKey));
  strncpy (license.strVendor, vendor, sizeof (license.strVendor));

  rc = impl_->context.AddLicense (license);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "licence failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  rc = impl_->depth.Create (impl_->context);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Depth generator  failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }
  //rc = impl_->depth.SetIntProperty("HoleFilter", 1);
  impl_->depth.SetMapOutputMode (mode);
  impl_->has_depth = true;

  rc = impl_->image.Create (impl_->context);
  if (rc != XN_STATUS_OK)
  {
    printf ("Image generator creation failed: %s\n", xnGetStatusString (rc));
    impl_->has_image = false;
  }
  else
  {
    impl_->has_image = true;
    impl_->image.SetMapOutputMode (mode);
  }

  getParams ();

  rc = impl_->context.StartGeneratingAll ();
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Start failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }
}

void
pcl::gpu::kinfuLS::CaptureOpenNI::open (const std::string& filename)
{
  impl_.reset ( new Impl () );

  XnStatus rc;

  rc = impl_->context.Init ();
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  rc = impl_->context.OpenFileRecording (filename.c_str (), impl_->node);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "Open failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  rc = impl_->context.FindExistingNode (XN_NODE_TYPE_DEPTH, impl_->depth);
  impl_->has_depth = (rc == XN_STATUS_OK);

  rc = impl_->context.FindExistingNode (XN_NODE_TYPE_IMAGE, impl_->image);
  impl_->has_image = (rc == XN_STATUS_OK);

  if (!impl_->has_depth)
    REPORT_ERROR ("No depth nodes. Check your configuration");

  if (impl_->has_depth)
    impl_->depth.GetMetaData (impl_->depthMD);

  if (impl_->has_image)
    impl_->image.GetMetaData (impl_->imageMD);

  // RGB is the only image format supported.
  if (impl_->imageMD.PixelFormat () != XN_PIXEL_FORMAT_RGB24)
    REPORT_ERROR ("Image format must be RGB24\n");

  getParams ();
}

void
pcl::gpu::kinfuLS::CaptureOpenNI::release ()
{
  if (impl_)
  {
    impl_->context.StopGeneratingAll ();
    impl_->context.Release ();
  }

  impl_.reset ();
  depth_focal_length_VGA = 0;
  baseline = 0.f;
  shadow_value = 0;
  no_sample_value = 0;
  pixelSize = 0.0;
}

bool
pcl::gpu::kinfuLS::CaptureOpenNI::grab (PtrStepSz<const unsigned short>& depth, PtrStepSz<const RGB>& rgb24)
{
  XnStatus rc = XN_STATUS_OK;

  rc = impl_->context.WaitAndUpdateAll ();
  if (rc != XN_STATUS_OK)
    return printf ("Read failed: %s\n", xnGetStatusString (rc)), false;

  if (impl_->has_depth)
  {
    impl_->depth.GetMetaData (impl_->depthMD);
    const XnDepthPixel* pDepth = impl_->depthMD.Data ();
    int x = impl_->depthMD.FullXRes ();
    int y = impl_->depthMD.FullYRes ();
    depth.cols = x;
    depth.rows = y;
    depth.data = pDepth;
    depth.step = x * depth.elemSize ();
  }
  else
    printf ("no depth\n");

  if (impl_->has_image)
  {
    impl_->image.GetMetaData (impl_->imageMD);
    const XnRGB24Pixel* pImage = impl_->imageMD.RGB24Data ();
    int x = impl_->imageMD.FullXRes ();
    int y = impl_->imageMD.FullYRes ();

    rgb24.data = (const RGB*)pImage;
    rgb24.cols = x;
    rgb24.rows = y;
    rgb24.step = x * rgb24.elemSize ();
  }
  else
  {
    printf ("no image\n");
    rgb24.data = nullptr;
    rgb24.cols = rgb24.rows = rgb24.step = 0;
  }

  return impl_->has_image || impl_->has_depth;
}

void
pcl::gpu::kinfuLS::CaptureOpenNI::getParams ()
{
  XnStatus rc = XN_STATUS_OK;

  max_depth = impl_->depth.GetDeviceMaxDepth ();

  rc = impl_->depth.GetRealProperty ( "ZPPS", pixelSize );  // in mm
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "ZPPS failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  XnUInt64 depth_focal_length_SXGA_mm;   //in mm
  rc = impl_->depth.GetIntProperty ("ZPD", depth_focal_length_SXGA_mm);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "ZPD failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  XnDouble baseline_local;
  rc = impl_->depth.GetRealProperty ("LDDIS", baseline_local);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "ZPD failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }

  XnUInt64 shadow_value_local;
  rc = impl_->depth.GetIntProperty ("ShadowValue", shadow_value_local);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "ShadowValue failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }
  shadow_value = (int)shadow_value_local;

  XnUInt64 no_sample_value_local;
  rc = impl_->depth.GetIntProperty ("NoSampleValue", no_sample_value_local);
  if (rc != XN_STATUS_OK)
  {
    sprintf (impl_->strError, "NoSampleValue failed: %s\n", xnGetStatusString (rc));
    REPORT_ERROR (impl_->strError);
  }
  no_sample_value = (int)no_sample_value_local;


  // baseline from cm -> mm
  baseline = (float)(baseline_local * 10);

  //focal length from mm -> pixels (valid for 1280x1024)
  float depth_focal_length_SXGA = static_cast<float>(depth_focal_length_SXGA_mm / pixelSize);
  depth_focal_length_VGA = depth_focal_length_SXGA / 2;
}

bool
pcl::gpu::kinfuLS::CaptureOpenNI::setRegistration (bool value)
{
  XnStatus rc = XN_STATUS_OK;

  if (value)
  {
    if (!impl_->has_image)
      return false;

    if (impl_->depth.GetAlternativeViewPointCap ().IsViewPointAs (impl_->image) )
      return true;

    if (!impl_->depth.GetAlternativeViewPointCap ().IsViewPointSupported (impl_->image) )
    {
      printf ("SetRegistration failed: Unsupported viewpoint.\n");
      return false;
    }

    rc = impl_->depth.GetAlternativeViewPointCap ().SetViewPoint (impl_->image);
    if (rc != XN_STATUS_OK)
      printf ("SetRegistration failed: %s\n", xnGetStatusString (rc));

  }
  else   // "off"
  {
    rc = impl_->depth.GetAlternativeViewPointCap ().ResetViewPoint ();
    if (rc != XN_STATUS_OK)
      printf ("SetRegistration failed: %s\n", xnGetStatusString (rc));
  }

  getParams ();
  return rc == XN_STATUS_OK;
}

#endif
