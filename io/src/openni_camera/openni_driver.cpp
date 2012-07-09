/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif

#include <pcl/io/openni_camera/openni.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_camera/openni_device_kinect.h>
#include <pcl/io/openni_camera/openni_device_primesense.h>
#include <pcl/io/openni_camera/openni_device_xtion.h>
#include <pcl/io/openni_camera/openni_device_oni.h>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <locale>
#include <cctype>
#include <map>

#ifndef _WIN32
#include <libusb-1.0/libusb.h>
#else
#include <pcl/io/boost.h>
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDriver::OpenNIDriver ()
  : device_context_ ()
  , context_ ()
  , bus_map_ ()
  , serial_map_ ()
  , connection_string_map_ ()
{
  // Initialize the Engine
  XnStatus status = context_.Init ();
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("initialization failed. Reason: %s", xnGetStatusString (status));

  updateDeviceList ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned 
openni_wrapper::OpenNIDriver::updateDeviceList ()
{
  // clear current list of devices
  device_context_.clear ();
  // clear maps
  bus_map_.clear ();
  serial_map_.clear ();
  connection_string_map_.clear ();

  // enumerate all devices
  static xn::NodeInfoList node_info_list;
  XnStatus status = context_.EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);
  if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ())
    THROW_OPENNI_EXCEPTION ("enumerating devices failed. Reason: %s", xnGetStatusString (status));
  else if (node_info_list.Begin () == node_info_list.End ())
    return 0; // no exception

  for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt)
  {
    connection_string_map_[(*nodeIt).GetCreationInfo ()] = static_cast<unsigned int> (device_context_.size ());
    device_context_.push_back (DeviceContext (*nodeIt));
  }

  // enumerate depth nodes
  static xn::NodeInfoList depth_nodes;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("enumerating depth generators failed. Reason: %s", xnGetStatusString (status));

  for (xn::NodeInfoList::Iterator nodeIt = depth_nodes.Begin (); nodeIt != depth_nodes.End (); ++nodeIt)
  {
    // check if to which device this node is assigned to
    for (xn::NodeInfoList::Iterator neededIt = (*nodeIt).GetNeededNodes ().Begin (); neededIt != (*nodeIt).GetNeededNodes ().End (); ++neededIt)
    {
      if ( connection_string_map_.count ((*neededIt).GetCreationInfo ()) )
      {
        unsigned device_index = connection_string_map_[(*neededIt).GetCreationInfo ()];
        device_context_[device_index].depth_node.reset (new xn::NodeInfo(*nodeIt));
      }
    }
  }

  // enumerate image nodes
  static xn::NodeInfoList image_nodes;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);


  // Suat: This is an ugly ASUS Xtion workaround.
  if (status == XN_STATUS_OK)
  {
    //THROW_OPENNI_EXCEPTION ("enumerating image generators failed. Reason: %s", xnGetStatusString (status));

    for (xn::NodeInfoList::Iterator nodeIt = image_nodes.Begin (); nodeIt != image_nodes.End (); ++nodeIt)
    {
      // check to which device this node is assigned to
      for (xn::NodeInfoList::Iterator neededIt = (*nodeIt).GetNeededNodes ().Begin (); neededIt != (*nodeIt).GetNeededNodes ().End (); ++neededIt)
      {
        if ( connection_string_map_.count ((*neededIt).GetCreationInfo ()) )
        {
          unsigned device_index = connection_string_map_[(*neededIt).GetCreationInfo ()];
          device_context_[device_index].image_node.reset (new xn::NodeInfo(*nodeIt));
        }
      }
    }
  }

  // enumerate IR nodes
  static xn::NodeInfoList ir_nodes;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_IR, NULL, ir_nodes, NULL);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("enumerating IR generators failed. Reason: %s", xnGetStatusString (status));

  for (xn::NodeInfoList::Iterator nodeIt = ir_nodes.Begin (); nodeIt != ir_nodes.End (); ++nodeIt)
  {
    // check if to which device this node is assigned to
    for (xn::NodeInfoList::Iterator neededIt = (*nodeIt).GetNeededNodes ().Begin (); neededIt != (*nodeIt).GetNeededNodes ().End (); ++neededIt)
    {
      if ( connection_string_map_.count ((*neededIt).GetCreationInfo ()) )
      {
        unsigned device_index = connection_string_map_[(*neededIt).GetCreationInfo ()];
        device_context_[device_index].ir_node.reset (new xn::NodeInfo(*nodeIt));
      }
    }
  }

#ifndef _WIN32
  // add context object for each found device
  for (unsigned deviceIdx = 0; deviceIdx < device_context_.size (); ++deviceIdx)
  {
    // register bus@address to the corresponding context object
    unsigned short vendor_id;
    unsigned short product_id;
    unsigned char bus;
    unsigned char address;
    sscanf (device_context_[deviceIdx].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
    bus_map_ [bus][address] = deviceIdx;
  }

  // get additional info about connected devices like serial number, vendor name and prduct name
  getDeviceInfos ();
#endif
  // build serial number -> device index map
  for (unsigned deviceIdx = 0; deviceIdx < device_context_.size (); ++deviceIdx)
  {
    std::string serial_number = getSerialNumber (deviceIdx);
    if (!serial_number.empty ())
      serial_map_[serial_number] = deviceIdx;
  }


  // redundant, but needed for Windows right now and also for Xtion
  for (unsigned deviceIdx = 0; deviceIdx < device_context_.size (); ++deviceIdx)
  {
    unsigned short product_id;
    unsigned short vendor_id;

    getDeviceType(device_context_[deviceIdx].device_node.GetCreationInfo (), vendor_id, product_id );

#if _WIN32
    if (vendor_id == 0x45e)
    {
      strcpy (const_cast<char*> (device_context_[deviceIdx].device_node.GetDescription ().strVendor), "Microsoft");
      strcpy (const_cast<char*> (device_context_[deviceIdx].device_node.GetDescription ().strName), "Xbox NUI Camera");
    }
    else
#endif
    if (vendor_id == 0x1d27 && device_context_[deviceIdx].image_node.get () == 0)
    {
      strcpy (const_cast<char*> (device_context_[deviceIdx].device_node.GetDescription ().strVendor), "ASUS");
      strcpy (const_cast<char*> (device_context_[deviceIdx].device_node.GetDescription ().strName), "Xtion Pro");
    }
  }
  return (static_cast<unsigned int> (device_context_.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDriver::stopAll ()
{
  XnStatus status = context_.StopGeneratingAll ();
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("stopping all streams failed. Reason: %s", xnGetStatusString (status));
}

openni_wrapper::OpenNIDriver::~OpenNIDriver () throw ()
{
  // no exception during destuctor
  try
  {
    stopAll ();
  }
  catch (...)
  {
  }

#if (XN_MINOR_VERSION >= 3)
  context_.Release ();
#else
  context_.Shutdown ();
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<openni_wrapper::OpenNIDevice> 
openni_wrapper::OpenNIDriver::createVirtualDevice (const std::string& path, bool repeat, bool stream) const
{
  return (boost::shared_ptr<OpenNIDevice> (new DeviceONI (context_, path, repeat, stream)));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<openni_wrapper::OpenNIDevice> 
openni_wrapper::OpenNIDriver::getDeviceByIndex (unsigned index) const
{
  using namespace std;

  if (index >= device_context_.size ())
    THROW_OPENNI_EXCEPTION ("Device index out of range. Only %d devices connected but device %d requested.", device_context_.size (), index);
  boost::shared_ptr<openni_wrapper::OpenNIDevice> device = device_context_[index].device.lock ();
  if (!device)
  {
    unsigned short vendor_id;
    unsigned short product_id;
    getDeviceType (device_context_[index].device_node.GetCreationInfo (), vendor_id, product_id );

    if (vendor_id == 0x45e)
    {
      device.reset (new DeviceKinect (context_, 
                                      device_context_[index].device_node,
                                      *device_context_[index].image_node, 
                                      *device_context_[index].depth_node,
                                      *device_context_[index].ir_node));
      device_context_[index].device = device;
    }
    else if (vendor_id == 0x1d27)
    {
      if (device_context_[index].image_node.get())
        device.reset (new DevicePrimesense (context_, 
                                            device_context_[index].device_node, 
                                            *device_context_[index].image_node, 
                                            *device_context_[index].depth_node,
                                            *device_context_[index].ir_node));
      else
        device.reset (new DeviceXtionPro (context_, 
                                          device_context_[index].device_node, 
                                          *device_context_[index].depth_node, 
                                          *device_context_[index].ir_node));
      device_context_[index].device = device;
    }
    else
    {
      THROW_OPENNI_EXCEPTION ("Vendor %s (%s) unknown!",
                              getVendorName (index), vendor_id);
    }
  }
  return (device);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<openni_wrapper::OpenNIDevice>
openni_wrapper::OpenNIDriver::getDeviceBySerialNumber (const std::string& serial_number) const
{
  std::map<std::string, unsigned>::const_iterator it = serial_map_.find (serial_number);

  if (it != serial_map_.end ())
  {
    return getDeviceByIndex (it->second);
  }

  THROW_OPENNI_EXCEPTION ("No device with serial number \'%s\' found", serial_number.c_str ());

  // because of warnings!!!
  return (boost::shared_ptr<openni_wrapper::OpenNIDevice> (static_cast<OpenNIDevice*> (NULL)));
}

#ifndef _WIN32
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<openni_wrapper::OpenNIDevice>
openni_wrapper::OpenNIDriver::getDeviceByAddress (unsigned char bus, unsigned char address) const
{
  std::map<unsigned char, std::map<unsigned char, unsigned> >::const_iterator busIt = bus_map_.find (bus);
  if (busIt != bus_map_.end ())
  {
    std::map<unsigned char, unsigned>::const_iterator devIt = busIt->second.find (address);
    if (devIt != busIt->second.end ())
    {
      return getDeviceByIndex (devIt->second);
    }
  }

  THROW_OPENNI_EXCEPTION ("No device on bus: %d @ %d found", bus, address);

  // because of warnings!!!
  return (boost::shared_ptr<OpenNIDevice > (static_cast<OpenNIDevice*> (NULL)));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
openni_wrapper::OpenNIDriver::getDeviceInfos () throw ()
{
  libusb_context *context = NULL;
  int result;
  result = libusb_init (&context); //initialize a library session

  if (result < 0)
    return;

  libusb_device **devices;
  int count = static_cast<int> (libusb_get_device_list (context, &devices));
  if (count < 0)
    return;

  for (int devIdx = 0; devIdx < count; ++devIdx)
  {
    libusb_device* device = devices[devIdx];
    uint8_t busId = libusb_get_bus_number (device);
    std::map<unsigned char, std::map<unsigned char, unsigned> >::const_iterator busIt = bus_map_.find (busId);
    if (busIt == bus_map_.end ())
      continue;

    uint8_t address = libusb_get_device_address (device);
    std::map<unsigned char, unsigned>::const_iterator addressIt = busIt->second.find (address);
    if (addressIt == busIt->second.end ())
      continue;

    unsigned nodeIdx = addressIt->second;
    xn::NodeInfo& current_node = device_context_[nodeIdx].device_node;
    XnProductionNodeDescription& description = const_cast<XnProductionNodeDescription&>(current_node.GetDescription ());

    libusb_device_descriptor descriptor;
    result = libusb_get_device_descriptor (devices[devIdx], &descriptor);

    if (result < 0)
    {
      strcpy (description.strVendor, "unknown");
      strcpy (description.strName, "unknown");
      current_node.SetInstanceName ("");
    }
    else
    {
      libusb_device_handle* dev_handle;
      result = libusb_open (device, &dev_handle);
      if (result < 0)
      {
        strcpy (description.strVendor, "unknown");
        strcpy (description.strName, "unknown");
        current_node.SetInstanceName ("");
      }
      else
      {
        unsigned char buffer[1024];
        libusb_get_string_descriptor_ascii (dev_handle, descriptor.iManufacturer, buffer, 1024);
        strcpy (description.strVendor, reinterpret_cast<char*> (buffer));

        libusb_get_string_descriptor_ascii (dev_handle, descriptor.iProduct, buffer, 1024);
        strcpy (description.strName, reinterpret_cast<char*> (buffer));

        int len = libusb_get_string_descriptor_ascii (dev_handle, descriptor.iSerialNumber, buffer, 1024);
        if (len > 4)
          current_node.SetInstanceName (reinterpret_cast<char*> (buffer));
        else
          current_node.SetInstanceName ("");

        libusb_close (dev_handle);
      }
    }
  }
  libusb_free_device_list (devices, 1);
  libusb_exit (context);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDriver::getSerialNumber (unsigned index) const throw ()
{
#ifndef _WIN32
  return (device_context_[index].device_node.GetInstanceName ());
#else
  std::string info = device_context_[index].device_node.GetCreationInfo ();
  boost::char_separator<char> sep ("#");
  boost::tokenizer<boost::char_separator<char> > tokens (info, sep);
  boost::tokenizer<boost::char_separator<char> >::iterator itr = tokens.begin ();
  itr++;
  itr++;
  std::string sn = *itr;
  device_context_[index].device_node.SetInstanceName(sn.c_str ());
  return (device_context_[index].device_node.GetInstanceName ());
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
openni_wrapper::OpenNIDriver::getDeviceType (const std::string& connectionString, unsigned short& vendorId, unsigned short& productId)
{
#if _WIN32
    // expected format: "\\?\usb#vid_[ID]&pid_[ID]#[SERIAL]#{GUID}"
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> separators("#&_");
    tokenizer tokens (connectionString, separators);

    unsigned int tokenIndex = 0;
    for (tokenizer::iterator tok_iter = tokens.begin (); tok_iter != tokens.end (); ++tok_iter, tokenIndex++)
    {
      std::string tokenValue = *tok_iter;

      switch (tokenIndex) 
      {
        case 2:    // the vendor ID
          sscanf (tokenValue.c_str (), "%hx", &vendorId);
          break;
        case 4: // the product ID
          sscanf (tokenValue.c_str (), "%hx", &productId);
          break;
      }
    }
#else
    unsigned char bus;
    unsigned char address;
    sscanf (connectionString.c_str(), "%hx/%hx@%hhu/%hhu", &vendorId, &productId, &bus, &address);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDriver::getConnectionString (unsigned index) const throw ()
{
  return (device_context_[index].device_node.GetCreationInfo ());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDriver::getVendorName (unsigned index) const throw ()
{
  return (device_context_[index].device_node.GetDescription ().strVendor);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* 
openni_wrapper::OpenNIDriver::getProductName (unsigned index) const throw ()
{
  return (device_context_[index].device_node.GetDescription ().strName);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short 
openni_wrapper::OpenNIDriver::getVendorID (unsigned index) const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  getDeviceType (device_context_[index].device_node.GetCreationInfo (), vendor_id, product_id);
#endif
  return (vendor_id);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short 
openni_wrapper::OpenNIDriver::getProductID (unsigned index) const throw ()
{
  unsigned short vendor_id;
  unsigned short product_id;
#ifndef _WIN32
  unsigned char bus;
  unsigned char address;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);

#else
  getDeviceType (device_context_[index].device_node.GetCreationInfo (), vendor_id, product_id);
#endif
  return (product_id);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char 
openni_wrapper::OpenNIDriver::getBus (unsigned index) const throw ()
{
  unsigned char bus = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char address;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return (bus);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char 
openni_wrapper::OpenNIDriver::getAddress (unsigned index) const throw ()
{
  unsigned char address = 0;
#ifndef _WIN32
  unsigned short vendor_id;
  unsigned short product_id;
  unsigned char bus;
  sscanf (device_context_[index].device_node.GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &vendor_id, &product_id, &bus, &address);
#endif
  return (address);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDriver::DeviceContext::DeviceContext (const xn::NodeInfo& device, xn::NodeInfo* image, xn::NodeInfo* depth, xn::NodeInfo* ir)
: device_node (device)
, image_node (image)
, depth_node (depth)
, ir_node (ir)
, device ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDriver::DeviceContext::DeviceContext (const xn::NodeInfo& device)
: device_node (device)
, image_node (static_cast<xn::NodeInfo*> (0))
, depth_node (static_cast<xn::NodeInfo*> (0))
, ir_node (static_cast<xn::NodeInfo*> (0))
, device ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
openni_wrapper::OpenNIDriver::DeviceContext::DeviceContext (const DeviceContext& other)
: device_node (other.device_node)
, image_node (other.image_node)
, depth_node (other.depth_node)
, ir_node (other.ir_node)
, device (other.device)
{
}

#endif
