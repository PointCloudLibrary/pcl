/* $NoKeywords: $ */
/*
//
// Copyright (c) 1993-2012 Robert McNeel & Associates. All rights reserved.
// OpenNURBS, Rhinoceros, and Rhino3D are registered trademarks of Robert
// McNeel & Associates.
//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY.
// ALL IMPLIED WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND OF
// MERCHANTABILITY ARE HEREBY DISCLAIMED.
//				
// For complete openNURBS copyright information see <http://www.opennurbs.org>.
//
////////////////////////////////////////////////////////////////
*/

#if !defined(OPENNURBS_PLUGINLIST_INC_)
#define OPENNURBS_PLUGINLIST_INC_

/*
Description:
  The ON_PluginRef class is used to store a list of
  application plug-ins that may have saved user data
  in a 3dm file so they can be loaded as needed for
  reading their user data.
*/
class ON_CLASS ON_PlugInRef
{
public:
  ON_PlugInRef();

  // executable informtion
  ON_UUID m_plugin_id;
  int m_plugin_type; // CRhinoPlugIn::plugin_type enum value
  int m_plugin_platform; // 0 = unknown, 1 = C++, 2 = .NET
  int m_plugin_sdk_version;
  int m_plugin_sdk_service_release;
  ON_wString m_plugin_name;
  ON_wString m_plugin_version;
  ON_wString m_plugin_filename; // name of executable file

  // developer contact information
  ON_wString m_developer_organization;
  ON_wString m_developer_address;
  ON_wString m_developer_country;
  ON_wString m_developer_phone;
  ON_wString m_developer_email;
  ON_wString m_developer_website;
  ON_wString m_developer_updateurl;
  ON_wString m_developer_fax;

  void Default();

  bool Write( ON_BinaryArchive& file ) const;
  bool Read( ON_BinaryArchive& file );

  void Dump(ON_TextLog& text_log) const;
};


#if defined(ON_DLL_TEMPLATE)

// This stuff is here because of a limitation in the way Microsoft
// handles templates and DLLs.  See Microsoft's knowledge base 
// article ID Q168958 for details.
#pragma warning( push )
#pragma warning( disable : 4231 )
ON_DLL_TEMPLATE template class ON_CLASS ON_ClassArray<ON_PlugInRef>;
#pragma warning( pop )
#endif

#endif

