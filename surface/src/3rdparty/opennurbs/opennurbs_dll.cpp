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

#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

// opennurbs_dll.cpp : Defines the entry point for the Windows DLL application.
//

#if defined(ON_OS_WINDOWS) && defined(ON_DLL_EXPORTS)

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
  static int bRunning = 0;
  if ( !bRunning ) 
  {
    bRunning = true;
  }

  switch( ul_reason_for_call ) {

  case DLL_PROCESS_ATTACH:
    ON_ClassId::IncrementMark(); // make sure each DLL that each process that 
                                 // uses OpenNURBS has a unique mark.
    break;

  // 16 August 2010 Dale Lear:
  //   These generate too much noise in the output window
  //
  //case DLL_THREAD_ATTACH:
  //  ::OutputDebugStringA("OpenNURBS DllMain() ul_reason_for_call = DLL_THREAD_ATTACH\n");
  //  break;

  //case DLL_THREAD_DETACH:
  //  ::OutputDebugStringA("OpenNURBS DllMain() ul_reason_for_call = DLL_THREAD_DETACH\n");
  //  break;

  //case DLL_PROCESS_DETACH:
  //  ::OutputDebugStringA("OpenNURBS DllMain() ul_reason_for_call = DLL_PROCESS_DETACH\n");
  //  break;

  //default:
  //  ::OutputDebugStringA("OpenNURBS DllMain() ul_reason_for_call = ?\n");
  //  break;
  }

  return true;
}

#endif
