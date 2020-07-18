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

// openNURBS Geometry Library Errors and Warnings
//
//   If an error condition occurs during a openNURBS Geometry Library
//   computation, the ON_Error() function is called, the computation is
//   stopped, and an error code (negative integer ) is returned.  If a
//   warning condition occurs during a Trout Lake Geometry Library 
//   computation, the ON_Warning() function is called and the computation
//   continues.
//
//   ON_GetErrorCount()
//   ON_GetWarningCount()
//   ON_Error()
//   ON_Warning()
//

static int ON_ERROR_COUNT = 0;
static int ON_WARNING_COUNT = 0;
static int ON_MATH_ERROR_COUNT = 0;
static int ON_DEBUG_ERROR_MESSAGE_OPTION = 0; 

int ON_GetErrorCount(void)
{
  return ON_ERROR_COUNT;	
}	


int ON_GetWarningCount(void)
{
  return ON_WARNING_COUNT;	
}	

int ON_GetMathErrorCount(void)
{
  return ON_MATH_ERROR_COUNT;	
}	

int ON_GetDebugErrorMessage(void)
{
  return ON_DEBUG_ERROR_MESSAGE_OPTION?true:false;
}


void ON_EnableDebugErrorMessage( int bEnableDebugErrorMessage )
{
  ON_DEBUG_ERROR_MESSAGE_OPTION = bEnableDebugErrorMessage ? 1 : 0;
}

// The sMessage[] string is used by ON_Error() and ON_Warning() 
// to hold the message.  The static function ON_FormatMessage() 
// is used to do most of the actual formatting.  

#define MAX_MSG_LENGTH 2048
static char sMessage[MAX_MSG_LENGTH];
static bool ON_FormatMessage(const char*, va_list );

void ON_MathError( 
        const char* sModuleName,
        const char* sErrorType,
        const char* sFunctionName
        )
{
  ON_MATH_ERROR_COUNT++; // <- Good location for a debugger breakpoint.

  if ( !sModuleName)
    sModuleName = "";
  if ( !sErrorType )
    sErrorType = "";
  if ( !sFunctionName )
    sFunctionName = "";

  ON_Error(__FILE__,__LINE__,
           "Math library or floating point ERROR # %d module=%s type=%s function=%s",
           ON_MATH_ERROR_COUNT, 
           sModuleName, // rhino.exe, opennurbs.dll, etc.
           sErrorType,   
           sFunctionName 
           );
}	

static void ON_IncrementErrorCount()
{
  ON_ERROR_COUNT++;
}

static void ON_IncrementWarningCount()
{
  ON_WARNING_COUNT++;
}

bool ON_IsNotValid()
{
  return false;
}

static bool ON_PrintErrorHeader(
    int type, // 0 = warning, 1 = error, 2 = assert
    const char* sFileName, 
    int line_number,
    const char* sFunctionName
    )
{
  bool bPrintMessage = false;
  sMessage[0] = 0;

#if defined(ON_COMPILER_MSC)
  // use sprintf_s() ...
  std::size_t sz = (sizeof(sMessage)/sizeof(sMessage[0])) - 1;
#define ON_SPRINTF4(s,count,fname,ln,func) sprintf_s(sMessage,sz,s,count,fname,ln,func)
#define ON_SPRINTF3(s,count,fname,ln) sprintf_s(sMessage,sz,s,count,fname,ln)
#define ON_SPRINTF1(s,count) sprintf_s(sMessage,sz,s,count)
#else
  // use sprintf() ...
#define ON_SPRINTF4(s,count,fname,ln,func) sprintf(sMessage,s,count,fname,ln,func)
#define ON_SPRINTF3(s,count,fname,ln) sprintf(sMessage,s,count,fname,ln)
#define ON_SPRINTF1(s,count) sprintf(sMessage,s,count)
#endif

  if ( ON_DEBUG_ERROR_MESSAGE_OPTION )
  {
    if ( 0 == type )
    {
      if ( ON_WARNING_COUNT < 50 )
      {
        if (0 == sFileName )
          sFileName = "";
        if ( sFunctionName && sFunctionName[0] )
          ON_SPRINTF4("openNURBS WARNING # %d %s.%d %s(): ",ON_WARNING_COUNT,sFileName,line_number,sFunctionName);
        else
          ON_SPRINTF3("openNURBS WARNING # %d %s.%d ",ON_WARNING_COUNT,sFileName,line_number);
        bPrintMessage = true;
      }
      else if ( 50 == ON_ERROR_COUNT )
      {
        ON_SPRINTF1("openNURBS WARNING # %d - Too many warnings.  No more printed messages.",ON_WARNING_COUNT);
        bPrintMessage = true;
      }
    }
    else if ( 1 == type || 2 == type )
    {
      if ( ON_ERROR_COUNT < 50 )
      {
        if (0 == sFileName )
          sFileName = "";
        if ( sFunctionName && sFunctionName[0] )
          ON_SPRINTF4("openNURBS ERROR # %d %s.%d %s(): ",ON_ERROR_COUNT,sFileName,line_number,sFunctionName);
        else
          ON_SPRINTF3("openNURBS ERROR # %d %s.%d ",ON_ERROR_COUNT,sFileName,line_number);
        bPrintMessage = true;
      }
      else if ( 50 == ON_ERROR_COUNT )
      {
        ON_SPRINTF1("openNURBS ERROR # %d - Too many errors.  No more printed messages.",ON_ERROR_COUNT);
        bPrintMessage = true;
      }
    }
  }

#undef ON_SPRINTF4
#undef ON_SPRINTF3
#undef ON_SPRINTF1

  return bPrintMessage;
}

void ON_Error(const char* sFileName, int line_number, 
              const char* sFormat, ...)
{
  ON_IncrementErrorCount();

  bool bPrintMessage = ON_PrintErrorHeader(1,sFileName,line_number,0);

  if ( bPrintMessage )
  {
    if (sFormat && sFormat[0]) 
    {
      // append formatted error message to sMessage[]
      va_list args;
      va_start(args, sFormat);
      bPrintMessage = ON_FormatMessage(sFormat,args);
      va_end(args);
    }
    if ( bPrintMessage )
      ON_ErrorMessage(1,sMessage);
  }
}

void ON_ErrorEx(const char* sFileName, int line_number, const char* sFunctionName, 
              const char* sFormat, ...)
{
  ON_IncrementErrorCount();

  bool bPrintMessage = ON_PrintErrorHeader(1,sFileName,line_number,sFunctionName);

  if ( bPrintMessage )
  {
    if (sFormat && sFormat[0]) 
    {
      // append formatted error message to sMessage[]
      va_list args;
      va_start(args, sFormat);
      bPrintMessage = ON_FormatMessage(sFormat,args);
      va_end(args);
    }
    if ( bPrintMessage )
      ON_ErrorMessage(1,sMessage);
  }
}

void ON_Warning(const char* sFileName, int line_number, 
                const char* sFormat, ...)
{
  ON_IncrementWarningCount();

  bool bPrintMessage = ON_PrintErrorHeader(0,sFileName,line_number,0);

  if ( bPrintMessage )
  {
    if (sFormat && sFormat[0]) 
    {
      // append formatted error message to sMessage[]
      va_list args;
      va_start(args, sFormat);
      bPrintMessage = ON_FormatMessage(sFormat,args);
      va_end(args);
    }
    if ( bPrintMessage )
      ON_ErrorMessage(0,sMessage);
  }
}


void ON_WarningEx(const char* sFileName, int line_number, const char* sFunctionName,
                const char* sFormat, ...)
{
  ON_IncrementWarningCount();

  bool bPrintMessage = ON_PrintErrorHeader(0,sFileName,line_number,sFunctionName);

  if ( bPrintMessage )
  {
    if (sFormat && sFormat[0]) 
    {
      // append formatted error message to sMessage[]
      va_list args;
      va_start(args, sFormat);
      bPrintMessage = ON_FormatMessage(sFormat,args);
      va_end(args);
    }
    if ( bPrintMessage )
      ON_ErrorMessage(0,sMessage);
  }
}

void ON_Assert(int bCondition,
               const char* sFileName, int line_number, 
               const char* sFormat, ...)
{
  if ( !bCondition ) 
  {
    ON_IncrementErrorCount();

    bool bPrintMessage = ON_PrintErrorHeader(2,sFileName,line_number,0);

    if ( bPrintMessage )
    {
      if (sFormat && sFormat[0]) 
      {
        // append formatted error message to sMessage[]
        va_list args;
        va_start(args, sFormat);
        bPrintMessage = ON_FormatMessage(sFormat,args);
        va_end(args);
      }
      if ( bPrintMessage )
        ON_ErrorMessage(2,sMessage);
    }
  }
}

void ON_AssertEx(int bCondition,
               const char* sFileName, int line_number, const char* sFunctionName,
               const char* sFormat, ...)
{
  if ( !bCondition ) 
  {
    ON_IncrementErrorCount();

    bool bPrintMessage = ON_PrintErrorHeader(2,sFileName,line_number,sFunctionName);

    if ( bPrintMessage )
    {
      if (sFormat && sFormat[0]) 
      {
        // append formatted error message to sMessage[]
        va_list args;
        va_start(args, sFormat);
        bPrintMessage = ON_FormatMessage(sFormat,args);
        va_end(args);
      }
      if ( bPrintMessage )
        ON_ErrorMessage(2,sMessage);
    }
  }
}

static bool ON_FormatMessage(const char* format, va_list args)
{
  // appends formatted message to sMessage[]
  int len = ((int)strlen(sMessage));
  if (len < 0 )
    return false;
  if (MAX_MSG_LENGTH-1-len < 2)
    return false;
  sMessage[MAX_MSG_LENGTH-1] = 0;
  on_vsnprintf(sMessage+len, MAX_MSG_LENGTH-1-len, format, args);
  return true;
}	
