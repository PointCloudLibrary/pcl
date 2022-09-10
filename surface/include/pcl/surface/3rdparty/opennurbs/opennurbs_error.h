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

#if !defined(OPENNURBS_ERROR_INC_)
#define OPENNURBS_ERROR_INC_

/*
// Macros used to log errors and warnings.  The ON_Warning() and ON_Error()
// functions are defined in opennurbs_error.cpp.
*/

#if defined(__FUNCTION__)
// __FUNCTION__ macro exists
#define ON_ERROR(msg) ON_ErrorEx(__FILE__,__LINE__,__FUNCTION__,msg)
#define ON_WARNING(msg) ON_WarningEx(__FILE__,__LINE__,__FUNCTION__,msg)
#define ON_ASSERT(cond) ON_AssertEx(cond,__FILE__,__LINE__,__FUNCTION__, #cond " is false")
#define ON_ASSERT_OR_RETURN(cond,returncode) do{if (!(cond)) {ON_AssertEx(false,__FILE__,__LINE__,__FUNCTION__, #cond " is false");return(returncode);}}while(0)
#else
// __FUNCTION__ macro does not exist
#define ON_ERROR(msg) ON_Error(__FILE__,__LINE__,msg)
#define ON_WARNING(msg) ON_Warning(__FILE__,__LINE__,msg)
#define ON_ASSERT(cond) ON_Assert(cond,__FILE__,__LINE__, #cond " is false")
#define ON_ASSERT_OR_RETURN(cond,returncode) do{if (!(cond)) {ON_Assert(false,__FILE__,__LINE__, #cond " is false");return(returncode);}}while(0)
#endif


ON_BEGIN_EXTERNC

/*
// All error/warning messages are sent to ON_ErrorMessage().  Replace the
// default handler (defined in opennurbs_error_message.cpp) with something
// that is appropriate for debugging your application.
*/
ON_DECL
void ON_ErrorMessage( 
       int,         /* 0 = warning message, 1 = serious error message, 2 = assert failure */
       const char*  
       ); 

/*
Returns:
  Number of opennurbs errors since program started.
*/
ON_DECL
int     ON_GetErrorCount(void);

/*
Returns:
  Number of opennurbs warnings since program started.
*/
ON_DECL
int     ON_GetWarningCount(void);

/*
Returns:
  Number of math library or floating point errors that have 
  been handled since program started.
*/
ON_DECL
int     ON_GetMathErrorCount(void);

ON_DECL
int     ON_GetDebugErrorMessage(void);

ON_DECL
void    ON_EnableDebugErrorMessage( int bEnableDebugErrorMessage );


ON_DECL
void    ON_Error( const char*, /* sFileName:   __FILE__ will do fine */
                  int,         /* line number: __LINE__ will do fine */
                  const char*, /* printf() style format string */
                  ...          /* printf() style ags */
                  );

PCL_EXPORTS ON_DECL
void    ON_ErrorEx( const char*, // sFileName:   __FILE__ will do fine
                  int,           // line number: __LINE__ will do fine
                  const char*,   // sFunctionName: __FUNCTION__ will do fine
                  const char*,   // printf() style format string
                  ...            // printf() style ags
                  );
ON_DECL
void    ON_Warning( const char*, /* sFileName:   __FILE__ will do fine */
                    int,         /* line number: __LINE__ will do fine */
                    const char*, /* printf() style format string */
                    ...          /* printf() style ags */
                  );
ON_DECL
void    ON_WarningEx( const char*, // sFileName:   __FILE__ will do fine
                  int,           // line number: __LINE__ will do fine
                  const char*,   // sFunctionName: __FUNCTION__ will do fine
                  const char*,   // printf() style format string
                  ...            // printf() style ags
                  );

// Ideally - these "assert" functions will be deleted when the SDK can be changed.
ON_DECL
void    ON_Assert( int,         /* if false, error is flagged */
                   const char*, /* sFileName:   __FILE__ will do fine */
                   int,         /* line number: __LINE__ will do fine */
                   const char*, /* printf() style format string */
                   ...          /* printf() style ags */
                  );

ON_DECL
void    ON_AssertEx( int,        // if false, error is flagged
                  const char*,   // sFileName:   __FILE__ will do fine
                  int,           // line number: __LINE__ will do fine
                  const char*,   // sFunctionName: __FUNCTION__ will do fine
                  const char*,   // printf() style format string
                  ...            // printf() style ags
                  );

ON_DECL
void    ON_MathError( 
        const char*, /* sModuleName */
        const char*, /* sErrorType */
        const char*  /* sFunctionName */
        );

ON_END_EXTERNC

#endif
