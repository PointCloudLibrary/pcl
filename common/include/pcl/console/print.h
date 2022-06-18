/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <cstdio>

#include <pcl/pcl_exports.h>
#include <pcl/pcl_config.h>

// Use e.g. like this:
// PCL_INFO_STREAM("Info: this is a point: " << pcl::PointXYZ(1.0, 2.0, 3.0) << std::endl);
// PCL_ERROR_STREAM("Error: an Eigen vector: " << std::endl << Eigen::Vector3f(1.0, 2.0, 3.0) << std::endl);
#define PCL_LOG_STREAM(LEVEL, STREAM, CSTR, ATTR, FG, ARGS) if(pcl::console::isVerbosityLevelEnabled(pcl::console::LEVEL)) { fflush(stdout); pcl::console::change_text_color(CSTR, pcl::console::ATTR, pcl::console::FG); STREAM << ARGS; pcl::console::reset_text_color(CSTR); }
#define PCL_ALWAYS_STREAM(ARGS)  PCL_LOG_STREAM(L_ALWAYS,  std::cout, stdout, TT_RESET,  TT_WHITE,  ARGS)
#define PCL_ERROR_STREAM(ARGS)   PCL_LOG_STREAM(L_ERROR,   std::cerr, stderr, TT_BRIGHT, TT_RED,    ARGS)
#define PCL_WARN_STREAM(ARGS)    PCL_LOG_STREAM(L_WARN,    std::cerr, stderr, TT_BRIGHT, TT_YELLOW, ARGS)
#define PCL_INFO_STREAM(ARGS)    PCL_LOG_STREAM(L_INFO,    std::cout, stdout, TT_RESET,  TT_WHITE,  ARGS)
#define PCL_DEBUG_STREAM(ARGS)   PCL_LOG_STREAM(L_DEBUG,   std::cout, stdout, TT_RESET,  TT_GREEN,  ARGS)
#define PCL_VERBOSE_STREAM(ARGS) PCL_LOG_STREAM(L_VERBOSE, std::cout, stdout, TT_RESET,  TT_WHITE,  ARGS)


#define PCL_ALWAYS(...)  pcl::console::print (pcl::console::L_ALWAYS, __VA_ARGS__)
#define PCL_ERROR(...)   pcl::console::print (pcl::console::L_ERROR, __VA_ARGS__)
#define PCL_WARN(...)    pcl::console::print (pcl::console::L_WARN, __VA_ARGS__)
#define PCL_INFO(...)    pcl::console::print (pcl::console::L_INFO, __VA_ARGS__)
#define PCL_DEBUG(...)   pcl::console::print (pcl::console::L_DEBUG, __VA_ARGS__)
#define PCL_VERBOSE(...) pcl::console::print (pcl::console::L_VERBOSE, __VA_ARGS__)

#define PCL_ASSERT_ERROR_PRINT_CHECK(pred, msg) \
    do \
    { \
        if (!(pred)) \
        { \
            PCL_ERROR(msg); \
            PCL_ERROR("In File %s, in line %d\n" __FILE__, __LINE__); \
        } \
    } while (0)

#define PCL_ASSERT_ERROR_PRINT_RETURN(pred, msg, err) \
    do \
    { \
        PCL_ASSERT_ERROR_PRINT_CHECK(pred, msg); \
        if (!(pred)) return err; \
    } while (0)

namespace pcl
{
  namespace console
  {
    enum TT_ATTIBUTES
    {
      TT_RESET     = 0,
      TT_BRIGHT    = 1,
      TT_DIM       = 2,
      TT_UNDERLINE = 3,
      TT_BLINK     = 4,
      TT_REVERSE   = 7,
      TT_HIDDEN    = 8
    };

    enum TT_COLORS
    {
      TT_BLACK,
      TT_RED,
      TT_GREEN,
      TT_YELLOW,
      TT_BLUE,
      TT_MAGENTA,
      TT_CYAN,
      TT_WHITE
    };

    enum VERBOSITY_LEVEL
    {
      L_ALWAYS,
      L_ERROR,
      L_WARN,
      L_INFO,
      L_DEBUG,
      L_VERBOSE
    };

    /** set the verbosity level */
    PCL_EXPORTS void 
    setVerbosityLevel (VERBOSITY_LEVEL level);

    /** get the verbosity level. */
    PCL_EXPORTS VERBOSITY_LEVEL 
    getVerbosityLevel ();

    /** initialize verbosity level. */
    PCL_EXPORTS bool 
    initVerbosityLevel ();

    /** is verbosity level enabled? */
    PCL_EXPORTS bool 
    isVerbosityLevelEnabled (VERBOSITY_LEVEL severity);

    /** \brief Enable or disable colored text output, overriding the default behavior.
      *
      * By default, colored output is enabled for interactive terminals or when the environment
      * variable PCL_CLICOLOR_FORCE is set.
      *
      * \param stream the output stream (stdout, stderr, etc)
      * \param enable whether to emit color codes when calling any of the color related methods
      */
    PCL_EXPORTS void
    enableColoredOutput (FILE *stream, bool enable);

    /** \brief Change the text color (on either stdout or stderr) with an attr:fg:bg
      * \param stream the output stream (stdout, stderr, etc)
      * \param attribute the text attribute
      * \param fg the foreground color
      * \param bg the background color
      */
    PCL_EXPORTS void 
    change_text_color (FILE *stream, int attribute, int fg, int bg);
    
    /** \brief Change the text color (on either stdout or stderr) with an attr:fg
      * \param stream the output stream (stdout, stderr, etc)
      * \param attribute the text attribute
      * \param fg the foreground color
      */
    PCL_EXPORTS void 
    change_text_color (FILE *stream, int attribute, int fg);

    /** \brief Reset the text color (on either stdout or stderr) to its original state
      * \param stream the output stream (stdout, stderr, etc)
      */
    PCL_EXPORTS void 
    reset_text_color (FILE *stream);

    /** \brief Print a message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param attr the text attribute
      * \param fg the foreground color
      * \param format the message
      */
    PCL_EXPORTS void 
    print_color (FILE *stream, int attr, int fg, const char *format, ...);

    /** \brief Print an info message on stream with colors
      * \param format the message
      */
    PCL_EXPORTS void 
    print_info  (const char *format, ...);

    /** \brief Print an info message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print_info  (FILE *stream, const char *format, ...);

    /** \brief Print a highlighted info message on stream with colors
      * \param format the message
      */
    PCL_EXPORTS void 
    print_highlight  (const char *format, ...);

    /** \brief Print a highlighted info message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print_highlight  (FILE *stream, const char *format, ...);

    /** \brief Print an error message on stream with colors
      * \param format the message
      */
    PCL_EXPORTS void 
    print_error (const char *format, ...);

    /** \brief Print an error message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print_error (FILE *stream, const char *format, ...);

    /** \brief Print a warning message on stream with colors
      * \param format the message
      */
    PCL_EXPORTS void 
    print_warn (const char *format, ...);

    /** \brief Print a warning message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print_warn (FILE *stream, const char *format, ...);

    /** \brief Print a debug message on stream with colors
      * \param format the message
      */
    PCL_EXPORTS void 
    print_debug (const char *format, ...);

    /** \brief Print a debug message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print_debug (FILE *stream, const char *format, ...);


    /** \brief Print a value message on stream with colors
      * \param format the message
      */
    PCL_EXPORTS void 
    print_value (const char *format, ...);

    /** \brief Print a value message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print_value (FILE *stream, const char *format, ...);

    /** \brief Print a message on stream
      * \param level the verbosity level
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    PCL_EXPORTS void 
    print (VERBOSITY_LEVEL level, FILE *stream, const char *format, ...);

    /** \brief Print a message
      * \param level the verbosity level
      * \param format the message
      */
    PCL_EXPORTS void 
    print (VERBOSITY_LEVEL level, const char *format, ...);
  }
} 
