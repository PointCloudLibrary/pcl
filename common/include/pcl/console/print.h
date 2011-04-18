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
 * $Id$
 *
 */
#ifndef TERMINAL_TOOLS_PRINT_H_
#define TERMINAL_TOOLS_PRINT_H_

#include <stdio.h>
#include <stdarg.h>

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

    /** \brief Change the text color (on either stdout or stderr) with an attr:fg:bg
      * \param stream the output stream (stdout, stderr, etc)
      * \param attribute the text attribute
      * \param fg the foreground color
      * \param bg the background color
      */
    void 
    change_text_color (FILE *stream, int attribute, int fg, int bg);
    
    /** \brief Change the text color (on either stdout or stderr) with an attr:fg
      * \param stream the output stream (stdout, stderr, etc)
      * \param attribute the text attribute
      * \param fg the foreground color
      */
    void 
    change_text_color (FILE *stream, int attribute, int fg);

    /** \brief Reset the text color (on either stdout or stderr) to its original state
      * \param stream the output stream (stdout, stderr, etc)
      */
    void 
    reset_text_color (FILE *stream);

    /** \brief Print a message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param attr the text attribute
      * \param fg the foreground color
      * \param format the message
      */
    void 
    print_color (FILE *stream, int attr, int fg, const char *format, ...);

    /** \brief Print an info message on stream with colors
      * \param format the message
      */
    void 
    print_info  (const char *format, ...);

    /** \brief Print an info message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    void 
    print_info  (FILE *stream, const char *format, ...);

    /** \brief Print a highlighted info message on stream with colors
      * \param format the message
      */
    void 
    print_highlight  (const char *format, ...);

    /** \brief Print a highlighted info message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    void 
    print_highlight  (FILE *stream, const char *format, ...);

    /** \brief Print an error message on stream with colors
      * \param format the message
      */
    void 
    print_error (const char *format, ...);

    /** \brief Print an error message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    void 
    print_error (FILE *stream, const char *format, ...);

    /** \brief Print a warning message on stream with colors
      * \param format the message
      */
    void 
    print_warn (const char *format, ...);

    /** \brief Print a warning message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    void 
    print_warn (FILE *stream, const char *format, ...);

    /** \brief Print a debug message on stream with colors
      * \param format the message
      */
    void 
    print_debug (const char *format, ...);

    /** \brief Print a debug message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    void 
    print_debug (FILE *stream, const char *format, ...);


    /** \brief Print a value message on stream with colors
      * \param format the message
      */
    void 
    print_value (const char *format, ...);

    /** \brief Print a value message on stream with colors
      * \param stream the output stream (stdout, stderr, etc)
      * \param format the message
      */
    void 
    print_value (FILE *stream, const char *format, ...);
  }
} 

#endif
