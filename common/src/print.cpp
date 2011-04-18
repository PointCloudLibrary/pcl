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
#include <pcl/console/print.h>

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::change_text_color (FILE *stream, int attribute, int fg, int bg)
{
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
  fprintf (stream, "%s", command);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::change_text_color (FILE *stream, int attribute, int fg)
{
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
  fprintf (stream, "%s", command);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::reset_text_color (FILE *stream)
{
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[0;m", 0x1B);
  fprintf (stream, "%s", command);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_color (FILE *stream, int attr, int fg, const char *format, ...)
{
  change_text_color (stream, attr, fg);
  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
  
  reset_text_color (stream);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_info (const char *format, ...)
{
  reset_text_color (stdout);

  va_list ap;

  va_start (ap, format);
  vfprintf (stdout, format, ap);
  va_end (ap);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_info (FILE *stream, const char *format, ...)
{
  reset_text_color (stream);

  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_highlight (const char *format, ...)
{
  change_text_color (stdout, TT_BRIGHT, TT_GREEN);
  fprintf (stdout, "> ");
  reset_text_color (stdout);

  va_list ap;

  va_start (ap, format);
  vfprintf (stdout, format, ap);
  va_end (ap);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_highlight (FILE *stream, const char *format, ...)
{
  change_text_color (stream, TT_BRIGHT, TT_GREEN);
  fprintf (stream, "> ");
  reset_text_color (stream);

  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_error (const char *format, ...)
{
  change_text_color (stderr, TT_RESET, TT_RED);
  va_list ap;

  va_start (ap, format);
  vfprintf (stderr, format, ap);
  va_end (ap);
  
  reset_text_color (stderr);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_error (FILE *stream, const char *format, ...)
{
  change_text_color (stream, TT_RESET, TT_RED);
  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
  
  reset_text_color (stream);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_warn (const char *format, ...)
{
  change_text_color (stderr, TT_RESET, TT_YELLOW);
  va_list ap;

  va_start (ap, format);
  vfprintf (stderr, format, ap);
  va_end (ap);
  
  reset_text_color (stderr);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_warn (FILE *stream, const char *format, ...)
{
  change_text_color (stream, TT_RESET, TT_YELLOW);
  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
  
  reset_text_color (stream);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_value (const char *format, ...)
{
  change_text_color (stdout, TT_RESET, TT_CYAN);
  va_list ap;

  va_start (ap, format);
  vfprintf (stdout, format, ap);
  va_end (ap);
  
  reset_text_color (stdout);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_value (FILE *stream, const char *format, ...)
{
  change_text_color (stream, TT_RESET, TT_CYAN);
  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
  
  reset_text_color (stream);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_debug (const char *format, ...)
{
  change_text_color (stdout, TT_RESET, TT_GREEN);
  va_list ap;

  va_start (ap, format);
  vfprintf (stdout, format, ap);
  va_end (ap);
  
  reset_text_color (stdout);
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::print_debug (FILE *stream, const char *format, ...)
{
  change_text_color (stream, TT_RESET, TT_GREEN);
  va_list ap;

  va_start (ap, format);
  vfprintf (stream, format, ap);
  va_end (ap);
  
  reset_text_color (stream);
}

