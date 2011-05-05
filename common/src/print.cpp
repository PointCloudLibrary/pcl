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
#if defined WIN32 && defined _MSC_VER
# include <windows.h>

DWORD convertAttributesColor(int attribute, int fg, int bg=-1)
{
    static DWORD wAttributes[7]  = { 0, //TT_RESET
                                     FOREGROUND_INTENSITY , // TT_BRIGHT
                                     0, // TT_DIM
                                     COMMON_LVB_UNDERSCORE, // TT_UNDERLINE
                                     0, // TT_BLINK     = 4,
                                     COMMON_LVB_REVERSE_VIDEO, // TT_REVERSE
                                     0 //TT_HIDDEN    = 8
                                    };
    static DWORD wFgColors[8]  = { 0, // TT_BLACK
                                   FOREGROUND_RED, // TT_RED
                                   FOREGROUND_GREEN , // TT_GREEN
                                   FOREGROUND_GREEN | FOREGROUND_RED , // TT_YELLOW
                                   FOREGROUND_BLUE , // TT_BLUE
                                   FOREGROUND_RED | FOREGROUND_BLUE , // TT_MAGENTA
                                   FOREGROUND_GREEN | FOREGROUND_BLUE, // TT_CYAN
                                   FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED // TT_WHITE
                                 };
    static DWORD wBgColors[8]  = { 0, // TT_BLACK
                                   BACKGROUND_RED, // TT_RED
                                   BACKGROUND_GREEN , // TT_GREEN
                                   BACKGROUND_GREEN | BACKGROUND_BLUE , // TT_YELLOW
                                   BACKGROUND_BLUE , // TT_BLUE
                                   BACKGROUND_RED | BACKGROUND_BLUE , // TT_MAGENTA
                                   BACKGROUND_GREEN | BACKGROUND_BLUE, // TT_CYAN
                                   BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_RED // TT_WHITE
                                 };

    if(bg < 0)
        return wAttributes[attribute] | wFgColors[fg];
    else
        return wAttributes[attribute] | wFgColors[fg] | wBgColors[bg];

}

#endif

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::change_text_color (FILE *stream, int attribute, int fg, int bg)
{
#ifdef WIN32
  HANDLE h = GetStdHandle((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
  SetConsoleTextAttribute(h, convertAttributesColor(attribute, fg, bg));
#else
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
  fprintf (stream, "%s", command);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::change_text_color (FILE *stream, int attribute, int fg)
{
#ifdef WIN32
  HANDLE h = GetStdHandle((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
  SetConsoleTextAttribute(h, convertAttributesColor(attribute, fg));
#else
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[%d;%dm", 0x1B, attribute, fg + 30);
  fprintf (stream, "%s", command);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::reset_text_color (FILE *stream)
{
#ifdef WIN32
  HANDLE h = GetStdHandle((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
  SetConsoleTextAttribute(h, convertAttributesColor(0, TT_WHITE, TT_BLACK));
#else
  char command[13];
  // Command is the control command to the terminal
  sprintf (command, "%c[0;m", 0x1B);
  fprintf (stream, "%s", command);
#endif
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
  change_text_color (stderr, TT_BRIGHT, TT_RED);
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
  change_text_color (stream, TT_BRIGHT, TT_RED);
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
  change_text_color (stderr, TT_BRIGHT, TT_YELLOW);
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
  change_text_color (stream, TT_BRIGHT, TT_YELLOW);
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

