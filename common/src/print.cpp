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
#include <pcl/console/print.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cctype> // for toupper
#include <map> // For std::map
#include <string>
#include <boost/optional.hpp>

#if defined _WIN32
# include <windows.h>
# include <io.h>

#ifndef _MSC_VER
# define COMMON_LVB_UNDERSCORE    0
# define COMMON_LVB_REVERSE_VIDEO 0
#endif

WORD
convertAttributesColor (int attribute, int fg, int bg=0)
{
  static WORD wAttributes[7]  = { 0,                        // TT_RESET
                                   FOREGROUND_INTENSITY ,    // TT_BRIGHT
                                   0,                        // TT_DIM
                                   COMMON_LVB_UNDERSCORE,    // TT_UNDERLINE
                                   0,                        // TT_BLINK
                                   COMMON_LVB_REVERSE_VIDEO, // TT_REVERSE
                                   0                         // TT_HIDDEN
                                 };
  static WORD wFgColors[8]  = { 0,                                                  // TT_BLACK
                                 FOREGROUND_RED,                                     // TT_RED
                                 FOREGROUND_GREEN ,                                  // TT_GREEN
                                 FOREGROUND_GREEN | FOREGROUND_RED ,                 // TT_YELLOW
                                 FOREGROUND_BLUE ,                                   // TT_BLUE
                                 FOREGROUND_RED | FOREGROUND_BLUE ,                  // TT_MAGENTA
                                 FOREGROUND_GREEN | FOREGROUND_BLUE,                 // TT_CYAN
                                 FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED // TT_WHITE
                               };
  static WORD wBgColors[8]  = { 0,                                                  // TT_BLACK
                                 BACKGROUND_RED,                                     // TT_RED
                                 BACKGROUND_GREEN ,                                  // TT_GREEN
                                 BACKGROUND_GREEN | BACKGROUND_BLUE ,                // TT_YELLOW
                                 BACKGROUND_BLUE ,                                   // TT_BLUE
                                 BACKGROUND_RED | BACKGROUND_BLUE ,                  // TT_MAGENTA
                                 BACKGROUND_GREEN | BACKGROUND_BLUE,                 // TT_CYAN
                                 BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_RED // TT_WHITE
                               };

  return wAttributes[attribute] | wFgColors[fg] | wBgColors[bg];
}

#else
#  include <unistd.h>
#endif

// Map to store, for each output stream, whether to use colored output
static std::map<FILE *, boost::optional<bool> > colored_output;

////////////////////////////////////////////////////////////////////////////////
inline bool
useColoredOutput (FILE *stream)
{
  auto &colored = colored_output[stream];
  if (!colored)
  {
    // Use colored output if PCL_CLICOLOR_FORCE is set or if the output is an interactive terminal
#ifdef _WIN32
    colored = getenv ("PCL_CLICOLOR_FORCE") || _isatty (_fileno (stream));
#else
    colored = getenv ("PCL_CLICOLOR_FORCE") || isatty (fileno (stream));
#endif
  }
  return colored.get ();
}

pcl::console::Logger&
pcl::console::Logger::getInstance()
{
  static Logger instance;
  return instance;
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::enableColoredOutput (FILE *stream, bool enable)
{
  colored_output[stream] = enable;
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::change_text_color (FILE *stream, int attribute, int fg, int bg)
{
  if (!useColoredOutput (stream)) return;

#ifdef _WIN32
  HANDLE h = GetStdHandle ((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
  SetConsoleTextAttribute (h, convertAttributesColor (attribute, fg, bg));
#else
  char command[40];
  // Command is the control command to the terminal
  snprintf (command, sizeof(command), "%c[%d;%d;%dm", 0x1B, attribute, fg + 30, bg + 40);
  fprintf (stream, "%s", command);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::change_text_color (FILE *stream, int attribute, int fg)
{
  if (!useColoredOutput (stream)) return;

#ifdef _WIN32
  HANDLE h = GetStdHandle ((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
  SetConsoleTextAttribute (h, convertAttributesColor (attribute, fg));
#else
  char command[17];
  // Command is the control command to the terminal
  snprintf (command, sizeof(command), "%c[%d;%dm", 0x1B, attribute, fg + 30);
  fprintf (stream, "%s", command);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void
pcl::console::reset_text_color (FILE *stream)
{
  if (!useColoredOutput (stream)) return;

#ifdef _WIN32
  HANDLE h = GetStdHandle ((stream == stdout) ? STD_OUTPUT_HANDLE : STD_ERROR_HANDLE);
  SetConsoleTextAttribute (h, convertAttributesColor (0, TT_WHITE, TT_BLACK));
#else
  char command[13];
  // Command is the control command to the terminal
  snprintf (command, sizeof(command), "%c[0;m", 0x1B);
  fprintf (stream, "%s", command);
#endif
}

////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  namespace console
  {
    static bool s_NeedVerbosityInit = true;
#ifdef VERBOSITY_LEVEL_ALWAYS
  static VERBOSITY_LEVEL s_VerbosityLevel = pcl::console::L_ALWAYS;
#elif defined VERBOSITY_LEVEL_ERROR
  static VERBOSITY_LEVEL s_VerbosityLevel = pcl::console::L_ERROR;
#elif defined VERBOSITY_LEVEL_WARN
  static VERBOSITY_LEVEL s_VerbosityLevel = pcl::console::L_WARN;
#elif defined VERBOSITY_LEVEL_DEBUG
  static VERBOSITY_LEVEL s_VerbosityLevel = pcl::console::L_DEBUG;
#elif defined VERBOSITY_LEVEL_VERBOSE
  static VERBOSITY_LEVEL s_VerbosityLevel = pcl::console::L_VERBOSE;
#else
  static VERBOSITY_LEVEL s_VerbosityLevel = pcl::console::L_INFO;
#endif
  }
}

////////////////////////////////////////////////////////////////////////////////
void pcl::console::setVerbosityLevel (pcl::console::VERBOSITY_LEVEL level)
{
  if (s_NeedVerbosityInit) pcl::console::initVerbosityLevel ();
  s_VerbosityLevel = level;
}

////////////////////////////////////////////////////////////////////////////////
pcl::console::VERBOSITY_LEVEL
pcl::console::getVerbosityLevel ()
{
  if (s_NeedVerbosityInit) pcl::console::initVerbosityLevel ();
  return s_VerbosityLevel;
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::isVerbosityLevelEnabled (pcl::console::VERBOSITY_LEVEL level)
{
  if (s_NeedVerbosityInit) pcl::console::initVerbosityLevel ();
  return level <= s_VerbosityLevel;
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::initVerbosityLevel ()
{
#ifdef VERBOSITY_LEVEL_ALWAYS
  s_VerbosityLevel = pcl::console::L_ALWAYS;
#elif defined VERBOSITY_LEVEL_ERROR
  s_VerbosityLevel = pcl::console::L_ERROR;
#elif defined VERBOSITY_LEVEL_WARN
  s_VerbosityLevel = pcl::console::L_WARN;
#elif defined VERBOSITY_LEVEL_DEBUG
  s_VerbosityLevel = pcl::console::L_DEBUG;
#elif defined VERBOSITY_LEVEL_VERBOSE
  s_VerbosityLevel = pcl::console::L_VERBOSE;
#else
  s_VerbosityLevel = pcl::console::L_INFO; // Default value
#endif

  char* pcl_verbosity_level = getenv ( "PCL_VERBOSITY_LEVEL");
  if (pcl_verbosity_level)
  {
    std::string s_pcl_verbosity_level (pcl_verbosity_level);
    std::transform (s_pcl_verbosity_level.begin (), s_pcl_verbosity_level.end (), s_pcl_verbosity_level.begin (), toupper);

    if (s_pcl_verbosity_level.find ("ALWAYS") != std::string::npos)          s_VerbosityLevel = L_ALWAYS;
    else if (s_pcl_verbosity_level.find ("ERROR") != std::string::npos)      s_VerbosityLevel = L_ERROR;
    else if (s_pcl_verbosity_level.find ("WARN") != std::string::npos)       s_VerbosityLevel = L_WARN;
    else if (s_pcl_verbosity_level.find ("INFO") != std::string::npos)       s_VerbosityLevel = L_INFO;
    else if (s_pcl_verbosity_level.find ("DEBUG") != std::string::npos)      s_VerbosityLevel = L_DEBUG;
    else if (s_pcl_verbosity_level.find ("VERBOSE") != std::string::npos)    s_VerbosityLevel = L_VERBOSE;
    else printf ("Warning: invalid PCL_VERBOSITY_LEVEL set (%s)\n", s_pcl_verbosity_level.c_str ());
  }

  s_NeedVerbosityInit = false;
  return true;
}

void
pcl::console::Logger::print(FILE* stream, const LogRecord& logEntry)
{
  if (!pcl::console::isVerbosityLevelEnabled(logEntry.level))
    return;

  if (logcallback)
    logcallback(logEntry);
  else {
    switch (logEntry.level) {
    case L_DEBUG:
      change_text_color(stream, TT_RESET, TT_GREEN);
      break;
    case L_WARN:
      change_text_color(stream, TT_BRIGHT, TT_YELLOW);
      break;
    case L_ERROR:
      change_text_color(stream, TT_BRIGHT, TT_RED);
      break;
    case L_ALWAYS:
    case L_INFO:
    case L_VERBOSE:
    default:
      break;
    }

    fputs(logEntry.message.c_str(), stream);

    reset_text_color(stream);
  }
}

void
pcl::console::Logger::print(const LogRecord& logEntry)
{
  FILE* stream =
      (logEntry.level == L_WARN || logEntry.level == L_ERROR) ? stderr : stdout;

  print(stream, logEntry);
}

void
pcl::console::Logger::print_highlight(FILE* stream, const LogRecord& logEntry)
{
  if (logcallback)
    logcallback(logEntry);
  else {
    change_text_color(stream, TT_BRIGHT, TT_GREEN);
    fputs("> ", stream);
    reset_text_color(stream);

    fputs(logEntry.message.c_str(), stream);
  }
}

void
pcl::console::Logger::print_highlight(const LogRecord& logEntry)
{
  print_highlight(stdout, logEntry);
}

void
pcl::console::Logger::print_value(FILE* stream, const LogRecord& logEntry)
{
  if (logcallback)
    logcallback(logEntry);
  else {
    change_text_color(stream, TT_RESET, TT_CYAN);

    fputs(logEntry.message.c_str(), stream);

    reset_text_color(stream);
  }
}

void
pcl::console::Logger::print_value(const LogRecord& logEntry)
{
  print_value(stdout, logEntry);
}

void
pcl::console::Logger::print_color(
    FILE* stream, int attr, int fg, const LogRecord& logEntry)
{
  if (logcallback)
    logcallback(logEntry);
  else {
    change_text_color(stream, attr, fg);
    fputs(logEntry.message.c_str(), stream);
    reset_text_color(stream);
  }
}
