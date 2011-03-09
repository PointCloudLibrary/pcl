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
 * $Id: parse.cpp 30713 2010-07-09 20:00:51Z rusu $
 *
 */

#include <stdio.h>
#include <pcl/terminal_tools/parse.h>
#include <pcl/terminal_tools/print.h>

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for a specific given command line argument. Returns the value 
  * sent as a string.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the string value to search for
  * \param val the resultant value
  */
int
  terminal_tools::parse_argument (int argc, char** argv, const char* str, std::string &val)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = std::string (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for a specific given command line argument. Returns the value 
  * sent as a boolean.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the string value to search for
  * \param val the resultant value
  */
int
  terminal_tools::parse_argument (int argc, char** argv, const char* str, bool &val)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = (bool)atoi (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for a specific given command line argument. Returns the value 
  * sent as a double.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the string value to search for
  * \param val the resultant value
  */
int
  terminal_tools::parse_argument (int argc, char** argv, const char* str, double &val)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atof (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for a specific given command line argument. Returns the value 
  * sent as an int.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the string value to search for
  * \param val the resultant value
  */
int
  terminal_tools::parse_argument (int argc, char** argv, const char* str, int &val)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atoi (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for a specific given command line argument. Returns the value 
  * sent as an unsigned int.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the string value to search for
  * \param val the resultant value
  */
int
  terminal_tools::parse_argument (int argc, char** argv, const char* str, unsigned int &val)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      val = atoi (argv[i]);
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse command line arguments for file names. Returns a vector with 
  * file names indices.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param extension to search for
  */
std::vector<int>
  terminal_tools::parse_file_extension_argument (int argc, char** argv, const std::string &extension)
{
  std::vector<int> indices;
  for (int i = 1; i < argc; ++i)
  {
    std::string fname = std::string (argv[i]);
    std::string ext = extension;
    
    // Needs to be at least 4: .ext
    if (fname.size () <= 4)
      continue;
    
    // For being case insensitive
    std::transform (fname.begin (), fname.end (), fname.begin (), tolower);
    std::transform (ext.begin (), ext.end (), ext.begin (), tolower);
    
    // Check if found
    std::string::size_type it;
    if ((it = fname.find (ext)) != std::string::npos)
    {
      // Additional check: we want to be able to differentiate between .p and .png
      if ((ext.size () - (fname.size () - it)) == 0)
        indices.push_back (i);
    }
  }
  return (indices);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (2x values comma 
  * separated). Returns the values sent as doubles.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param f the first output value
  * \param s the second output value
  */
int
  terminal_tools::parse_2x_arguments (int argc, char** argv, const char* str, double &f, double &s, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2 && debug)
      {
        print_error ("[parse_2x_arguments] Number of values for %s (%d) different than 2!\n", str, (int)values.size ());
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (2x values comma 
  * separated). Returns the values sent as ints.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param f the first output value
  * \param s the second output value
  */
int
  terminal_tools::parse_2x_arguments (int argc, char** argv, const char* str, int &f, int &s, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2 && debug)
      {
        print_error ("[parse_2x_arguments] Number of values for %s (%d) different than 2!\n", str, (int)values.size ());
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (3x values comma 
  * separated). Returns the values sent as doubles.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param f the first output value
  * \param s the second output value
  * \param t the third output value
  */
int
  terminal_tools::parse_3x_arguments (int argc, char** argv, const char* str, double &f, double &s, double &t, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3 && debug)
      {
        print_error ("[parse_3x_arguments] Number of values for %s (%d) different than 3!\n", str, (int)values.size ());
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (3x values comma 
  * separated). Returns the values sent as ints.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param f the first output value
  * \param s the second output value
  * \param t the third output value
  */
int
  terminal_tools::parse_3x_arguments (int argc, char** argv, const char* str, int &f, int &s, int &t, bool debug)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3 && debug)
      {
        print_error ("[parse_3x_arguments] Number of values for %s (%d) different than 3!\n", str, (int)values.size ());
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      t = atoi (values.at (2).c_str ());
      return (i-1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (multiple occurances 
  * of the same command line parameter). Returns the values sent as a vector.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param values the resultant output values
  */
bool
  terminal_tools::parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<int> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      int val = atoi (argv[i]);
      values.push_back (val);
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (multiple occurances 
  * of the same command line parameter). Returns the values sent as a vector.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param values the resultant output values
  */
bool
  terminal_tools::parse_multiple_arguments (int argc, char** argv, const char* str, std::vector<double> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      double val = atof (argv[i]);
      values.push_back (val);
    }
  }
  if (values.size () == 0)
    return (false);
  else
    return (true);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (multiple occurances 
  * of 2x argument groups, separated by commas). Returns 2 vectors holding the 
  * given values.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param values_f the first vector of output values
  * \param values_s the second vector of output values
  */
bool 
  terminal_tools::parse_multiple_2x_arguments (int argc, char** argv, const char* str, std::vector<double> &values_f, std::vector<double> &values_s)
{
  double f, s;
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 2)
      {
        print_error ("[parse_multiple_2x_arguments] Number of values for %s (%d) different than 2!\n", str, (int)values.size ());
        return (false);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      values_f.push_back (f);
      values_s.push_back (s);
    }
  }
  if (values_f.size () == 0)
    return (false);
  else
    return (true);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Parse for specific given command line arguments (multiple occurances 
  * of 3x argument groups, separated by commas). Returns 3 vectors holding the 
  * given values.
  * \param argc the number of command line arguments
  * \param argv the command line arguments
  * \param str the command line argument to search for
  * \param values_f the first vector of output values
  * \param values_s the second vector of output values
  * \param values_t the third vector of output values
  */
bool
  terminal_tools::parse_multiple_3x_arguments (int argc, char** argv, const char* str,
                                               std::vector<double> &values_f, 
                                               std::vector<double> &values_s, 
                                               std::vector<double> &values_t)
{
  double f, s, t;
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);
      if (values.size () != 3)
      {
        print_error ("[parse_multiple_3x_arguments] Number of values for %s (%d) different than 3!\n", str, (int)values.size ());
        return (false);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      values_f.push_back (f);
      values_s.push_back (s);
      values_t.push_back (t);
    }
  }
  if (values_f.size () == 0)
    return (false);
  else
    return (true);
}

