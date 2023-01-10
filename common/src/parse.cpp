/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *
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
 */

#include <cctype>
#include <cerrno>
#include <limits>
#include <type_traits>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <boost/algorithm/string.hpp>

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::find_argument (int argc, const char * const * argv, const char * argument_name)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if (strcmp (argv[i], argument_name) == 0)
    {
      return (i);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::find_switch (int argc, const char * const * argv, const char * argument_name)
{
  return (find_argument (argc, argv, argument_name) != -1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, std::string &val)
{
  int index = find_argument (argc, argv, str) + 1;
  if (index > 0 && index < argc )
    val = argv[index];

  return index - 1;
}

////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
namespace console
{
template <class T, class V = T(*)(const char*, const char**)> int
parse_generic (V convert_func, int argc, const char* const* argv, const char* str, T& val)
{
  char *endptr = nullptr;
  int index = find_argument (argc, argv, str) + 1;
  errno = 0;

  if (index > 0 && index < argc )
  {
    val = convert_func (argv[index], &endptr);  // similar to strtol, strtod, strtof
    // handle out-of-range, junk at the end and no conversion
    if (errno == ERANGE || *endptr != '\0' || str == endptr)
    {
      return -1;
    }
  }

  return (index - 1);
}

int
parse_argument (int argc, const char * const * argv, const char * str, long int &val) noexcept
{
  const auto strtol_10 = [](const char *str, char **str_end){ return strtol(str, str_end, 10); };
  return parse_generic(strtol_10, argc, argv, str, val);
}

int
parse_argument (int argc, const char * const * argv, const char * str, long long int &val) noexcept
{
  const auto strtoll_10 = [](const char *str, char **str_end){ return strtoll(str, str_end, 10); };
  return parse_generic(strtoll_10, argc, argv, str, val);
}

int
parse_argument (int argc, const char * const * argv, const char * str, unsigned long long int &val) noexcept
{
  long long int dummy = -1;
  const auto ret = parse_argument (argc, argv, str, dummy);
  if ((ret == -1) || dummy < 0)
  {
      return -1;
  }
  val = dummy;
  return ret;
}

namespace detail
{
template <typename T, typename U>
constexpr auto legally_representable_v = (std::numeric_limits<T>::max () >= std::numeric_limits<U>::max ()) &&
                                       (std::numeric_limits<T>::lowest () <= std::numeric_limits<U>::lowest ());
template <typename T, typename U>
struct legally_representable {
    constexpr static bool value = legally_representable_v<T, U>;
};

// assumptions:
// * either long int or long long int is a valid type for storing Integral
// * unsigned long long int is handled specially
template <typename Integral>
using primary_legal_input_type = std::conditional_t<legally_representable_v<long int, Integral>,
                                                    long int, long long int>;

// special handling if unsigned [long] int is of same size as long long int
template <typename Integral>
using legal_input_type = std::conditional_t<(std::is_unsigned<Integral>::value &&
                                             (sizeof (Integral) == sizeof (long long int))),
                                            unsigned long long int,
                                            primary_legal_input_type<Integral>>;
}

template <typename T>
using IsIntegral = std::enable_if_t<std::is_integral<T>::value, bool>;

template <typename T, IsIntegral<T> = true> int
parse_argument (int argc, const char * const * argv, const char * str, T &val) noexcept
{
  using InputType = detail::legal_input_type<T>;
  InputType dummy;
  const auto ret = parse_argument (argc, argv, str, dummy);
  if ((ret == -1) ||
      (dummy < static_cast<InputType> (std::numeric_limits<T>::min ())) ||
      (dummy > static_cast<InputType> (std::numeric_limits<T>::max ())))
  {
    return -1;
  }

  val = static_cast<T> (dummy);
  return ret;
}
}
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, double &val)
{
  // added lambda wrapper for `strtod` to handle noexcept-type warning in GCC 7,
  // refer to: https://stackoverflow.com/questions/46798456/handling-gccs-noexcept-type-warning
  const auto strtod_l = [](const char *str, char **str_end){ return strtod(str, str_end); };
  return parse_generic(strtod_l, argc, argv, str, val);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, float &val)
{
  // added lambda wrapper for `strtof` to handle noexcept-type warning in GCC 7,
  // refer to: https://stackoverflow.com/questions/46798456/handling-gccs-noexcept-type-warning
  const auto strtof_l = [](const char *str, char **str_end){ return strtof(str, str_end); };
  return parse_generic(strtof_l, argc, argv, str, val);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, unsigned int &val)
{
  return parse_argument<unsigned int> (argc, argv, str, val);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, int &val)
{
  return parse_argument<int> (argc, argv, str, val);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, bool &val)
{
  long int dummy;
  const auto ret = parse_argument (argc, argv, str, dummy);
  if (ret != -1)
  {
    val = static_cast<bool> (dummy);
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_argument (int argc, const char * const * argv, const char * str, char &val)
{
  int index = find_argument (argc, argv, str) + 1;

  if (index > 0 && index < argc )
    val = argv[index][0];

  return (index - 1);
}

////////////////////////////////////////////////////////////////////////////////
std::vector<int>
pcl::console::parse_file_extension_argument (int argc, const char * const * argv,
  const std::vector<std::string> &extension)
{
  std::vector<int> indices;
  for (int i = 1; i < argc; ++i)
  {
    std::string fname = std::string (argv[i]);
    for (auto ext : extension)
    {
      // Needs to be at least 4: .ext
      if (fname.size () <= 4)
        continue;

      // For being case insensitive
      std::transform (fname.begin (), fname.end (), fname.begin (), tolower);
      std::transform (ext.begin (), ext.end (), ext.begin (), tolower);

      // Check if found
      std::string::size_type it;
      if ((it = fname.rfind (ext)) != std::string::npos)
      {
        // Additional check: we want to be able to differentiate between .p and .png
        if ((ext.size () - (fname.size () - it)) == 0)
        {
          indices.push_back (i);
          break;
        }
      }
    }
  }
  return (indices);
}

////////////////////////////////////////////////////////////////////////////////
std::vector<int>
pcl::console::parse_file_extension_argument (int argc, const char * const * argv,
  const std::string &ext)
{
  std::vector<std::string> extensions;
  extensions.push_back (ext);
  return parse_file_extension_argument (argc, argv, extensions);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_2x_arguments (int argc, const char * const * argv, const char * str, float &f, float &s, bool debug)
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
        print_error ("[parse_2x_arguments] Number of values for %s (%lu) different than 2!\n", str, values.size ());
        return (-2);
      }
      f = static_cast<float> (atof (values.at (0).c_str ()));
      s = static_cast<float> (atof (values.at (1).c_str ()));
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_2x_arguments (int argc, const char * const * argv, const char * str, double &f, double &s, bool debug)
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
        print_error ("[parse_2x_arguments] Number of values for %s (%lu) different than 2!\n", str, values.size ());
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_2x_arguments (int argc, const char * const * argv, const char * str, int &f, int &s, bool debug)
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
        print_error ("[parse_2x_arguments] Number of values for %s (%lu) different than 2!\n", str, values.size ());
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_3x_arguments (int argc, const char * const * argv, const char * str, float &f, float &s, float &t, bool debug)
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
        print_error ("[parse_3x_arguments] Number of values for %s (%lu) different than 3!\n", str, values.size ());
        return (-2);
      }
      f = static_cast<float> (atof (values.at (0).c_str ()));
      s = static_cast<float> (atof (values.at (1).c_str ()));
      t = static_cast<float> (atof (values.at (2).c_str ()));
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_3x_arguments (int argc, const char * const * argv, const char * str, double &f, double &s, double &t, bool debug)
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
        print_error ("[parse_3x_arguments] Number of values for %s (%lu) different than 3!\n", str, values.size ());
        return (-2);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      t = atof (values.at (2).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_3x_arguments (int argc, const char * const * argv, const char * str, int &f, int &s, int &t, bool debug)
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
        print_error ("[parse_3x_arguments] Number of values for %s (%lu) different than 3!\n", str, values.size ());
        return (-2);
      }
      f = atoi (values.at (0).c_str ());
      s = atoi (values.at (1).c_str ());
      t = atoi (values.at (2).c_str ());
      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_x_arguments (int argc, const char * const * argv, const char * str, std::vector<double>& v)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);

      v.resize (values.size ());
      for (std::size_t j = 0; j < v.size (); ++j)
        v[j] = atof (values.at (j).c_str ());

      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_x_arguments (int argc, const char * const * argv, const char * str, std::vector<float>& v)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);

      v.resize (values.size ());
      for (std::size_t j = 0; j < v.size (); ++j)
        v[j] = static_cast<float> (atof (values.at (j).c_str ()));

      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
int
pcl::console::parse_x_arguments (int argc, const char * const * argv, const char * str, std::vector<int>& v)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      // look for ',' as a separator
      std::vector<std::string> values;
      boost::split (values, argv[i], boost::is_any_of (","), boost::token_compress_on);

      v.resize (values.size ());
      for (std::size_t j = 0; j < v.size (); ++j)
        v[j] = atoi (values.at (j).c_str ());

      return (i - 1);
    }
  }
  return (-1);
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<int> &values)
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
  return (!values.empty ());
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<double> &values)
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
  return (!values.empty ());
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<float> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      float val = static_cast<float> (atof (argv[i]));
      values.push_back (val);
    }
  }
  return (!values.empty ());
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<std::string> &values)
{
  for (int i = 1; i < argc; ++i)
  {
    // Search for the string
    if ((strcmp (argv[i], str) == 0) && (++i < argc))
    {
      values.emplace_back(argv[i]);
    }
  }
  return (!values.empty ());
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::parse_multiple_2x_arguments (int argc, const char * const * argv, const char * str, std::vector<double> &values_f, std::vector<double> &values_s)
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
        print_error ("[parse_multiple_2x_arguments] Number of values for %s (%lu) different than 2!\n", str, values.size ());
        return (false);
      }
      f = atof (values.at (0).c_str ());
      s = atof (values.at (1).c_str ());
      values_f.push_back (f);
      values_s.push_back (s);
    }
  }
  return (!values_f.empty ());
}

////////////////////////////////////////////////////////////////////////////////
bool
pcl::console::parse_multiple_3x_arguments (int argc, const char * const * argv, const char * str,
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
        print_error ("[parse_multiple_3x_arguments] Number of values for %s (%lu) different than 3!\n", str, values.size ());
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
  return (!values_f.empty ());
}

