/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#pragma once

#include <vector>
#include <sstream>
#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace console
  {
    /** \brief Finds whether the argument with name "argument_name" is in the argument list "argv".
      * An example for a widely used switch argument is the "-r" flag for unix commands that indicates whether
      * the command should run recursively or not.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] argument_name the string value to search for
      * \return true if argument was found, false otherwise
      * \note find_switch is simply returning find_argument != -1.
      */
    PCL_EXPORTS bool
    find_switch (int argc, const char * const * argv, const char * argument_name);

    /** \brief Finds the position of the argument with name "argument_name" in the argument list "argv"
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] argument_name the string value to search for
      * \return index of found argument or -1 if arguments do not appear in list
     */
    PCL_EXPORTS int
    find_argument (int argc, const char * const * argv, const char * argument_name);

    /** \brief Template version for parsing arguments. Template parameter needs to have input stream operator overloaded!
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] argument_name the name of the argument to search for
      * \param[out] value The value of the argument
      * \return index of found argument or -1 if arguments do not appear in list
     */
    template<typename Type> int
    parse (int argc, const char * const * argv, const char * argument_name, Type& value)
    {
      int index = find_argument (argc, argv, argument_name) + 1;

      if (index > 0 && index < argc)
      {
        std::istringstream stream;
        stream.clear ();
        stream.str (argv[index]);
        stream >> value;
      }

      return (index - 1);
    }

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, std::string &val);

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, bool &val);

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, float &val);
    
    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, double &val);

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, int &val);

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, unsigned int &val);

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, long int &val) noexcept;

    /** \brief Parse for a specific given command line argument.
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] val the resultant value
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_argument (int argc, const char * const * argv, const char * str, char &val);

    /** \brief Parse for specific given command line arguments (2x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[in] debug whether to print debug info or not
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_2x_arguments (int argc, const char * const * argv, const char * str, float &f, float &s, bool debug = true);

    /** \brief Parse for specific given command line arguments (2x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[in] debug whether to print debug info or not
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_2x_arguments (int argc, const char * const * argv, const char * str, double &f, double &s, bool debug = true);

    /** \brief Parse for specific given command line arguments (2x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[in] debug whether to print debug info or not
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_2x_arguments (int argc, const char * const * argv, const char * str, int &f, int &s, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[out] t the third output value
      * \param[in] debug whether to print debug info or not
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_3x_arguments (int argc, const char * const * argv, const char * str, float &f, float &s, float &t, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[out] t the third output value
      * \param[in] debug whether to print debug info or not
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_3x_arguments (int argc, const char * const * argv, const char * str, double &f, double &s, double &t, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] f the first output value
      * \param[out] s the second output value
      * \param[out] t the third output value
      * \param[in] debug whether to print debug info or not
      * return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_3x_arguments (int argc, const char * const * argv, const char * str, int &f, int &s, int &t, bool debug = true);

    /** \brief Parse for specific given command line arguments (3x values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] v the vector into which the parsed values will be copied
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_x_arguments (int argc, const char * const * argv, const char * str, std::vector<double>& v);

    /** \brief Parse for specific given command line arguments (N values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] v the vector into which the parsed values will be copied
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_x_arguments (int argc, const char * const * argv, const char * str, std::vector<float>& v);

    /** \brief Parse for specific given command line arguments (N values comma
      * separated).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] v the vector into which the parsed values will be copied
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS int
    parse_x_arguments (int argc, const char * const * argv, const char * str, std::vector<int>& v);

    /** \brief Parse for specific given command line arguments (multiple occurrences
      * of the same command line parameter).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values the resultant output values
      * \return index of found argument or -1 if arguments do not appear in list
      */
    PCL_EXPORTS bool
    parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<int> &values);

    /** \brief Parse for specific given command line arguments (multiple occurrences
      * of the same command line parameter).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values the resultant output values
      * \return true if found, false otherwise
      */
    PCL_EXPORTS bool
    parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<float> &values);

    /** \brief Parse for specific given command line arguments (multiple occurrences
      * of the same command line parameter).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values the resultant output values
      * \return true if found, false otherwise
      */
    PCL_EXPORTS bool
    parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<double> &values);

    /** \brief Parse for a specific given command line argument (multiple occurrences
      * of the same command line parameter).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the string value to search for
      * \param[out] values the resultant output values
      * \return true if found, false otherwise
      */
    PCL_EXPORTS bool
    parse_multiple_arguments (int argc, const char * const * argv, const char * str, std::vector<std::string> &values);

    /** \brief Parse command line arguments for file names with given extension (multiple occurrences
      * of 2x argument groups, separated by commas).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values_f the first vector of output values
      * \param[out] values_s the second vector of output values
      * \return true if found, false otherwise
      */
    PCL_EXPORTS bool
    parse_multiple_2x_arguments (int argc, const char * const * argv, const char * str,
                                 std::vector<double> &values_f, 
                                 std::vector<double> &values_s);

    /** \brief Parse command line arguments for file names with given extension (multiple occurrences
      * of 3x argument groups, separated by commas).
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] str the command line argument to search for
      * \param[out] values_f the first vector of output values
      * \param[out] values_s the second vector of output values
      * \param[out] values_t the third vector of output values
      * \return true if found, false otherwise
      */
    PCL_EXPORTS bool
    parse_multiple_3x_arguments (int argc, const char * const * argv, const char * str,
                                 std::vector<double> &values_f, 
                                 std::vector<double> &values_s, 
                                 std::vector<double> &values_t);

    /** \brief Parse command line arguments for file names with given extension vector
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] extensions the extensions to search for
      * \return a vector with file names indices
      */
    PCL_EXPORTS std::vector<int>
    parse_file_extension_argument (int argc, const char * const * argv,
      const std::vector<std::string> &extensions);

    /** \brief Parse command line arguments for file names with given extension
      * \param[in] argc the number of command line arguments
      * \param[in] argv the command line arguments
      * \param[in] ext the extension to search for
      * \return a vector with file names indices
      */
    PCL_EXPORTS std::vector<int>
    parse_file_extension_argument (int argc, const char * const * argv, const std::string &ext);
  }
}
