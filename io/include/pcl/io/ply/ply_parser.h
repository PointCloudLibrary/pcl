/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2007-2012, Ares Lagae
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_IO_PLY_PLY_PARSER_H
#define PCL_IO_PLY_PLY_PARSER_H

#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <string>
#include <vector>

#ifdef BUILD_Maintainer
#  if defined __GNUC__
#    if __GNUC__ == 4 && __GNUC_MINOR__ > 3
#      pragma GCC diagnostic ignored "-Weffc++"
#      pragma GCC diagnostic ignored "-pedantic"
#    else
#      pragma GCC system_header 
#    endif
#  elif defined _MSC_VER
#    pragma warning(push, 1)
#  endif
#endif

#include <pcl/io/boost.h>

#include <pcl/io/ply/ply.h>
#include <pcl/io/ply/io_operators.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace io
  {
    namespace ply
    {
      /** Class ply_parser parses a PLY file and generates appropriate atomic
        * parsers for the body.
        * \author Ares Lagae as part of libply, Nizar Sallem
        * Ported with agreement from the author under the terms of the BSD
        * license.
        */     
      class PCL_EXPORTS ply_parser
      {
        public:

          typedef boost::function<void (std::size_t, const std::string&)> info_callback_type;
          typedef boost::function<void (std::size_t, const std::string&)> warning_callback_type;
          typedef boost::function<void (std::size_t, const std::string&)> error_callback_type;
         
          typedef boost::function<void ()> magic_callback_type;
          typedef boost::function<void (format_type, const std::string&)> format_callback_type;
          typedef boost::function<void (const std::string&)> comment_callback_type;
          typedef boost::function<void (const std::string&)> obj_info_callback_type;
          typedef boost::function<bool ()> end_header_callback_type;
         
          typedef boost::function<void ()> begin_element_callback_type;
          typedef boost::function<void ()> end_element_callback_type;
          typedef boost::tuple<begin_element_callback_type, end_element_callback_type> element_callbacks_type;
          typedef boost::function<element_callbacks_type (const std::string&, std::size_t)> element_definition_callback_type;
         
          template <typename ScalarType>
          struct scalar_property_callback_type
          {
            typedef boost::function<void (ScalarType)> type;
          };

          template <typename ScalarType>
          struct scalar_property_definition_callback_type
          {
            typedef typename scalar_property_callback_type<ScalarType>::type scalar_property_callback_type;
            typedef boost::function<scalar_property_callback_type (const std::string&, const std::string&)> type;
          };
       
          typedef boost::mpl::vector<int8, int16, int32, uint8, uint16, uint32, float32, float64> scalar_types;

          class scalar_property_definition_callbacks_type
          {
            private:
              template <typename T>
              struct callbacks_element
              {
//                callbacks_element () : callback ();
                typedef T scalar_type;
                typename scalar_property_definition_callback_type<scalar_type>::type callback;
              };
             
              typedef boost::mpl::inherit_linearly<
                scalar_types,
                boost::mpl::inherit<
                boost::mpl::_1,
                callbacks_element<boost::mpl::_2>
                >
                >::type callbacks;
              callbacks callbacks_;
             
            public:
              template <typename ScalarType>
              const typename scalar_property_definition_callback_type<ScalarType>::type&
              get () const
              {
                return (static_cast<const callbacks_element<ScalarType>&> (callbacks_).callback);
              }
             
              template <typename ScalarType>
              typename scalar_property_definition_callback_type<ScalarType>::type&
              get ()
              {
                return (static_cast<callbacks_element<ScalarType>&> (callbacks_).callback);
              }
             
              template <typename ScalarType>
              friend typename scalar_property_definition_callback_type<ScalarType>::type&
              at (scalar_property_definition_callbacks_type& scalar_property_definition_callbacks);
           
              template <typename ScalarType>
              friend const typename scalar_property_definition_callback_type<ScalarType>::type&
              at (const scalar_property_definition_callbacks_type& scalar_property_definition_callbacks);
          };

          template <typename ScalarType> static
          typename scalar_property_definition_callback_type<ScalarType>::type&
          at (scalar_property_definition_callbacks_type& scalar_property_definition_callbacks)
          {
              return (scalar_property_definition_callbacks.get<ScalarType> ());
          }

          
          template <typename ScalarType> static
          const typename scalar_property_definition_callback_type<ScalarType>::type&
          at (const scalar_property_definition_callbacks_type& scalar_property_definition_callbacks)
          {
              return (scalar_property_definition_callbacks.get<ScalarType> ());
          }

          template <typename SizeType, typename ScalarType>
          struct list_property_begin_callback_type
          {
            typedef boost::function<void (SizeType)> type;
          };
         
          template <typename SizeType, typename ScalarType>
          struct list_property_element_callback_type
          {
            typedef boost::function<void (ScalarType)> type;
          };
       
          template <typename SizeType, typename ScalarType>
          struct list_property_end_callback_type
          {
            typedef boost::function<void ()> type;
          };

          template <typename SizeType, typename ScalarType>
          struct list_property_definition_callback_type
          {
            typedef typename list_property_begin_callback_type<SizeType, ScalarType>::type list_property_begin_callback_type;
            typedef typename list_property_element_callback_type<SizeType, ScalarType>::type list_property_element_callback_type;
            typedef typename list_property_end_callback_type<SizeType, ScalarType>::type list_property_end_callback_type;
            typedef boost::function<
            boost::tuple<
            list_property_begin_callback_type,
              list_property_element_callback_type,
              list_property_end_callback_type
              > (const std::string&, const std::string&)> type;
          };

          typedef boost::mpl::vector<uint8, uint16, uint32> size_types;
     
          class list_property_definition_callbacks_type
          {
            private:
              template <typename T> struct pair_with : boost::mpl::pair<T,boost::mpl::_> {};
              template<typename Sequence1, typename Sequence2>
          
                struct sequence_product :
                  boost::mpl::fold<Sequence1, boost::mpl::vector0<>,
                    boost::mpl::joint_view<
                      boost::mpl::_1,boost::mpl::transform<Sequence2, pair_with<boost::mpl::_2> > > >
                {};

              template <typename T>
              struct callbacks_element
              {
                typedef typename T::first size_type;
                typedef typename T::second scalar_type;
                typename list_property_definition_callback_type<size_type, scalar_type>::type callback;
              };
           
              typedef boost::mpl::inherit_linearly<sequence_product<size_types, scalar_types>::type, boost::mpl::inherit<boost::mpl::_1, callbacks_element<boost::mpl::_2> > >::type callbacks;
              callbacks callbacks_;
     
            public:
              template <typename SizeType, typename ScalarType>
              typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              get ()
              {
                return (static_cast<callbacks_element<boost::mpl::pair<SizeType, ScalarType> >&> (callbacks_).callback);
              }

              template <typename SizeType, typename ScalarType>
              const typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              get () const
              {
                return (static_cast<const callbacks_element<boost::mpl::pair<SizeType, ScalarType> >&> (callbacks_).callback);
              }

              template <typename SizeType, typename ScalarType>
              friend typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              at (list_property_definition_callbacks_type& list_property_definition_callbacks);
           
              template <typename SizeType, typename ScalarType>
              friend const typename list_property_definition_callback_type<SizeType, ScalarType>::type&
              at (const list_property_definition_callbacks_type& list_property_definition_callbacks);
          };
          
          template <typename SizeType, typename ScalarType> static
          typename list_property_definition_callback_type<SizeType, ScalarType>::type&
          at (list_property_definition_callbacks_type& list_property_definition_callbacks)
          {
              return (list_property_definition_callbacks.get<SizeType, ScalarType> ());
          }
          
          template <typename SizeType, typename ScalarType> static
          const typename list_property_definition_callback_type<SizeType, ScalarType>::type&
          at (const list_property_definition_callbacks_type& list_property_definition_callbacks)
          {
              return (list_property_definition_callbacks.get<SizeType, ScalarType> ());
          }


          inline void
          info_callback (const info_callback_type& info_callback);

          inline void
          warning_callback (const warning_callback_type& warning_callback);

          inline void
          error_callback (const error_callback_type& error_callback);

          inline void
          magic_callback (const magic_callback_type& magic_callback);

          inline void
          format_callback (const format_callback_type& format_callback);

          inline void
          element_definition_callback (const element_definition_callback_type& element_definition_callback);

          inline void
          scalar_property_definition_callbacks (const scalar_property_definition_callbacks_type& scalar_property_definition_callbacks);

          inline void
          list_property_definition_callbacks (const list_property_definition_callbacks_type& list_property_definition_callbacks);

          inline void
          comment_callback (const comment_callback_type& comment_callback);

          inline void
          obj_info_callback (const obj_info_callback_type& obj_info_callback);

          inline void
          end_header_callback (const end_header_callback_type& end_header_callback);

          typedef int flags_type;
          enum flags { };

          ply_parser (flags_type flags = 0) : 
            flags_ (flags), 
            comment_callback_ (), obj_info_callback_ (), end_header_callback_ (), 
            line_number_ (0), current_element_ ()
          {}
              
          bool parse (const std::string& filename);
          //inline bool parse (const std::string& filename);

        private:
            
          struct property
          {
            property (const std::string& name) : name (name) {}
            virtual ~property () {}
            virtual bool parse (class ply_parser& ply_parser, format_type format, std::istream& istream) = 0;
            std::string name;
          };
            
          template <typename ScalarType>
          struct scalar_property : public property
          {
            typedef ScalarType scalar_type;
            typedef typename scalar_property_callback_type<scalar_type>::type callback_type;
            scalar_property (const std::string& name, callback_type callback)
              : property (name)
              , callback (callback)
            {}
            bool parse (class ply_parser& ply_parser, 
                        format_type format, 
                        std::istream& istream) 
            { 
              return ply_parser.parse_scalar_property<scalar_type> (format, istream, callback); 
            }
            callback_type callback;
          };

          template <typename SizeType, typename ScalarType>
          struct list_property : public property
          {
            typedef SizeType size_type;
            typedef ScalarType scalar_type;
            typedef typename list_property_begin_callback_type<size_type, scalar_type>::type begin_callback_type;
            typedef typename list_property_element_callback_type<size_type, scalar_type>::type element_callback_type;
            typedef typename list_property_end_callback_type<size_type, scalar_type>::type end_callback_type;
            list_property (const std::string& name, 
                           begin_callback_type begin_callback, 
                           element_callback_type element_callback, 
                           end_callback_type end_callback)
              : property (name)
              , begin_callback (begin_callback)
              , element_callback (element_callback)
              , end_callback (end_callback)
            {}
            bool parse (class ply_parser& ply_parser, 
                        format_type format, 
                        std::istream& istream) 
            { 
              return ply_parser.parse_list_property<size_type, scalar_type> (format, 
                                                                             istream,
                                                                             begin_callback,
                                                                             element_callback,
                                                                             end_callback);
            }
            begin_callback_type begin_callback;
            element_callback_type element_callback;
            end_callback_type end_callback;
          };
        
          struct element
          {
            element (const std::string& name, 
                    std::size_t count, 
                    const begin_element_callback_type& begin_element_callback, 
                    const end_element_callback_type& end_element_callback)
              : name (name)
              , count (count)
              , begin_element_callback (begin_element_callback)
              , end_element_callback (end_element_callback)
              , properties ()
            {}
            std::string name;
            std::size_t count;
            begin_element_callback_type begin_element_callback;
            end_element_callback_type end_element_callback;
            std::vector<boost::shared_ptr<property> > properties;
          };

          flags_type flags_;
          
          info_callback_type info_callback_;
          warning_callback_type warning_callback_;
          error_callback_type error_callback_;
          
          magic_callback_type magic_callback_;
          format_callback_type format_callback_;
          element_definition_callback_type element_definition_callbacks_;
          scalar_property_definition_callbacks_type scalar_property_definition_callbacks_;
          list_property_definition_callbacks_type list_property_definition_callbacks_;
          comment_callback_type comment_callback_;
          obj_info_callback_type obj_info_callback_;
          end_header_callback_type end_header_callback_;
          
          template <typename ScalarType> inline void 
          parse_scalar_property_definition (const std::string& property_name);

          template <typename SizeType, typename ScalarType> inline void 
          parse_list_property_definition (const std::string& property_name);
          
          template <typename ScalarType> inline bool 
          parse_scalar_property (format_type format, 
                                 std::istream& istream, 
                                 const typename scalar_property_callback_type<ScalarType>::type& scalar_property_callback);

          template <typename SizeType, typename ScalarType> inline bool 
          parse_list_property (format_type format, 
                               std::istream& istream, 
                               const typename list_property_begin_callback_type<SizeType, ScalarType>::type& list_property_begin_callback, 
                               const typename list_property_element_callback_type<SizeType, ScalarType>::type& list_property_element_callback, 
                               const typename list_property_end_callback_type<SizeType, ScalarType>::type& list_property_end_callback);
            
          std::size_t line_number_;
          element* current_element_;
      };
    } // namespace ply
  } // namespace io
} // namespace pcl

/* inline bool pcl::io::ply::ply_parser::parse (const std::string& filename) */
/* { */
/*   std::ifstream ifstream (filename.c_str ()); */
/*   return (parse (ifstream)); */
/* } */

inline void pcl::io::ply::ply_parser::info_callback (const info_callback_type& info_callback)
{
  info_callback_ = info_callback;
}
    
inline void pcl::io::ply::ply_parser::warning_callback (const warning_callback_type& warning_callback)
{
  warning_callback_ = warning_callback;
}
    
inline void pcl::io::ply::ply_parser::error_callback (const error_callback_type& error_callback)
{
  error_callback_ = error_callback;
}
    
inline void pcl::io::ply::ply_parser::magic_callback (const magic_callback_type& magic_callback)
{
  magic_callback_ = magic_callback;
}
    
inline void pcl::io::ply::ply_parser::format_callback (const format_callback_type& format_callback)
{
  format_callback_ = format_callback;
}
    
inline void pcl::io::ply::ply_parser::element_definition_callback (const element_definition_callback_type& element_definition_callback)
{
  element_definition_callbacks_ = element_definition_callback;
}
    
inline void pcl::io::ply::ply_parser::scalar_property_definition_callbacks (const scalar_property_definition_callbacks_type& scalar_property_definition_callbacks)
{
  scalar_property_definition_callbacks_ = scalar_property_definition_callbacks;
}
    
inline void pcl::io::ply::ply_parser::list_property_definition_callbacks (const list_property_definition_callbacks_type& list_property_definition_callbacks)
{
  list_property_definition_callbacks_ = list_property_definition_callbacks;
}
    
inline void pcl::io::ply::ply_parser::comment_callback (const comment_callback_type& comment_callback)
{
  comment_callback_ = comment_callback;
}

inline void pcl::io::ply::ply_parser::obj_info_callback (const obj_info_callback_type& obj_info_callback)
{
  obj_info_callback_ = obj_info_callback;
}

inline void pcl::io::ply::ply_parser::end_header_callback (const end_header_callback_type& end_header_callback)
{
  end_header_callback_ = end_header_callback;
}

template <typename ScalarType>
inline void pcl::io::ply::ply_parser::parse_scalar_property_definition (const std::string& property_name)
{
  typedef ScalarType scalar_type;
  typename scalar_property_definition_callback_type<scalar_type>::type& scalar_property_definition_callback = 
    scalar_property_definition_callbacks_.get<scalar_type> ();
  typename scalar_property_callback_type<scalar_type>::type scalar_property_callback;
  if (scalar_property_definition_callback)
  {
    scalar_property_callback = scalar_property_definition_callback (current_element_->name, property_name);
  }
  if (!scalar_property_callback)
  {
    if (warning_callback_)
    {
      warning_callback_ (line_number_, 
                        "property '" + std::string (type_traits<scalar_type>::name ()) + " " + 
                        property_name + "' of element '" + current_element_->name + "' is not handled");
    }
  }
  current_element_->properties.push_back (boost::shared_ptr<property> (new scalar_property<scalar_type> (property_name, scalar_property_callback)));
}

template <typename SizeType, typename ScalarType>
inline void pcl::io::ply::ply_parser::parse_list_property_definition (const std::string& property_name)
{
  typedef SizeType size_type;
  typedef ScalarType scalar_type;
  typedef typename  list_property_definition_callback_type<size_type, scalar_type>::type list_property_definition_callback_type;
  list_property_definition_callback_type& list_property_definition_callback = list_property_definition_callbacks_.get<size_type, scalar_type> ();
  typedef typename list_property_begin_callback_type<size_type, scalar_type>::type list_property_begin_callback_type;
  typedef typename list_property_element_callback_type<size_type, scalar_type>::type list_property_element_callback_type;
  typedef typename list_property_end_callback_type<size_type, scalar_type>::type list_property_end_callback_type;
  boost::tuple<list_property_begin_callback_type, list_property_element_callback_type, list_property_end_callback_type> list_property_callbacks;
  if (list_property_definition_callback)
  {
    list_property_callbacks = list_property_definition_callback (current_element_->name, property_name);
  }
  if (!boost::get<0> (list_property_callbacks) || !boost::get<1> (list_property_callbacks) || !boost::get<2> (list_property_callbacks))
  {
    if (warning_callback_)
    {
      warning_callback_ (line_number_, 
                        "property 'list " + std::string (type_traits<size_type>::name ()) + " " + 
                        std::string (type_traits<scalar_type>::name ()) + " " + 
                        property_name + "' of element '" + 
                        current_element_->name + "' is not handled");
    }
  }
  current_element_->properties.push_back (boost::shared_ptr<property> (
                                           new list_property<size_type, scalar_type> (
                                             property_name, 
                                             boost::get<0> (list_property_callbacks), 
                                             boost::get<1> (list_property_callbacks), 
                                             boost::get<2> (list_property_callbacks))));
}

template <typename ScalarType>
inline bool pcl::io::ply::ply_parser::parse_scalar_property (format_type format, 
                                                             std::istream& istream, 
                                                             const typename scalar_property_callback_type<ScalarType>::type& scalar_property_callback)
{
  using namespace io_operators;
  typedef ScalarType scalar_type;
  if (format == ascii_format)
  {
    std::string value_s;
    scalar_type value;
    char space = ' ';
    istream >> value_s;
    try
    {
      value = static_cast<scalar_type> (boost::lexical_cast<typename pcl::io::ply::type_traits<scalar_type>::parse_type> (value_s));
    }
    catch (boost::bad_lexical_cast &)
    {
      value = std::numeric_limits<scalar_type>::quiet_NaN ();
    }

    if (!istream.eof ())
      istream >> space >> std::ws;
    if (!istream || !isspace (space))
    {
      if (error_callback_)
        error_callback_ (line_number_, "parse error");
      return (false);
    }
    if (scalar_property_callback)
      scalar_property_callback (value);
    return (true);
  }
  else
  {
    scalar_type value = std::numeric_limits<scalar_type>::quiet_NaN ();
    istream.read (reinterpret_cast<char*> (&value), sizeof (scalar_type));
    if (!istream)
    {
      if (error_callback_)
        error_callback_ (line_number_, "parse error");
      return (false);
    }
    if (((format == binary_big_endian_format) && (host_byte_order == little_endian_byte_order)) ||
        ((format == binary_little_endian_format) && (host_byte_order == big_endian_byte_order)))
      swap_byte_order (value);
    if (scalar_property_callback)
      scalar_property_callback (value);
    return (true);
  }
}

template <typename SizeType, typename ScalarType>
inline bool pcl::io::ply::ply_parser::parse_list_property (format_type format, std::istream& istream, 
                                                           const typename list_property_begin_callback_type<SizeType, ScalarType>::type& list_property_begin_callback, 
                                                           const typename list_property_element_callback_type<SizeType, ScalarType>::type& list_property_element_callback, 
                                                           const typename list_property_end_callback_type<SizeType, ScalarType>::type& list_property_end_callback)
{
  using namespace io_operators;
  typedef SizeType size_type;
  typedef ScalarType scalar_type;
  if (format == ascii_format)
  {
    size_type size = std::numeric_limits<size_type>::infinity ();
    char space = ' ';
    istream >> size;
    if (!istream.eof ())
    {
      istream >> space >> std::ws;
    }
    if (!istream || !isspace (space))
    {
      if (error_callback_)
      {
        error_callback_ (line_number_, "parse error");
      }
      return (false);
    }
    if (list_property_begin_callback)
    {
      list_property_begin_callback (size);
    }
    for (std::size_t index = 0; index < size; ++index)
    {
      std::string value_s;
      scalar_type value;
      char space = ' ';
      istream >> value_s;
      try
      {
        value = static_cast<scalar_type> (boost::lexical_cast<typename pcl::io::ply::type_traits<scalar_type>::parse_type> (value_s));
      }
      catch (boost::bad_lexical_cast &)
      {
        value = std::numeric_limits<scalar_type>::quiet_NaN ();
      }

      if (!istream.eof ())
      {
        istream >> space >> std::ws;
      }
      if (!istream || !isspace (space))
      {
        if (error_callback_)
        {
          error_callback_ (line_number_, "parse error");
        }
        return (false);
      }
      if (list_property_element_callback)
      {
        list_property_element_callback (value);
      }
    }
    if (list_property_end_callback)
    {
      list_property_end_callback ();
    }
    return (true);
  }
  else
  {
    size_type size = std::numeric_limits<size_type>::infinity ();
    istream.read (reinterpret_cast<char*> (&size), sizeof (size_type));
    if (((format == binary_big_endian_format) && (host_byte_order == little_endian_byte_order)) || 
        ((format == binary_little_endian_format) && (host_byte_order == big_endian_byte_order)))
    {
      swap_byte_order (size);
    }
    if (!istream)
    {
      if (error_callback_)
      {
        error_callback_ (line_number_, "parse error");
      }
      return (false);
    }
    if (list_property_begin_callback)
    {
      list_property_begin_callback (size);
    }
    for (std::size_t index = 0; index < size; ++index) {
      scalar_type value  = std::numeric_limits<scalar_type>::quiet_NaN ();
      istream.read (reinterpret_cast<char*> (&value), sizeof (scalar_type));
      if (!istream) {
        if (error_callback_) {
          error_callback_ (line_number_, "parse error");
        }
        return (false);
      }
      if (((format == binary_big_endian_format) && (host_byte_order == little_endian_byte_order)) ||
          ((format == binary_little_endian_format) && (host_byte_order == big_endian_byte_order)))
      {
        swap_byte_order (value);
      }
      if (list_property_element_callback)
      {
        list_property_element_callback (value);
      }
    }
    if (list_property_end_callback)
    {
      list_property_end_callback ();
    }
    return (true);
  }
}

#ifdef BUILD_Maintainer
#  if defined __GNUC__
#    if __GNUC__ == 4 && __GNUC_MINOR__ > 3
#      pragma GCC diagnostic warning "-Weffc++"
#      pragma GCC diagnostic warning "-pedantic"
#    endif
#  elif defined _MSC_VER
#    pragma warning(pop)
#  endif
#endif

#endif // PCL_IO_PLY_PLY_PARSER_H
