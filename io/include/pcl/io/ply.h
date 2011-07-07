/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_IO_PLY_H_
#define PCL_IO_PLY_H_

#include "pcl/point_types.h"
#include "pcl/io/io.h"
#include <functional>
#include <algorithm>

namespace pcl
{
  namespace io
  {
    namespace ply
    {
      ///available PLY formats
      enum Format
      {
        ASCII_FORMAT = 0,
        BIG_ENDIAN_FORMAT = 1,
        LITTLE_ENDIAN_FORMAT = 2
      };

#if (defined(__powerpc) || defined(__powerpc__) || defined(__POWERPC__) || defined(__ppc__) || defined(_M_PPC) || defined(__ARCH_PPC))
#  define PCL_BIG_ENDIAN
#elif (defined(i386) || defined(__i386__) || defined(__i386) || defined(_M_IX86) || defined(_X86_) || defined(__THW_INTEL__) || defined(__I86__) || defined(__INTEL__)) \
  || (defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(__x86_64) || defined(_M_X64))
#  define PCL_LITTLE_ENDIAN
#else
#  error
#endif
      ///@return endianess of the host machine
      Format getEndianess()
      {
        #ifdef PCL_LITTLE_ENDIAN
        return LITTLE_ENDIAN_FORMAT;
        #elif defined PCL_BIG_ENDIAN
        return BIG_ENDIAN_FORMAT;
        #else
        #error
        #endif
      }
#undef PCL_BIG_ENDIAN
#undef PCL_LITTLE_ENDIAN
      
      ///PLY file available flags
      enum Flags
      {
        NONE              = 0x0000000,
        //vertex properties
        VERTEX_XYZ        = 0x0000001,
        VERTEX_NORMAL     = 0x0000002,
        VERTEX_COLOR      = 0x0000004,
        VERTEX_INTENSITY  = 0x0000008,
        VERTEX_NORNAL     = 0x0000010,
        VERTEX_RADIUS     = 0x0000020,
        VERTEX_CONFIDENCE = 0x0000040,
        VERTEX_VIEWPOINT  = 0x0000080,
        VERTEX_RANGE      = 0x0000100,
        VERTEX_STRENGTH   = 0x0000200,
        VERTEX_XY         = 0x0000400,
        //face properties
        FACE_VERTICES     = 0x0010000,
        //camera properties
        CAMERA            = 0x8000000,
        //all properties are set
        ALL               = 0xFFFFFFF
      };
      ///@return a property type from its type name
      int getTypeFromTypeName(const std::string& type_name) 
      {
        if(!strcmp(type_name.c_str(), "char"))
          return sensor_msgs::PointField::INT8;
        if(!strcmp(type_name.c_str(), "uchar"))
          return sensor_msgs::PointField::UINT8;
        if(!strcmp(type_name.c_str(), "short"))
          return sensor_msgs::PointField::INT16;
        if(!strcmp(type_name.c_str(), "ushort"))
          return sensor_msgs::PointField::UINT16;
        if(!strcmp(type_name.c_str(), "int"))
          return sensor_msgs::PointField::INT32;
        if(!strcmp(type_name.c_str(), "uint"))
          return sensor_msgs::PointField::UINT32;
        if(!strcmp(type_name.c_str(), "float"))
          return sensor_msgs::PointField::FLOAT32;
        if(!strcmp(type_name.c_str(), "double"))
          return sensor_msgs::PointField::FLOAT64;
        if(!strcmp(type_name.c_str(), "list"))
          return -1;
        return -2;
      };
      
      size_t getMaximumCapacity(int size_type)
      {
        switch(size_type)
        {
          case sensor_msgs::PointField::UINT8 : 
            return std::numeric_limits<unsigned char>::max();
          case sensor_msgs::PointField::UINT16 : 
            return std::numeric_limits<unsigned short>::max();
        case sensor_msgs::PointField::UINT32 : 
            return std::numeric_limits<unsigned int>::max();
          default:
            return 0;
        }
      };

      const size_t CAMERA_SIZE = sizeof(float) * 19 + sizeof(int) * 2;

      struct property
      {
        std::string name_;
        int data_type_;
        size_t offset_;
        property(const std::string& name) : name_(name), offset_(0) {}
        property(const std::string& name, int data_type) : 
          name_(name), data_type_(data_type)
        {
          offset_ = pcl::getFieldSize(data_type_);
        }
      };

      struct list_property : public property
      {
        int size_type_;
        list_property(const std::string& name, int size_type, int data_type) : 
        property(name, data_type), size_type_(size_type) 
        {
          offset_ = pcl::getFieldSize(size_type_) + 
                    getMaximumCapacity(size_type_) * pcl::getFieldSize(data_type_);
        }
        
        inline void set_size(const void* size)
        {
          offset_ = pcl::getFieldSize(size_type_);
          switch(size_type_)
          {
            case sensor_msgs::PointField::UINT8 : 
            {
              const unsigned char *size_; size_ = (unsigned char*) size;
              offset_ += (*size_) * pcl::getFieldSize(size_type_);
            }
            break;
            case sensor_msgs::PointField::UINT16 : 
            {
              const unsigned short *size_; size_ = (unsigned short*) size;
              offset_ += (*size_) * pcl::getFieldSize(size_type_);
            }
            break;
            case sensor_msgs::PointField::UINT32 : 
            {
              const unsigned int *size_; size_ = (unsigned int*) size;
              offset_ += (*size_) * pcl::getFieldSize(size_type_);
            }
            break;
          }
        }
      };

      class element
      {
        public:
          std::string name_;
          size_t count_;
          size_t offset_;
          element(const std::string& name, size_t count) : 
          name_(name), count_(count), offset_(0), properties_(0), list_properties_(0) {}

          typedef std::vector<property*>::iterator iterator;
          typedef std::vector<property*>::const_iterator const_iterator;
          typedef std::vector<iterator>::iterator iterator_iterator;
          typedef std::vector<iterator>::const_iterator const_iterator_iterator;
          inline size_t properties_size() { return properties_.size(); }
          inline property* operator[](const std::string &prop_name)
          {
            std::vector<property*>::iterator properties_it = properties_.begin();
            for(; properties_it != properties_.end(); ++properties_it)
              if((*properties_it)->name_ == prop_name) break;
            if (properties_it == properties_.end())
              return NULL;
            else
              return *properties_it;
          }
          
          inline const property* operator[](const std::string &prop_name) const 
          {
            std::vector<property*>::const_iterator properties_it = properties_.begin ();
            for(; properties_it != properties_.end (); ++properties_it)
              if((*properties_it)->name_ == prop_name) break;
            if (properties_it == properties_.end ())
              return NULL;
            else
              return *properties_it;
          }

          inline int push_property(const std::string& name, int data_type)
          {
            property* p = new property (name, data_type);
            properties_.push_back (p);
            offset_+= p->offset_;
            return int (properties_.size ());
          }
          
          inline int push_property(const std::string& name, int size_type, int data_type)
          {
            property *lp = new list_property (name, size_type, data_type);
            properties_.push_back (lp);
            list_properties_.push_back (properties_.end() - 1);
            offset_+= lp->offset_;
            return int (properties_.size ());
          }

          inline bool has_list_properties() const { return (!list_properties_.empty()); }
          
          size_t offset_before(const std::string& prop_name)
          {
            size_t offset = 0;
            std::vector<property*>::const_iterator properties_it = properties_.begin();
            for(; properties_it != properties_.end(); ++properties_it)
              if((*properties_it)->name_ == prop_name) 
                break;
              else
                offset+= (*properties_it)->offset_;
            if (properties_it == properties_.end())
              return -1;
            return offset;
          }

          inline void update_offset() 
          {
            offset_ = 0;
            std::vector<property*>::const_iterator properties_it = properties_.begin();
            for(; properties_it != properties_.end(); ++properties_it)
              offset_+= (*properties_it)->offset_;
          }

          inline bool is_list_property(const_iterator property_pos)
          {
            return std::find_if(list_properties_.begin(),
                                list_properties_.end(),
                                std::bind1st(std::equal_to<const_iterator>(),property_pos)) != list_properties_.end();
          }

          std::vector<property*> properties_;
          std::vector<iterator> list_properties_;
      };

      class parser
      {
        public:
          parser() : elements_(0), last_element_(0) {}

          typedef std::vector<element*>::iterator iterator;
          typedef std::vector<element*>::const_iterator const_iterator;
          inline iterator begin() { return elements_.begin(); }
          inline iterator end() { return elements_.end(); }

          inline element* operator[](const std::string &element_name)
          {
            std::vector<element*>::iterator elements_it = elements_.begin();
            for(; elements_it != elements_.end(); ++elements_it)
              if((*elements_it)->name_ == element_name) break;
            if (elements_it == elements_.end())
              return NULL;
            else
              return *elements_it;
          }

          inline const element* operator[](const std::string &element_name) const
          {
            std::vector<element*>::const_iterator elements_it = elements_.begin();
            for(; elements_it != elements_.end(); ++elements_it)
              if((*elements_it)->name_ == element_name) break;
            if (elements_it == elements_.end())
              return NULL;
            else
              return *elements_it;
          }
          
          inline int push_element(const std::string& name, size_t count)
          {
            last_element_ = new element(name, count);
            elements_.push_back(last_element_);
            return int (elements_.size());
          }

          inline int push_property(const std::string& name, int data_type)
          {
            return last_element_->push_property(name, data_type);
          }

          inline int push_property(const std::string& name, int size_type, int data_type)
          {
            return last_element_->push_property(name, size_type, data_type);
          }

          size_t offset_before(const std::string& element_name)
          {
            size_t offset = 0;
            std::vector<element*>::const_iterator elements_it = elements_.begin();
            for(; elements_it != elements_.end(); ++elements_it)
              if((*elements_it)->name_ == element_name) 
                break;
              else
                offset+= (*elements_it)->offset_ * (*elements_it)->count_;
            if (elements_it == this->end())
              return -1;
            return offset;
          }

        private:
          std::vector<element*> elements_;
          element* last_element_;
      };

      /** Wrapper for PLY camera structure to ease read/write */
      struct camera
      {
        float view_px; float view_py; float view_pz;
        float x_axisx; float x_axisy; float x_axisz;
        float y_axisx; float y_axisy; float y_axisz;
        float z_axisx; float z_axisy; float z_axisz;
        float focal; float scalex; float scaley; float centerx; float centery;
        int viewportx; int viewporty; float k1; float k2;
        /**constructor 
          *\param origin: sensor origin to store in view_p[x,y,z]
          *\param orientation: sensor orientation to store in [x,y,z]_axis[x,y,z]
          *\remark instrinsics are set to 0 cause they are unused
          */
        camera() :
          view_px(0), view_py(0), view_pz(0),
          x_axisx(0), x_axisy(0), x_axisz(0),
          y_axisx(0), y_axisy(0), y_axisz(0),
          z_axisx(0), z_axisy(0), z_axisz(0),
          focal(0), scalex(0), centerx(0), centery(0), viewportx(0), viewporty(0),
            k1(0), k2(0) {}

        camera(const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation) :
          view_px(origin[0]), view_py(origin[1]), view_pz(origin[2]),
          focal(0), scalex(0), centerx(0), centery(0), viewportx(0), viewporty(0),
          k1(0), k2(0)
        {
          Eigen::Matrix3f R = orientation.toRotationMatrix ();
          x_axisx = R(0,0); x_axisy = R(0,1); x_axisz = R(0,2);
          y_axisx = R(1,0); y_axisy = R(1,1); y_axisz = R(1,2);
          z_axisx = R(2,0); z_axisy = R(2,1); z_axisz = R(2,2);
        }
        
        /** converts extrinsics camera parameters to eigen structures */
        void ext_to_eigen(Eigen::Vector4f &origin, Eigen::Quaternionf &orientation)
        {
          origin[0] = view_px; origin[1] = view_py;  origin[2] = view_pz;
          Eigen::Matrix3f R;
          R << x_axisx, x_axisy, x_axisz,
               y_axisx, y_axisy, y_axisz,
               z_axisx, z_axisy, z_axisz;
          orientation = Eigen::Quaternionf(R);
        }
      };
      
      /** write out a pcl::io::ply::camera structure to an ostream */
      void write(const pcl::io::ply::camera& c, std::ostream& out, bool binary)
      {
        if(!binary)
        {
          out << c.view_px << " " << c.view_py << " " << c.view_pz << " ";
          out << c.x_axisx << " " << c.x_axisy << " " << c.x_axisz << " ";
          out << c.y_axisx << " " << c.y_axisy << " " << c.y_axisz << " ";
          out << c.z_axisx << " " << c.z_axisy << " " << c.z_axisz << " ";
          out << c.focal << " ";
          out << c.scalex << " " << c.scaley << " ";
          out << c.centerx << " " << c.centery << " ";
          out << c.viewportx << " " << c.viewporty << " ";
          out << c.k1 << " " << c.k2;
        }
        else {
          out.write ((const char*) &c, sizeof(camera));
        }
      };
    } //namespace ply
  } //namespace io
}//namespace pcl

#endif
