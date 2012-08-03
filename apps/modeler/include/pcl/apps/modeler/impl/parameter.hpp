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
*
*/
#ifndef PCL_MODELER_PARAMETER_IMPL_H_
#define PCL_MODELER_PARAMETER_IMPL_H_

#include <pcl/apps/modeler/parameter.h>


namespace pcl
{
  namespace modeler
  {
//////////////////////////////////////////////////////////////////////////////////////////////
    template <class T> std::string
    EnumParameter<T>::valueTip()
    {
      std::string tip("possible values: {");
      typename std::map<T, std::string>::const_iterator it = candidates_.begin();
      do
      {
        tip += it->second;
        ++ it;
        if (it != candidates_.end())
          tip += ", ";
      } while(it != candidates_.end());
      tip += "}";

      return (tip);
    }

//////////////////////////////////////////////////////////////////////////////////////////////
    template <class T> QWidget *
    EnumParameter<T>::createEditor(QWidget *parent)
    {
      QComboBox* editor = new QComboBox(parent);
      for (typename std::map<T, std::string>::const_iterator it = candidates_.begin();
        it != candidates_.end();
        ++ it)
      {
          editor->addItem(it->second.c_str());
      }

      return (editor);
    }

//////////////////////////////////////////////////////////////////////////////////////////////
    template <class T> void
    EnumParameter<T>::setEditorData(QWidget *editor)
    {
      QComboBox *comboBox = static_cast<QComboBox*>(editor);

      T value = T (*this);
      comboBox->setCurrentIndex(value);
    }

//////////////////////////////////////////////////////////////////////////////////////////////
    template <class T> void
    EnumParameter<T>::getEditorData(QWidget *editor)
    {
      QComboBox *comboBox = static_cast<QComboBox*>(editor);
      T value = T (comboBox->currentIndex());
      current_value_ = value;
    }

//////////////////////////////////////////////////////////////////////////////////////////////
    template <class T> std::pair<QVariant, int>
    EnumParameter<T>::toModelData()
    {
      std::pair<QVariant, int> model_data;
      for (typename std::map<T, std::string>::const_iterator it = candidates_.begin();
        it != candidates_.end();
        ++ it) 
      {
        if (it->first == value)
        {
          model_data.first = it->second;
          break;
        }
      }
      model_data.second = Qt::EditRole;

      return (model_data);
    }
  }
}

#endif // PCL_MODELER_PARAMETER_IMPL_H_
