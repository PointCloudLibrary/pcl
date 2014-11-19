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
#ifndef PCL_MODELER_PARAMETER_H_
#define PCL_MODELER_PARAMETER_H_

#include <map>
#include <string>

#include <boost/any.hpp>
#include <pcl/apps/modeler/qt.h>


namespace pcl
{
  namespace modeler
  {
    class Parameter
    {
      public:
        Parameter(const std::string& name, const std::string& description, const boost::any& value):
          name_(name), description_(description), default_value_(value), current_value_(value){}
        ~Parameter(void) {}

        const std::string&
        getName() const {return name_;}

        const std::string&
        getDescription()const {return description_;}

        void
        setDefaultValue(const boost::any& value)
        {
          default_value_ = value;
        }

        void
        reset() {current_value_ = default_value_;}

        virtual std::string
        valueTip() = 0;

        virtual QWidget*
        createEditor(QWidget *parent) = 0;

        virtual void
        setEditorData(QWidget *editor) = 0;

        virtual void
        setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index);

        virtual std::pair<QVariant, int>
        toModelData() = 0;

      protected:
        virtual void
        getEditorData(QWidget *editor) = 0;

        std::string     name_;
        std::string     description_;
        boost::any      default_value_;
        boost::any      current_value_;
    };

    class BoolParameter : public Parameter
    {
      public:
        BoolParameter(const std::string& name, const std::string& description, bool value):
            Parameter(name, description, value){}
        ~BoolParameter(){}

        operator bool() const {return boost::any_cast<bool>(current_value_);}

        virtual std::string
        valueTip();

        virtual QWidget*
        createEditor(QWidget *parent);

        virtual void
        setEditorData(QWidget *editor);

        virtual std::pair<QVariant, int>
        toModelData();

      protected:
        virtual void
        getEditorData(QWidget *editor);
    };

    class IntParameter : public Parameter
    {
      public:
        IntParameter(const std::string& name, const std::string& description, int value, int low, int high, int step=1):
          Parameter(name, description, value), low_(low), high_(high), step_(step){}
        virtual ~IntParameter(){}

        operator int() const {return boost::any_cast<int>(current_value_);}

        virtual std::string
        valueTip();

        virtual QWidget*
        createEditor(QWidget *parent);

        virtual void
        setEditorData(QWidget *editor);

        virtual std::pair<QVariant, int>
        toModelData();

        void
        setLow(int low)
        {
          low_ = low;
        }

        void
        setHigh(int high)
        {
          high_ = high;
        }

        void
        setStep(int step)
        {
          step_ = step;
        }

      protected:
        virtual void
        getEditorData(QWidget *editor);

        int     low_;
        int     high_;
        int     step_;
    };

    template <class T>
    class EnumParameter : public Parameter
    {
      public:
        EnumParameter(const std::string& name, const std::string& description, T value, const std::map<T, std::string>& candidates):
          Parameter(name, description, value), candidates_(candidates){}
        ~EnumParameter(){}

        operator T() const {return boost::any_cast<T>(current_value_);}

        virtual std::string
        valueTip();

        virtual QWidget*
        createEditor(QWidget *parent);

        virtual void
        setEditorData(QWidget *editor);

        virtual std::pair<QVariant, int>
        toModelData();

      protected:
        virtual void
        getEditorData(QWidget *editor);

        const std::map<T, std::string> candidates_;
    };

    class DoubleParameter : public Parameter
    {
      public:
        DoubleParameter(const std::string& name, const std::string& description, double value, double low, double high, double step=0.01):
          Parameter(name, description, value), low_(low), high_(high), step_(step){}
        virtual ~DoubleParameter(){}

        operator double() const {return boost::any_cast<double>(current_value_);}

        virtual std::string
        valueTip();

        virtual QWidget*
        createEditor(QWidget *parent);

        virtual void
        setEditorData(QWidget *editor);

        virtual std::pair<QVariant, int>
        toModelData();

        void
        setLow(double low)
        {
          low_ = low;
        }

        void
        setHigh(double high)
        {
          high_ = high;
        }

        void
        setStep(double step)
        {
          step_ = step;
        }

      protected:
        virtual void
        getEditorData(QWidget *editor);

        double  low_;
        double  high_;
        double  step_;
    };

    class ColorParameter : public Parameter
    {
      public:
        ColorParameter(const std::string& name, const std::string& description, QColor value):
          Parameter(name, description, value){}
        ~ColorParameter(){}

        operator QColor() const {return boost::any_cast<QColor>(current_value_);}

        virtual std::string
        valueTip();

        virtual QWidget*
        createEditor(QWidget *parent);

        virtual void
        setEditorData(QWidget *editor);

        virtual std::pair<QVariant, int>
        toModelData();

      protected:
        virtual void
        getEditorData(QWidget *editor);

    };
  }
}

#endif // PCL_MODELER_PARAMETER_H_
