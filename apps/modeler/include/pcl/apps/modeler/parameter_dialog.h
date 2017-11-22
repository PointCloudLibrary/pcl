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
#ifndef PCL_MODELER_PARAMETER_DIALOG_H_
#define PCL_MODELER_PARAMETER_DIALOG_H_

#include <pcl/apps/modeler/qt.h>

namespace pcl
{
  namespace modeler
  {
    class Parameter;

    class ParameterModel: public QStandardItemModel
    {
      public: 
        ParameterModel(QObject * parent = 0) : QStandardItemModel(parent){}
        ParameterModel(int rows, int columns, QObject * parent = 0) : QStandardItemModel(rows, columns, parent){}
        ~ParameterModel() {}

        Qt::ItemFlags
        flags ( const QModelIndex & index ) const
        {
          return (index.column() == 0)?(Qt::ItemIsEnabled | Qt::ItemIsSelectable):QStandardItemModel::flags(index);
        }
    };

    class ParameterDialog : public QDialog
    {
      Q_OBJECT
      public:
        ParameterDialog(const std::string& title, QWidget* parent=0);
        ~ParameterDialog(void){}

        void
        addParameter(Parameter* parameter);

        virtual int
        exec();

      protected:
        std::map<std::string, Parameter*>       name_parameter_map_;
        ParameterModel*                         parameter_model_;

      protected Q_SLOTS:
        void
        reset();
    };

    class ParameterDelegate : public QStyledItemDelegate
    {
      Q_OBJECT
      public:
        ParameterDelegate(std::map<std::string, Parameter*>& parameterMap, QObject *parent = 0);

        QWidget *
        createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;

        void
        setEditorData(QWidget *editor, const QModelIndex &index) const;

        void
        setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

        void
        updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;

      protected:
        void
        initStyleOption(QStyleOptionViewItem *option, const QModelIndex &index) const;

      private:
        Parameter*
        getCurrentParameter(const QModelIndex &index) const;

        std::map<std::string, Parameter*>&      parameter_map_;
    };

  }
}

#endif // PCL_MODELER_PARAMETER_DIALOG_H_
