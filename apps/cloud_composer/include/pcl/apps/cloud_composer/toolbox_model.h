/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2012, Jeremie Papon.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TOOLBOX_MODEL_H_
#define TOOLBOX_MODEL_H_

#include <QStandardItemModel>
#include <QItemSelectionModel>
#include <QVariant>

enum TOOLBOX_ROLES { 
  FACTORY = Qt::UserRole,
  PARAMETER_MODEL
};


namespace pcl
{
  namespace cloud_composer
  {
    class CloudCommand;
    class AbstractTool;
    class ToolFactory;
    
    class ToolBoxModel : public QStandardItemModel
    {
      Q_OBJECT
      
    public:
      ToolBoxModel (QObject *parent = 0);
      ToolBoxModel (const ToolBoxModel& to_copy);
      virtual ~ToolBoxModel ();
      
      void
      addTool (ToolFactory* tool_factory);
      
    public slots:

    signals:  

    private:
      QStandardItem* 
      addToolGroup (QString tool_group_name);
      
      
      
    };
  }
}

Q_DECLARE_METATYPE (pcl::cloud_composer::ToolBoxModel);


#endif //TOOLBOX_MODEL_H_

