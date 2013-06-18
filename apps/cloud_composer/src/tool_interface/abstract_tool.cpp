#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>

pcl::cloud_composer::AbstractTool::AbstractTool (PropertiesModel* parameter_model, QObject* parent) 
                      : QObject (parent) 
                      
{
  parameter_model_ = new PropertiesModel (this);
  //If there's a model copy it into the local copy
  if (parameter_model)
  {
    parameter_model_->copyProperties(parameter_model);
    
  }

    
}

QList <pcl::cloud_composer::CloudComposerItem*>
pcl::cloud_composer::AbstractTool::performAction (QList <const CloudComposerItem*>, PointTypeFlags::PointType)
{
  qDebug () << "AbstractTool::performTemplatedAction";
  return QList <CloudComposerItem*> ();
}
