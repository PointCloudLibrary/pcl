#include <QtGui>

#include <pcl/apps/cloud_composer/project_model.h>

pcl::cloud_composer::ProjectModel::ProjectModel (QObject* parent)
  : QStandardItemModel (parent)
{
  
}

pcl::cloud_composer::ProjectModel::ProjectModel (const ProjectModel& to_copy)
{
}

pcl::cloud_composer::ProjectModel::~ProjectModel ()
{

}
