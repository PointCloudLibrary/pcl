#include <pcl/apps/cloud_composer/item_inspector.h>



pcl::cloud_composer::ItemInspector::ItemInspector (QWidget* parent)
  : QWidget(parent)
{
  
}

pcl::cloud_composer::ItemInspector::~ItemInspector ()
{
  
}


void
pcl::cloud_composer::ItemInspector::setModel (ProjectModel* new_model)
{
  current_model_ = new_model;
  
}