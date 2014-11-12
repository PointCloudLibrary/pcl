/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#include <pcl/apps/cloud_composer/items/fpfh_item.h>
#include <pcl/apps/cloud_composer/qt.h>
#include <vtkRenderWindow.h>

pcl::cloud_composer::FPFHItem::FPFHItem (QString name, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_ptr, double radius)
  : CloudComposerItem (name)
  , fpfh_ptr_ (fpfh_ptr)
  , radius_ (radius)

{
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr fpfh_const = fpfh_ptr;
  this->setData (QVariant::fromValue (fpfh_const), ItemDataRole::CLOUD_TEMPLATED);
  properties_->addCategory ("Core Properties");
  properties_->addProperty ("Radius", QVariant (radius_), Qt::ItemIsEnabled, "Core Properties");
  
  
}

pcl::cloud_composer::FPFHItem*

pcl::cloud_composer::FPFHItem::clone () const
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_copy (new pcl::PointCloud<pcl::FPFHSignature33> (*fpfh_ptr_));
  FPFHItem* new_item = new FPFHItem (this->text (), fpfh_copy, radius_);
  
  PropertiesModel* new_item_properties = new_item->getPropertiesModel ();
  new_item_properties->copyProperties (properties_);
  
  return new_item;  
}

pcl::cloud_composer::FPFHItem::~FPFHItem ()
{
  
}

QMap <QString, QWidget*>
pcl::cloud_composer::FPFHItem::getInspectorTabs ()
{
 
  
  //Create the plotter and QVTKWidget if it doesnt exist
  if (!plot_)
  {
    plot_ = boost::shared_ptr<pcl::visualization::PCLPlotter> (new pcl::visualization::PCLPlotter);
    qvtk_ = new QVTKWidget ();
    hist_page_ = new QWidget ();
    QGridLayout *mainLayout = new QGridLayout (hist_page_);
    mainLayout-> addWidget (qvtk_,0,0);
  }

  //Plot the histogram
  plot_->addFeatureHistogram (*fpfh_ptr_, fpfh_ptr_->width, data(ItemDataRole::ITEM_ID).toString().toStdString ());
  //Set the render window of the QVTK widget, update
  plot_->setViewInteractor (vtkSmartPointer<vtkRenderWindowInteractor> (qvtk_->GetInteractor ()));
  qvtk_->SetRenderWindow (plot_->getRenderWindow ());
  qvtk_->show ();
  qvtk_->update ();
  
  QMap <QString, QWidget*> tabs;
  tabs.insert ("Histogram",hist_page_);
  return tabs;
}


