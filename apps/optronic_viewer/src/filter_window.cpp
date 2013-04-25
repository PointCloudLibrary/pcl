/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/apps/optronic_viewer/filter_window.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/fotonic_grabber.h>

#include <fz_api.h>
#include <fz_internal.h>

#include <QFileInfo>
#include <vtkActor.h>
#include <vtkRenderer.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
FilterWindow::
FilterWindow (std::vector<CloudFilterFactory*> & filter_factories, std::vector<CloudFilter*> & filter_list)
: filter_factories_ (filter_factories)
, filter_selection_combo_box_ (NULL)
, filter_list_ (&filter_list)
, filter_ (NULL)
{
  this->setOption (QWizard::IndependentPages, true);

  createFilterSelectionPage ();

  filter_ = filter_factories_[0]->create ();

  QString name (filter_factories_[0]->getName ().c_str ());
  filter_name_line_edit_->setText (name);

  if (filter_factories.size () > 0)
    last_added_page_id_ = this->addPage (filter_->getParameterPage ());

  connect (this->button (FinishButton), SIGNAL (released ()),this, SLOT (finished ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
FilterWindow::
~FilterWindow ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FilterWindow::
itemSelected (int id)
{
  std::cerr << "item " << id << " selected" << std::endl;

  delete filter_;
  filter_ = filter_factories_[id]->create ();

  QString name (filter_factories_[id]->getName ().c_str ());
  filter_name_line_edit_->setText (name);

  int tmp = this->addPage (filter_->getParameterPage ());
  this->removePage (last_added_page_id_);
  last_added_page_id_ = tmp;
  this->update ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FilterWindow::
finished ()
{
  std::cerr << "finished" << std::endl;

  filter_->setName (filter_name_line_edit_->text ().toStdString ());
  filter_list_->push_back (filter_);

  emit filterCreated ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FilterWindow::
next ()
{
  std::cerr << "next" << std::endl;
  this->QWizard::next ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FilterWindow::
createFilterSelectionPage ()
{
  QWizardPage * filter_selection_page = new QWizardPage ();

  QLabel * select_filter_label = new QLabel (tr ("Select Filter:"));
  filter_selection_combo_box_ = new QComboBox ();

  if (filter_factories_.empty ())
    filter_selection_combo_box_->addItem (tr ("none"));
  else
    fillFilterSelectionComboBox (filter_selection_combo_box_);

  connect (filter_selection_combo_box_, SIGNAL (currentIndexChanged (int)),this, SLOT (itemSelected (int)));

  QLabel * filter_name_label = new QLabel (tr ("Filter Name:"));
  filter_name_line_edit_ = new QLineEdit ();

  QVBoxLayout * main_layout = new QVBoxLayout (filter_selection_page);
  main_layout->addWidget (select_filter_label);
  main_layout->addWidget (filter_selection_combo_box_);
  main_layout->addWidget (filter_name_label);
  main_layout->addWidget (filter_name_line_edit_);

  this->addPage (filter_selection_page);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
FilterWindow::
fillFilterSelectionComboBox (QComboBox * combo_box)
{
  for (int factory_index = 0; factory_index < filter_factories_.size (); ++factory_index)
  {
    std::string name = filter_factories_[factory_index]->getName ();

    combo_box->addItem (tr (name.c_str ()));
  }
}
