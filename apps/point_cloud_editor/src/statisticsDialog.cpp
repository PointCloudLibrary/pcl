///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///
///
/// @file   statisticsDialog.cpp
/// @details the class representing the dialog which accepts the parameters
/// to PCL's denoising filter.
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/statisticsDialog.h>

StatisticsDialog::StatisticsDialog(QWidget *)
{
  button_box_ = new QDialogButtonBox;
  button_box_->addButton(tr("Hide"), QDialogButtonBox::AcceptRole);
  connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));

  stat_label_ = new QLabel(tr(""));
  QVBoxLayout *main_layout_ = new QVBoxLayout;
  main_layout_ -> addWidget(stat_label_);
  main_layout_ -> addWidget(button_box_);
  setLayout(main_layout_);
  setWindowTitle(tr("Cloud Editor Statistics"));
  connect(&timer_, SIGNAL(timeout()), this, SLOT(update()));
  timer_.start(200);
}

StatisticsDialog::~StatisticsDialog ()
{
  delete button_box_;
  delete stat_label_;
}

void
StatisticsDialog::update ()
{
  stat_label_->setText(tr(Statistics::getStats().c_str()));
}

void
StatisticsDialog::accept ()
{
  this->done(0);
}

