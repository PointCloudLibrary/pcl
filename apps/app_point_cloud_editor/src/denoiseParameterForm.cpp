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
/// @file   denoiseParameterForm.cpp
/// @details the class representing the dialog which accepts the parameters
/// to PCL's denoising filter.
/// @author  Yue Li and Matthew Hielsberg

#include <pcl/apps/point_cloud_editor/denoiseParameterForm.h>

DenoiseParameterForm::DenoiseParameterForm () : ok_(false)
{
  mean_K_line_ = new QLineEdit;
  std_dev_mul_thresh_line_ = new QLineEdit;
  button_box_ = new QDialogButtonBox;
  button_box_->addButton(tr("Cancel"),
                         QDialogButtonBox::RejectRole);
  button_box_->addButton(tr("OK"),
                         QDialogButtonBox::AcceptRole);
  connect(button_box_, SIGNAL(accepted()),
          this, SLOT(accept()));
  connect(button_box_, SIGNAL(rejected()),
          this, SLOT(reject()));
  layout_ = new QFormLayout;
  layout_->addRow(tr("&MeanK:"), mean_K_line_);
  layout_->addRow(tr("&Standard deviation threshold:"),
                 std_dev_mul_thresh_line_);

  main_layout_ = new QVBoxLayout;
  main_layout_->addLayout(layout_);
  main_layout_->addWidget(button_box_);
  setLayout(main_layout_);
  setWindowTitle(tr("Denoise Filter"));
}

DenoiseParameterForm::~DenoiseParameterForm ()
{
  delete mean_K_line_;
  delete std_dev_mul_thresh_line_;
  delete button_box_;
  delete layout_;
  delete main_layout_;
}

void
DenoiseParameterForm::accept ()
{
  QString mean_str = mean_K_line_->text();
  bool ok;
  mean_k_ = mean_str.toFloat(&ok);
  // validates the input.
  if (!ok)
  {
    ok_ = false;
    return;
  }
  QString std_dev_str = std_dev_mul_thresh_line_->text();
  std_dev_thresh_ = std_dev_str.toFloat(&ok);
  if (!ok)
  {
    ok_ = false;
    return;
  }
  this->done(0);
  ok_ = true;
}

void
DenoiseParameterForm::reject ()
{
  ok_ = false;
  this->done(0);
}

