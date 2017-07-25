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
/// @file   denoiseParameterForm.h
/// @details the class representing the dialog which accepts the parameters
/// to PCL's denoising filter.
/// @author  Yue Li and Matthew Hielsberg


#ifndef DENOISE_PARAMETER_FORM_H_
#define DENOISE_PARAMETER_FORM_H_

#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLineEdit>

class DenoiseParameterForm : public QDialog
{
  Q_OBJECT

  public:
    /// @brief Default Constructor
    DenoiseParameterForm();

    /// @brief Destructor
    ~DenoiseParameterForm ();

    /// @brief Returns the mean
    inline
    float
    getMeanK () const
    {
      return (mean_k_);
    }

    /// @brief Returns the standard deviation multiplier threshold
    inline
    float
    getStdDevThresh () const
    {
      return (std_dev_thresh_);
    }

    /// @brief Checks whether the OK button was pressed.
    inline
    bool
    ok () const
    {
      return (ok_);
    }

  private Q_SLOTS:
    /// @brief Accepts and stores the current user inputs, and turns off the
    /// dialog box.
    void
    accept ();

    /// @brief Rejects the current inputs, and turns off the dialog box.
    void
    reject ();

  private:
    /// The line for entering the mean
    QLineEdit *mean_K_line_;
    /// The line for entering the standard deviation multiplier threshold
    QLineEdit *std_dev_mul_thresh_line_;
    /// The button box.
    QDialogButtonBox *button_box_;
    /// The layout of the two input QLineEdit objects
    QFormLayout *layout_;
    /// The main layout for the dialog
    QVBoxLayout* main_layout_;
    /// The mean
    float mean_k_;
    /// The standard deviation multiplier threshold
    float std_dev_thresh_;
    /// The flag indicating whether the OK button was pressed
    bool ok_;
};

#endif // DENOISE_PARAMETER_FORM_H_
