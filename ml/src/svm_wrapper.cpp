 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2012, Willow Garage, Inc.
  *  Copyright (c) 2000-2012 Chih-Chung Chang and Chih-Jen Lin
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
  *   * Neither the name of copyright holders nor the names of its
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


#ifndef PCL_SVM_WRAPPER_HPP_
#define PCL_SVM_WRAPPER_HPP_

#include <pcl/ml/svm_wrapper.h>
#include <assert.h>
#include <fstream>


template <typename T>
inline T module (T a)
{
  if (a > 0)
    return a;
  else
    return -a;
}

char*
pcl::SVM::readline (FILE *input)
{
  int len;

  if (fgets (line_, max_line_len_, input) == NULL)
    return NULL;

  // Find the endline. If not found extend the max_line_len_
  while (strrchr (line_ , '\n') == NULL)
  {
    max_line_len_ *= 2;
    line_ = static_cast<char *> (realloc (line_, max_line_len_));
    len = int (strlen (line_));

    // if the new read part of the string is unavailable, break the while
    if (fgets (line_ + len, max_line_len_ - len, input) == NULL)
      break;
  }

  return line_;
}

void
pcl::SVMTrain::doCrossValidation ()
{
  int i;
  int total_correct = 0;
  double total_error = 0;
  double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
  double *target = Malloc (double, prob_.l);

  // number of fold for the cross validation (n of folds = number of splitting of the input dataset)
  if (nr_fold_ < 2)
  {
    fprintf (stderr, "n-fold cross validation: n must >= 2\n");
    return;
  }

  svm_cross_validation (&prob_, &param_, nr_fold_, target); // perform cross validation

  if (param_.svm_type == EPSILON_SVR ||
      param_.svm_type == NU_SVR)
  {
    for (i = 0;i < prob_.l;i++)
    {
      double y = prob_.y[i];
      double v = target[i];
      total_error += (v - y) * (v - y);
      sumv += v;
      sumy += y;
      sumvv += v * v;
      sumyy += y * y;
      sumvy += v * y;
    }

    pcl::console::print_info (" - Cross Validation Mean squared error = ");
    pcl::console::print_value ("%g\n", total_error / prob_.l);

    pcl::console::print_info (" - Cross Validation Squared correlation coefficient = ");
    pcl::console::print_value ("%g\n",
                               ( (prob_.l*sumvy - sumv*sumy) * (prob_.l*sumvy - sumv*sumy)) /
                               ( (prob_.l*sumvv - sumv*sumv) * (prob_.l*sumyy - sumy*sumy))
                              );
  }
  else
  {
    for (i = 0;i < prob_.l;i++)
      if (target[i] == prob_.y[i])
        ++total_correct;

    pcl::console::print_info (" - Cross Validation Accuracy = ");
    pcl::console::print_value ("%g%%\n", 100.0*total_correct / prob_.l);
  }

  free (target);
}

void
pcl::SVMTrain::scaleFactors (std::vector<SVMData> training_set, svm_scaling &scaling)
{
  int max = 0;

  for (size_t i = 0; i < training_set.size() ; i++)
    for (size_t j = 0; j < training_set[i].SV.size() ; j++)
      if (training_set[i].SV[j].idx > max)
        max = training_set[i].SV[j].idx; // max number of features

  max += 1;

  scaling.obj = Malloc (struct svm_node, max + 1);
  scaling.max = max;
  scaling.obj[max].index = -1; // last index is -1

  for (int i = 0; i < max; i++) // Initialize values
  {
    scaling.obj[i].index = 0;
    scaling.obj[i].value = 0;
  }

  for (size_t i = 0; i < training_set.size(); i++)
    for (size_t j = 0; j < training_set[i].SV.size(); j++)
      // save scaling factor finding the maximum value
      if (module (training_set[i].SV[j].value) > scaling.obj[ training_set[i].SV[j].idx ].value)
      {
        scaling.obj[ training_set[i].SV[j].idx ].index = 1;
        scaling.obj[ training_set[i].SV[j].idx ].value = module (training_set[i].SV[j].value);
      }
};

void
pcl::SVM::adaptLibSVMToInput (std::vector<SVMData> &training_set, svm_problem prob)
{
  training_set.clear (); // Reset input

  for (int i = 0; i < prob.l; i++)
  {
    SVMData parent; // buffer data container
    int j = 0;

    if (labelled_training_set_)
      parent.label = prob.y[i];

    while (prob.x[i][j].index != -1) // -1 means the end of problem entry list
    {
      SVMDataPoint seed; // single feature seed

      if (pcl_isfinite (prob.x[i][j].value))
      {
        seed.idx = prob.x[i][j].index;
        seed.value = float (prob.x[i][j].value);
        parent.SV.push_back (seed);
      }

      j++;
    }

    training_set.push_back (parent);
  }
};

void
pcl::SVM::adaptInputToLibSVM (std::vector<SVMData> training_set, svm_problem &prob)
{
  assert (training_set.size() > 0);
  assert (scaling_.max != 0);

  if (scaling_.max == 0)
  {
    // to be sure to have loaded the scaling
    PCL_ERROR ("[pcl::%s::adaptInputToLibSVM] Classifier model not loaded!\n", getClassName ().c_str ());
    return;
  }

  prob.l = int (training_set.size ()); // n of elements/points
  prob.y = Malloc (double, prob.l);
  prob.x = Malloc (struct svm_node *, prob.l);

  for (int i = 0; i < prob.l; i++)
  {
    if (pcl_isfinite (training_set[i].label) && labelled_training_set_)
    {
      prob.y[i] = training_set[i].label;
      labelled_training_set_ = 1;
    }
    else
      labelled_training_set_ = 0;

    prob.x[i] = Malloc (struct svm_node, training_set[i].SV.size() + 1);

    int k = 0;
    
    for (size_t j = 0; j < training_set[i].SV.size(); j++)
      if (training_set[i].SV[j].idx != -1 && pcl_isfinite (training_set[i].SV[j].value))
      {
        prob.x[i][k].index = training_set[i].SV[j].idx;
        if (training_set[i].SV[j].idx < scaling_.max && scaling_.obj[ training_set[i].SV[j].idx ].index == 1)
          prob.x[i][k].value = training_set[i].SV[j].value / scaling_.obj[ training_set[i].SV[j].idx ].value;
        else
          prob.x[i][k].value = training_set[i].SV[j].value;
        k++;
      }
      
    prob.x[i][k].index = -1;
  }
};

bool
pcl::SVMTrain::trainClassifier ()
{
  if (training_set_.size() == 0)
  {
    // to be sure to have loaded the training set
    PCL_ERROR ("[pcl::%s::trainClassifier] Training data not set!\n", getClassName ().c_str ());
    return 0;
  }
  
  scaleFactors (training_set_, scaling_);
  adaptInputToLibSVM (training_set_, prob_);

  const char *error_msg;
  error_msg = svm_check_parameter (&prob_, &param_);

  // initialize gamma parameter

  if (param_.gamma == 0 && scaling_.max > 0)
    param_.gamma = 1.0 / scaling_.max;

  if (error_msg)
  {
    PCL_ERROR ("[pcl::%s::trainClassifier] %s\n", getClassName ().c_str (), error_msg);
    //fprintf (stderr, "ERROR: %s\n", error_msg);
    exit (1);
  }

  if (cross_validation_)
  {
    doCrossValidation ();
  }
  else
  {
    SVMModel* out;
    out = static_cast<SVMModel*> (svm_train (&prob_, &param_));
    if (out == NULL)
    {
      PCL_ERROR ("[pcl::%s::trainClassifier] Error taining the classifier model.\n", getClassName ().c_str ());
      return 0;
    }
    model_ = *out;
    model_.scaling = scaling_.obj;
    free (out);
  }

  return 1;
}

bool
pcl::SVM::loadProblem (const char *filename, svm_problem &prob)
{
  int elements, max_index, inst_max_index, i, j;
  FILE *fp = fopen (filename, "r");
  svm_node *x_space_;
  char *endptr;
  char *idx, *val, *label;

  if (fp == NULL)
  {
    PCL_ERROR ("[pcl::%s] Can't open input file %s.\n", getClassName ().c_str (), filename);
    return 0;
  }

  elements = 0;
  prob.l = 0;

  line_ = Malloc (char, max_line_len_);
  
  // readline function writes one line in var. "line_"
  while (readline (fp) != NULL)
  {
    // "\t" cuts the tab or space.
    // strtok splits the string into tokens
    char *p = strtok (line_, " \t"); // label
    ++elements;
    // features

    while (1)
    {
      // split the next element
      p = strtok (NULL, " \t");

      if (p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
        break;

      ++elements;
    }
    ++elements; // contains the number of elements in the string
    ++prob.l; // number op
  }

  rewind (fp); // returns to the top pos of fp

  prob.y = Malloc (double, prob.l);
  prob.x = Malloc (struct svm_node *, prob.l);
  x_space_ = Malloc (struct svm_node, elements);

  max_index = 0;
  j = 0;
  bool isUnlabelled = 0;

  for (i = 0;i < prob.l;i++)
  {
    inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
    // read one line in the file
    readline (fp);
    prob.x[i] = &x_space_[j];

    if (!isUnlabelled)
    {
      label = strtok (line_, " \t\n"); //save first element as label
      char * pch;
      pch = strpbrk (label, ":");
      // std::cout << label << std::endl;

      // check if the first element is really a label

      if (pch == NULL)
      {
        if (label == NULL) // empty line
          exitInputError (i + 1);

        labelled_training_set_ = 1;

        prob.y[i] = strtod (label, &endptr);

        if (endptr == label || *endptr != '\0')
          exitInputError (i + 1);

        //idx = strtok(NULL,":"); // indice
      }
      else
      {
        isUnlabelled = 1;
        labelled_training_set_ = 0;
        i = -1;
        rewind (fp);
        continue;
      }
    } //else

//    idx=strtok(line,": \t\n");
    int k = 0;

    while (1)
    {
      if (k++ == 0 && isUnlabelled)
        idx = strtok (line_, ": \t\n");
      else
        idx = strtok (NULL, ":"); // indice

      val = strtok (NULL, " \t"); // valore

      if (val == NULL)
        break; // exit with the last element

      //std::cout << idx << ":" << val<< " ";
      errno = 0;

      x_space_[j].index = int (strtol (idx, &endptr, 10));

      if (endptr == idx || errno != 0 || *endptr != '\0' || x_space_[j].index <= inst_max_index)
        exitInputError (i + 1);
      else
        inst_max_index = x_space_[j].index;

      errno = 0;

      x_space_[j].value = strtod (val, &endptr);

      if (endptr == val || errno != 0 || (*endptr != '\0' && !isspace (*endptr)))
        exitInputError (i + 1);

      ++j;

    }

    //std::cout <<"\n";
    if (inst_max_index > max_index)
      max_index = inst_max_index;

    x_space_[j++].index = -1;
  }



  if (param_.gamma == 0 && max_index > 0)
    param_.gamma = 1.0 / max_index;

  if (param_.kernel_type == PRECOMPUTED)
    for (i = 0;i < prob.l;i++)
    {
      if (prob.x[i][0].index != 0)
      {
	PCL_ERROR ("[pcl::%s] Wrong input format: first column must be 0:sample_serial_number.\n", getClassName ().c_str () );
        return 0;
      }

      if (int (prob.x[i][0].value) <= 0 || int (prob.x[i][0].value) > max_index)
      {
	PCL_ERROR ("[pcl::%s] Wrong input format: sample_serial_number out of range.\n", getClassName ().c_str () );
        return 0;
      }
    }
  fclose (fp);

  return 1;
}


bool
pcl::SVM::saveProblem (const char *filename, bool labelled = 0)
{
  assert (training_set_.size() > 0);
  std::ofstream myfile;
  myfile.open (filename);

  if (!myfile.is_open())
  {
    PCL_ERROR ("[pcl::%s] Can't open/create file %s.\n", getClassName ().c_str (), filename);
    return 0;
  }
    

  for (size_t j = 0; j < training_set_.size() ; j++)
  {

    if (labelled)
    {
      assert (pcl_isfinite (training_set_[j].label));
      myfile << training_set_[j].label << " ";
    }

    for (size_t i = 0; i < training_set_[j].SV.size(); i++)
      if (pcl_isfinite (training_set_[j].SV[i].value))
        myfile << training_set_[j].SV[i].idx << ":" << training_set_[j].SV[i].value << " ";

    myfile << "\n";
  }

  myfile.close();

  //std::cout << " * " << filename << " saved" << std::endl;
  return 1;
}

bool
pcl::SVM::saveProblemNorm (const char *filename, svm_problem prob_, bool labelled = 0)
{
  if (prob_.l == 0)
  {
    PCL_ERROR ("[pcl::%s] Can't save file %s. Input data not set.\n", getClassName ().c_str (), filename);
    return 0;
  }

  std::ofstream myfile;

  myfile.open (filename);

  if (!myfile.is_open())
  {
    PCL_ERROR ("[pcl::%s] Can't open/create file %s.\n", getClassName ().c_str (), filename);
    return 0;
  }

  for (int j = 0; j < prob_.l ; j++)
  {
    if (labelled)
      myfile << prob_.y[j] << " ";

    // for (int i=0; i < nFeatures+2; i++)
    int i = 0;

    while (prob_.x[j][i].index != -1)
    {
      myfile << prob_.x[j][i].index << ":" << prob_.x[j][i].value << " ";
      i++;
    }

    myfile << "\n";
  }

  myfile.close();

  //std::cout << " * " << filename << " saved" << std::endl;
  return 1;
}

bool
pcl::SVMClassify::loadClassifierModel (const char *filename)
{
  SVMModel *out;
  out = static_cast<SVMModel*> (svm_load_model (filename));
  if (out == NULL)
  {
    PCL_ERROR ("[pcl::%s::loadClassifierModel] Can't open classifier model %s.\n", getClassName ().c_str (), filename);
    return 0;
  }
  
  model_ = *out;
  free (out);

  if (model_.l == 0)
  {
    PCL_ERROR ("[pcl::%s::loadClassifierModel] Can't open classifier model %s.\n", getClassName ().c_str (), filename);
    return 0;
  }

  scaling_.obj = model_.scaling;

  int i = 0;

  while (model_.scaling[i].index != -1)
    i++;

  scaling_.max = i;

  return 1;
}

bool
pcl::SVMClassify::classificationTest ()
{
  if (model_.l == 0)
  {
    PCL_ERROR ("[pcl::%s::classificationTest] Classifier model has no data.\n", getClassName ().c_str ());
    return 0;
  }
  
  if (prob_.l == 0)
  {
    PCL_ERROR ("[pcl::%s::classificationTest] Input dataset has no data.\n", getClassName ().c_str ());
    return 0;
  }
  
  if (!labelled_training_set_)
  {
    PCL_ERROR ("[pcl::%s::classificationTest] Input dataset is not labelled.\n", getClassName ().c_str ());
    return 0;
  }  

  if (predict_probability_)
  {
    if (svm_check_probability_model (&model_) == 0)
    {
      PCL_WARN ("[pcl::%s::classificationTest] Classifier model does not support probabiliy estimates. Automatically disabled.\n", getClassName ().c_str ());
      predict_probability_ = 0;
    }
  }
  else
  {
    if (svm_check_probability_model (&model_) != 0)
      PCL_WARN("[pcl::%s::classificationTest] Classifier model supports probability estimates, but disabled in prediction.\n", getClassName ().c_str ());
  }

  int correct = 0;

  int total = 0;
  double error = 0;
  double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

  int svm_type = svm_get_svm_type (&model_);
  int nr_class = svm_get_nr_class (&model_);
  double *prob_estimates = NULL;
  int j;

  prediction_.clear ();

  if (predict_probability_)
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      PCL_WARN ("[pcl::%s::classificationTest] Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", getClassName ().c_str (),svm_get_svr_probability (&model_));
    else
    {
      prob_estimates = static_cast<double *> (malloc (nr_class * sizeof (double)));
    }
  }

  int ii = 0;

  prediction_.resize (prob_.l);

  while (ii < prob_.l)
  {
    //int i = 0;
    double target_label, predict_label;
    //char *idx, *val, *endptr;
    //int inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0

    target_label = prob_.y[ii]; //takes the first label

    if (predict_probability_ && (svm_type == C_SVC || svm_type == NU_SVC))
    {
      predict_label = svm_predict_probability (&model_, prob_.x[ii], prob_estimates);
      prediction_[ii].push_back (predict_label);

      for (j = 0;j < nr_class;j++)
      {
        prediction_[ii].push_back (prob_estimates[j]);
      }
    }
    else
    {
      predict_label = svm_predict (&model_, prob_.x[ii]);
      prediction_[ii].push_back (predict_label);
    }


    if (predict_label == target_label)
      ++correct;

    error += (predict_label - target_label) * (predict_label - target_label);
    sump  += predict_label;
    sumt  += target_label;
    sumpp += predict_label * predict_label;
    sumtt += target_label * target_label;
    sumpt += predict_label * target_label;

    ++total;
    ii++;
  }

  if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
  {
    pcl::console::print_info (" - Mean squared error (regression) = ");
    pcl::console::print_value ("%g\n", error / total);

    pcl::console::print_info (" - Squared correlation coefficient (regression) = ");
    pcl::console::print_value ("%g\n",
                               ( (total*sumpt - sump*sumt) * (total*sumpt - sump*sumt)) /
                               ( (total*sumpp - sump*sump) * (total*sumtt - sumt*sumt))
                              );
  }
  else
  {
    pcl::console::print_info (" - Accuracy (classification) = ");
    pcl::console::print_value ("%g%% (%d/%d)\n",
            double (correct) / total*100, correct, total);
  }

  if (predict_probability_)
    free (prob_estimates);

  return true;
}

bool
pcl::SVMClassify::classification ()
{
  if (model_.l == 0)
  {
    PCL_ERROR ("[pcl::%s::classification] Classifier model has no data.\n", getClassName ().c_str ());
    return 0;
  }
  
  if (prob_.l == 0)
  {
    PCL_ERROR ("[pcl::%s::classification] Input dataset has no data.\n", getClassName ().c_str ());
    return 0;
  }
  

  if (predict_probability_)
  {
    if (svm_check_probability_model (&model_) == 0)
    {
      PCL_WARN ("[pcl::%s::classification] Classifier model does not support probabiliy estimates. Automatically disabled.\n", getClassName ().c_str ());
      predict_probability_ = 0;
    }
  }
  else
  {
    if (svm_check_probability_model (&model_) != 0)
      PCL_WARN("[pcl::%s::classification] Classifier model supports probability estimates, but disabled in prediction.\n", getClassName ().c_str ());
  }

  //int correct = 0;
  int total = 0;
  int svm_type = svm_get_svm_type (&model_);
  int nr_class = svm_get_nr_class (&model_);

  double *prob_estimates = NULL;

  int j;

  prediction_.clear();

  if (predict_probability_)
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      PCL_WARN ("[pcl::%s::classificationTest] Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", getClassName ().c_str (),svm_get_svr_probability (&model_));
    else
    {
      prob_estimates = static_cast<double *> (malloc (nr_class * sizeof (double)));
    }
  }

  int ii = 0;

  prediction_.resize (prob_.l);

  while (ii < prob_.l)
  {
    double predict_label;
    //char *idx, *val, *endptr;
    //int inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0

    if (predict_probability_ && (svm_type == C_SVC || svm_type == NU_SVC))
    {
      predict_label = svm_predict_probability (&model_, prob_.x[ii], prob_estimates);
      prediction_[ii].push_back (predict_label);

      for (j = 0;j < nr_class;j++)
      {
        prediction_[ii].push_back (prob_estimates[j]);
      }
    }
    else
    {
      predict_label = svm_predict (&model_, prob_.x[ii]);
      prediction_[ii].push_back (predict_label);
    }

    ++total;

    ii++;
  }

  if (predict_probability_)
    free (prob_estimates);

  return (true);
}

std::vector<double>
pcl::SVMClassify::classification (pcl::SVMData in)
{
  assert (model_.l != 0);

  if (model_.l == 0)
  {
    PCL_ERROR ("[pcl::%s::classification] Classifier model has no data.\n", getClassName ().c_str ());
    exit(0);
  }
  
  if (predict_probability_)
  {
    if (svm_check_probability_model (&model_) == 0)
    {
      PCL_WARN ("[pcl::%s::classification] Classifier model does not support probabiliy estimates. Automatically disabled.\n", getClassName ().c_str ());
      predict_probability_ = 0;
    }
  }
  else
  {
    if (svm_check_probability_model (&model_) != 0)
      PCL_WARN("[pcl::%s::classification] Classifier model supports probability estimates, but disabled in prediction.\n", getClassName ().c_str ());
  }

  int svm_type = svm_get_svm_type (&model_);
  int nr_class = svm_get_nr_class (&model_);
  double *prob_estimates = NULL;

  int j;

  svm_node *buff;
  buff = Malloc (struct svm_node, in.SV.size() + 10);

  size_t i = 0;

  for (i = 0; i < in.SV.size (); i++)
  {
    buff[i].index = in.SV[i].idx;

    if (in.SV[i].idx < scaling_.max  && scaling_.obj[in.SV[i].idx].index == 1)
      buff[i].value = in.SV[i].value / scaling_.obj[in.SV[i].idx].value;
    else
      buff[i].value = in.SV[i].value;
  }

  buff[i].index = -1;

  // clean the prediction vector
  prediction_.clear ();

  if (predict_probability_)
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
     PCL_WARN ("[pcl::%s::classification] Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", getClassName ().c_str (),svm_get_svr_probability (&model_));
    else
    {
      prob_estimates = static_cast<double *> (malloc (nr_class * sizeof (double)));
    }
  }

  prediction_.resize (1);

  double predict_label;

  if (predict_probability_ && (svm_type == C_SVC || svm_type == NU_SVC))
  {
    predict_label = svm_predict_probability (&model_, buff, prob_estimates);
    prediction_[0].push_back (predict_label);

    for (j = 0;j < nr_class;j++)
    {
      prediction_[0].push_back (prob_estimates[j]);
    }
  }
  else
  {
    predict_label = svm_predict (&model_, buff);
    prediction_[0].push_back (predict_label);
  }

  if (predict_probability_)
    free (prob_estimates);

  free (buff);

  return prediction_[0];
};

void
pcl::SVMClassify::scaleProblem (svm_problem &input, svm_scaling scaling)
{
  assert (scaling.max != 0);

  for (int i = 0;i < input.l;i++)
  {
    int j = 0;

    while (1)
    {
      if (input.x[i][j].index == -1)
        break;

      if (input.x[i][j].index < scaling.max && scaling.obj[ input.x[i][j].index ].index == 1)
        input.x[i][j].value = input.x[i][j].value / scaling.obj[ input.x[i][j].index ].value;

      j++;
    }
  }
}

void
pcl::SVMClassify::saveClassificationResult (const char *filename)
{
  assert (prediction_.size() > 0);
  assert (model_.l > 0);

  std::ofstream output;
  output.open (filename);

  int nr_class = svm_get_nr_class (&model_);
  int *labels = static_cast<int *> (malloc (nr_class * sizeof (int)));
  svm_get_labels (&model_, labels);

  if (predict_probability_)
  {
    output << "labels ";

    for (int j = 0 ; j < nr_class; j++)
      output << labels[j] << " ";

    output << "\n";
  }

  for (size_t i = 0; i < prediction_.size(); i++)
  {
    for (size_t j = 0; j < prediction_[i].size(); j++)
      output << prediction_[i][j] << " ";

    output << "\n";
  }

  output.close();

  free (labels);
}

#define PCL_INSTANTIATE_SVM(T) template class PCL_EXPORTS pcl::SVM<T>;
#define PCL_INSTANTIATE_SVMTrain(T) template class PCL_EXPORTS pcl::SVMTrain<T>;
#define PCL_INSTANTIATE_SVMClassify(T) template class PCL_EXPORTS pcl::SVMClassify<T>;

#endif //PCL_SVM_WRAPPER_HPP_
