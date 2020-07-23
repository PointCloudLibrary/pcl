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

#pragma once

#include <pcl/console/print.h> // for PCL_ERROR
#include <pcl/ml/svm.h>

#include <cassert> // for assert
#include <cstdio>
#include <cstdlib>
#include <limits> // for numeric_limits
#include <string> // for string
#include <vector>
#define Malloc(type, n) static_cast<type*>(malloc((n) * sizeof(type)))

namespace pcl {

/** The structure stores the parameters for the classificationa nd must be initialized
 *  and passed to the training method pcl::SVMTrain.
 *
 * \param svm_type {C_SVC, NU_SVC, ONE_CLASS, EPSILON_SVR, NU_SVR}
 * \param kernel_type {LINEAR, POLY, RBF, SIGMOID, PRECOMPUTED}
 * \param probability sets the probability estimates
 */
struct SVMParam : svm_parameter {
  SVMParam()
  {
    svm_type = C_SVC;  // C_SVC, NU_SVC, ONE_CLASS, EPSILON_SVR, NU_SVR
    kernel_type = RBF; // LINEAR, POLY, RBF, SIGMOID, PRECOMPUTED
    degree = 3;        // for poly
    gamma = 0;         // 1/num_features {for poly/rbf/sigmoid}
    coef0 = 0;         //  for poly/sigmoid

    nu = 0.5;         // for NU_SVC, ONE_CLASS, and NU_SVR
    cache_size = 100; // in MB
    C = 1;            // for C_SVC, EPSILON_SVR and NU_SVR
    eps = 1e-3;       // stopping criteria
    p = 0.1;          // for EPSILON_SVR
    shrinking = 0;    // use the shrinking heuristics
    probability = 0;  // do probability estimates

    nr_weight = 0;          // for C_SVC
    weight_label = nullptr; // for C_SVC
    weight = nullptr;       // for C_SVC
  }
};

/** The structure initialize a model created by the SVM (Support Vector Machines)
 *  classifier (pcl::SVMTrain).
 */
struct SVMModel : svm_model {
  SVMModel()
  {
    l = 0;
    probA = nullptr;
    probB = nullptr;
  }
};

/** The structure initialize a single feature value for the classification using
 *  SVM (Support Vector Machines).
 */
struct SVMDataPoint {
  /// It's the feature index. It has to be an integer number greater or equal to zero
  int idx;
  /// The value assigned to the correspondent feature.
  float value;

  SVMDataPoint() : idx(-1), value(0) {}
};

/** The structure stores the features and the label of a single sample which has to be
 *  used for the training or the classification of the SVM (Support Vector Machines).
 */
struct SVMData {
  /// Pointer to the label value. It is a mandatory to train the classifier
  double label;
  /// Vector of features for the specific sample.
  std::vector<pcl::SVMDataPoint> SV;

  SVMData() : label(std::numeric_limits<double>::signaling_NaN()) {}
};

/** Base class for SVM SVM (Support Vector Machines). */
class SVM {
protected:
  std::vector<SVMData> training_set_; // Basic training set
  svm_problem prob_;    // contains the problem (vector of samples with their features)
  SVMModel model_;      // model of the classifier
  svm_scaling scaling_; // for the best model training, the input dataset is scaled and
                        // the scaling factors are stored here
  SVMParam param_;      // it stores the training parameters
  std::string class_name_; // The SVM class name.

  char* line_;                 // buffer for line reading
  int max_line_len_;           // max line length in the input file
  bool labelled_training_set_; // it stores whether the input set of samples is labelled

  /** Set for output printings during classification. */
  static void
  printNull(const char*){};

  /** To read a line from the input file. Stored in "line_". */
  char*
  readline(FILE* input);

  /** Outputs an error in file reading. */
  void
  exitInputError(int line_num)
  {
    fprintf(stderr, "Wrong input format at line %d\n", line_num);
    exit(1);
  }

  /** Get a string representation of the name of this class. */
  inline const std::string&
  getClassName() const
  {
    return (class_name_);
  }

  /** Convert the input format (vector of SVMData) into a readable format for libSVM. */
  void
  adaptInputToLibSVM(std::vector<SVMData> training_set, svm_problem& prob);

  /** Convert the libSVM format (svm_problem) into a easier output format. */
  void
  adaptLibSVMToInput(std::vector<SVMData>& training_set, svm_problem prob) const;

  /** Load a problem from an extern file. */
  bool
  loadProblem(const char* filename, svm_problem& prob);

  /** Save the raw problem in an extern file.*/
  bool
  saveProblem(const char* filename, bool labelled);

  /** Save the problem (with normalized values) in an extern file.*/
  bool
  saveProblemNorm(const char* filename, svm_problem prob_, bool labelled);

public:
  /**  Constructor. */
  SVM() : prob_(), line_(nullptr), max_line_len_(10000), labelled_training_set_(true) {}

  /** Destructor. */
  ~SVM()
  {
    svm_destroy_param(&param_); // delete parameters

    if (scaling_.max > 0)
      free(scaling_.obj); // delete scaling factors

    // delete the problem
    if (prob_.l > 0) {
      free(prob_.x);
      free(prob_.y);
    }
  }

  /** Return the labels order from the classifier model. */
  void
  getLabel(std::vector<int>& labels)
  {
    int nr_class = svm_get_nr_class(&model_);
    int* labels_ = static_cast<int*>(malloc(nr_class * sizeof(int)));
    svm_get_labels(&model_, labels_);

    for (int j = 0; j < nr_class; j++)
      labels.push_back(labels_[j]);

    free(labels_);
  };

  /** Save the classifier model in an extern file (in svmlight format). */
  void
  saveClassifierModel(const char* filename)
  {
    // exit if model has no data
    if (model_.l == 0)
      return;

    if (svm_save_model(filename, &model_)) {
      fprintf(stderr, "can't save model to file %s\n", filename);
      exit(1);
    }
  };
};

/** SVM (Support Vector Machines) training class for the SVM machine learning.
 *
 *  It creates a model for the classifier from a labelled input dataset.
 *
 *  OPTIONAL: pcl::SVMParam has to be given as input to vary the default training method
 *  and parameters.
 */
class SVMTrain : public SVM {
protected:
  using SVM::class_name_;
  using SVM::labelled_training_set_;
  using SVM::line_;
  using SVM::max_line_len_;
  using SVM::model_;
  using SVM::param_;
  using SVM::prob_;
  using SVM::scaling_;
  using SVM::training_set_;

  /// Set to 1 to see the training output
  bool debug_;
  /// Set too 1 for cross validating the classifier
  int cross_validation_;
  /// Number of folds to be used during cross validation. It indicates in how many parts
  /// is split the input training set.
  int nr_fold_;

  /** To cross validate the classifier. It is automatic for probability estimate. */
  void
  doCrossValidation();

  /** It extracts scaling factors from the input training_set.
   *
   *  The scaling of the training_set is a mandatory for a good training of the
   *  classifier. */
  void
  scaleFactors(std::vector<SVMData> training_set, svm_scaling& scaling);

public:
  /** Constructor. */
  SVMTrain() : debug_(false), cross_validation_(0), nr_fold_(0)
  {
    class_name_ = "SVMTrain";
    svm_set_print_string_function(
        &printNull); // Default to NULL to not print debugging info
  }

  /** Destructor. */
  ~SVMTrain()
  {
    if (model_.l > 0)
      svm_free_model_content(&model_);
  }

  /** Change default training parameters (pcl::SVMParam). */
  void
  setParameters(SVMParam param)
  {
    param_ = param;
  }

  /** Return the current training parameters. */
  SVMParam
  getParameters()
  {
    return param_;
  }

  /** Return the result of the training. */
  SVMModel
  getClassifierModel()
  {
    return model_;
  }

  /** It adds/store the training set with labelled data. */
  void
  setInputTrainingSet(std::vector<SVMData> training_set)
  {
    training_set_.insert(training_set_.end(), training_set.begin(), training_set.end());
  }

  /** Return the current training set. */
  std::vector<SVMData>
  getInputTrainingSet()
  {
    return training_set_;
  }

  /** Reset the training set. */
  void
  resetTrainingSet()
  {
    training_set_.clear();
  }

  /** Start the training of the SVM classifier.
   *
   * \return false if fails
   */
  bool
  trainClassifier();

  /** Read in a problem (in svmlight format).
   *
   * \return false if fails
   */
  bool
  loadProblem(const char* filename)
  {
    return SVM::loadProblem(filename, prob_);
  };

  /** Set to 1 for debugging info. */
  void
  setDebugMode(bool in)
  {
    debug_ = in;

    if (in)
      svm_set_print_string_function(nullptr);
    else
      svm_set_print_string_function(&printNull);
  };

  /** Save the raw training set in a file (in svmlight format).
   *
   * \return false if fails
   */
  bool
  saveTrainingSet(const char* filename)
  {
    return SVM::saveProblem(filename, true);
  };

  /** Save the normalized training set in a file (in svmlight format).
   *
   * \return false if fails
   */
  bool
  saveNormTrainingSet(const char* filename)
  {
    return SVM::saveProblemNorm(filename, prob_, true);
  };
};

/** SVM (Support Vector Machines) classification of a dataset.
 *
 *  It can be used both for testing a classifier model and for classify of new data.
 */
class SVMClassify : public SVM {
protected:
  using SVM::class_name_;
  using SVM::labelled_training_set_;
  using SVM::line_;
  using SVM::max_line_len_;
  using SVM::model_;
  using SVM::param_;
  using SVM::prob_;
  using SVM::scaling_;
  using SVM::training_set_;

  bool model_extern_copied_; // Set to 0 if the model is loaded from an extern file.
  bool predict_probability_; // Set to 1 to predict probabilities.
  std::vector<std::vector<double>> prediction_; // It stores the resulting prediction.

  /** It scales the input dataset using the model information. */
  void
  scaleProblem(svm_problem& input, svm_scaling scaling);

public:
  /** Constructor. */
  SVMClassify() : model_extern_copied_(false), predict_probability_(false)
  {
    class_name_ = "SvmClassify";
  }

  /** Destructor. */
  ~SVMClassify()
  {
    if (!model_extern_copied_ && model_.l > 0)
      svm_free_model_content(&model_);
  }

  /** It adds/store the training set with labelled data. */
  void
  setInputTrainingSet(std::vector<SVMData> training_set)
  {
    assert(training_set.size() > 0);

    if (scaling_.max == 0) {
      // to be sure to have loaded the scaling
      PCL_ERROR("[pcl::%s::setInputTrainingSet] Classifier model not loaded!\n",
                getClassName().c_str());
      return;
    }

    training_set_.insert(training_set_.end(), training_set.begin(), training_set.end());
    SVM::adaptInputToLibSVM(training_set_, prob_);
  }

  /** Return the current training set. */
  std::vector<SVMData>
  getInputTrainingSet()
  {
    return training_set_;
  }

  /** Reset the training set. */
  void
  resetTrainingSet()
  {
    training_set_.clear();
  }

  /** Read in a classifier model (in svmlight format).
   *
   * \return false if fails
   */
  bool
  loadClassifierModel(const char* filename);

  /** Get the result of the classification. */
  void
  getClassificationResult(std::vector<std::vector<double>>& out)
  {
    out.clear();
    out.insert(out.begin(), prediction_.begin(), prediction_.end());
  }

  /** Save the classification result in an extern file. */
  void
  saveClassificationResult(const char* filename);

  /** Set the classifier model. */
  void
  setClassifierModel(SVMModel model)
  {
    // model (inner pointers are references)
    model_ = model;
    int i = 0;

    while (model_.scaling[i].index != -1)
      i++;

    scaling_.max = i;
    scaling_.obj = Malloc(struct svm_node, i + 1);
    scaling_.obj[i].index = -1;

    // Performing full scaling copy
    for (int j = 0; j < i; j++) {
      scaling_.obj[j] = model_.scaling[j];
    }

    model_extern_copied_ = true;
  };

  /** Read in a raw classification problem (in svmlight format).
   *
   *  The values are normalized using the classifier model information.
   *
   * \return false if fails
   */
  bool
  loadClassProblem(const char* filename)
  {
    assert(model_.l != 0);

    bool out = SVM::loadProblem(filename, prob_);
    SVM::adaptLibSVMToInput(training_set_, prob_);
    scaleProblem(prob_, scaling_);
    return out;
  };

  /** Read in a normalized classification problem (in svmlight format).
   *
   *  The data are kept whitout normalizing.
   *
   * \return false if fails
   */
  bool
  loadNormClassProblem(const char* filename)
  {
    bool out = SVM::loadProblem(filename, prob_);
    SVM::adaptLibSVMToInput(training_set_, prob_);
    return out;
  };

  /** Set whether the classification has to be done with the probability estimate. (The
   *  classifier model has to support it). */
  void
  setProbabilityEstimates(bool set)
  {
    predict_probability_ = set;
  };

  /** Start the classification on labelled input dataset.
   *
   *  It returns the accuracy percentage. To get the classification result, use
   *  getClassificationResult().
   *
   * \return false if fails
   */
  bool
  classificationTest();

  /** Start the classification on un-labelled input dataset.
   *
   *  To get the classification result, use getClassificationResult().
   *
   * \return false if fails
   */
  bool
  classification();

  /** Start the classification on a single set. */
  std::vector<double>
  classification(SVMData in);

  /** Save the raw classification problem in a file (in svmlight format).
   *
   * \return false if fails
   */
  bool
  saveClassProblem(const char* filename)
  {
    return SVM::saveProblem(filename, false);
  };

  /** Save the normalized classification problem in a file (in svmlight format).
   *
   * \return false if fails
   */
  bool
  saveNormClassProblem(const char* filename)
  {
    return SVM::saveProblemNorm(filename, prob_, false);
  };
};

} // namespace pcl
