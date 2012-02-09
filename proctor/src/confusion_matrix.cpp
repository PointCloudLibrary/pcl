#include "boost/format.hpp"
#include "proctor/confusion_matrix.h"

#include <iostream>
#include <algorithm>

namespace pcl
{

  namespace proctor
  {

    void
    ConfusionMatrix::increment(std::string truth, std::string guess)
    {
      if (std::find(ids_.begin(), ids_.end(), truth) == ids_.end()) {
        ids_.push_back(truth);
      }

      if (std::find(ids_.begin(), ids_.end(), guess) == ids_.end()) {
        ids_.push_back(guess);
      }
      matrix_[truth][guess] += 1;
    }

    void
    ConfusionMatrix::printMatrix()
    {
      for (std::vector<std::string>::iterator it = ids_.begin(); it != ids_.end(); ++it) {
        std::cout << boost::format("%-18s:      ") % *it;
        for (std::vector<std::string>::iterator it2 = ids_.begin(); it2 != ids_.end(); ++it2) {
          std::cout << boost::format("%-8d ") % matrix_[*it][*it2];
        }

        std::cout << std::endl;
      }
    }

    int
    ConfusionMatrix::total()
    {
      int total = 0;

      for (std::vector<std::string>::iterator it = ids_.begin(); it != ids_.end(); ++it) {
        for (std::vector<std::string>::iterator it2 = ids_.begin(); it2 != ids_.end(); ++it2) {
          total += matrix_[*it][*it2];
        }
      }

      return total;
    }

    int
    ConfusionMatrix::trace()
    {
      int trace = 0;

      for (std::vector<std::string>::iterator it = ids_.begin(); it != ids_.end(); ++it) {
        trace += matrix_[*it][*it];
      }

      return trace;
    }

  }

}
