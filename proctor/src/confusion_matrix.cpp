#include "proctor/confusion_matrix.h"

#include <iostream>
#include <algorithm>

namespace pcl {

  namespace proctor {

    void ConfusionMatrix::increment(std::string truth, std::string guess) {
      if (std::find(ids.begin(), ids.end(), truth) == ids.end()) {
        ids.push_back(truth);
      }

      if (std::find(ids.begin(), ids.end(), guess) == ids.end()) {
        ids.push_back(guess);
      }
      matrix[truth][guess] += 1;
    }

    void ConfusionMatrix::printMatrix() {
      for (std::vector<std::string>::iterator it = ids.begin(); it != ids.end(); ++it) {
        std::cout << *it << ":";
        for (std::vector<std::string>::iterator it2 = ids.begin(); it2 != ids.end(); ++it2) {
          std::cout << "\t" << matrix[*it][*it2];
        }

        std::cout << std::endl;
      }
    }

    int ConfusionMatrix::total() {
      int total = 0;

      for (std::vector<std::string>::iterator it = ids.begin(); it != ids.end(); ++it) {
        for (std::vector<std::string>::iterator it2 = ids.begin(); it2 != ids.end(); ++it2) {
          total += matrix[*it][*it2];
        }
      }

      return total;
    }

    int ConfusionMatrix::trace() {
      int trace = 0;

      for (std::vector<std::string>::iterator it = ids.begin(); it != ids.end(); ++it) {
        trace += matrix[*it][*it];
      }

      return trace;
    }

  }

}
