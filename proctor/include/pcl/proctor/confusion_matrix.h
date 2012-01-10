#ifndef CONFUSION_MATRIX_H_
#define CONFUSION_MATRIX_H_

#include <string>
#include <map>
#include <vector>

namespace pcl {

  namespace proctor {

    class ConfusionMatrix {
    public:
      void increment(std::string truth, std::string guess);

      void printMatrix();

      int total();

      int trace();

    private:
      std::map< std::string, std::map<std::string, int> > matrix;

      std::vector< std::string > ids;
    };

  }

}

#endif
