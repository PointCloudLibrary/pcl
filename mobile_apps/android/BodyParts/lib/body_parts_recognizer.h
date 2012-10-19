#ifndef BODY_PARTS_RECOGNIZER_
#define BODY_PARTS_RECOGNIZER_

#include <vector>

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

#include "rgbd_image.h"

typedef boost::uint8_t Label;

struct DecisionTree;

struct BodyPartsRecognizer
{
private:
  std::vector<boost::shared_ptr<DecisionTree> > trees;

public:
  BodyPartsRecognizer(std::size_t num_trees, const char * trees[]);
  void recognize(const RGBDImage & image, std::vector<Label> & labels);
};

#endif // BODY_PARTS_RECOGNIZER_
