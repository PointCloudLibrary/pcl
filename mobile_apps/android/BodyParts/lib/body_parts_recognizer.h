#ifndef BODY_PARTS_RECOGNIZER_
#define BODY_PARTS_RECOGNIZER_

#include <vector>

#include <boost/cstdint.hpp>

#include "rgbd_image.h"

typedef boost::uint8_t Label;

struct DecisionTree
{
  struct Offsets
  {
    boost::int16_t du1, dv1, du2, dv2;
  };

  struct Node
  {
    Offsets offsets;
    boost::int16_t threshold;
  };

  boost::uint16_t depth;
  std::vector<Node> nodes;
  std::vector<Label> leaves;

  void read(const char * data);
};

struct BodyPartsRecognizer
{
private:
  std::vector<DecisionTree> trees;

public:
  BodyPartsRecognizer(std::size_t num_trees, const char * trees[]);
  void recognize(const RGBDImage & image, std::vector<Label> & labels);
};

#endif // BODY_PARTS_RECOGNIZER_
