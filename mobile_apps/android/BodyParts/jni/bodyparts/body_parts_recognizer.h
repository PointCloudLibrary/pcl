#ifndef BODY_PARTS_RECOGNIZER_
#define BODY_PARTS_RECOGNIZER_

#include <memory>
#include <vector>

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

#include "cloud.h"

typedef boost::uint8_t Label;

struct Labels
{
  static const Label Lfoot       = 0;
  static const Label Lleg        = 1;
  static const Label Lknee       = 2;
  static const Label Lthigh      = 3;
  static const Label Rfoot       = 4;
  static const Label Rleg        = 5;
  static const Label Rknee       = 6;
  static const Label Rthigh      = 7;
  static const Label Rhips       = 8;
  static const Label Lhips       = 9;
  static const Label Neck        = 10;
  static const Label Rarm        = 11;
  static const Label Relbow      = 12;
  static const Label Rforearm    = 13;
  static const Label Rhand       = 14;
  static const Label Larm        = 15;
  static const Label Lelbow      = 16;
  static const Label Lforearm    = 17;
  static const Label Lhand       = 18;
  static const Label FaceLB      = 19;
  static const Label FaceRB      = 20;
  static const Label FaceLT      = 21;
  static const Label FaceRT      = 22;
  static const Label Rchest      = 23;
  static const Label Lchest      = 24;
  static const Label Lshoulder   = 25;
  static const Label Rshoulder   = 26;
  static const Label Groundplane = 27;
  static const Label Ceiling     = 28;
  static const Label Background  = 29;
  static const Label Reserved    = 30;
  static const Label NOLABEL     = 31;
  static const unsigned NUM_LABELS  = 32;
};

DECLARE_CLOUD_TAG(TagBPLabel, Label)

struct DecisionTreeCPU;

struct BodyPartsRecognizer
{
private:
  typedef DecisionTreeCPU Tree;
  std::vector<boost::shared_ptr<Tree> > trees;

public:
  BodyPartsRecognizer(std::size_t num_trees, const char * trees[]);
  void recognize(Cloud & cloud);
};

#endif // BODY_PARTS_RECOGNIZER_
