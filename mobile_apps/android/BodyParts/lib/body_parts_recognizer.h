#ifndef BODY_PARTS_RECOGNIZER_
#define BODY_PARTS_RECOGNIZER_

#include <vector>

#include "rgbd_image.h"

struct BodyPartsRecognizer
{
  void recognize(const RGBDImage & image, std::vector<signed char> & labels);
};

#endif // BODY_PARTS_RECOGNIZER_
