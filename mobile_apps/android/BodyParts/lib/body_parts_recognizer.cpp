#include <algorithm>

#include "body_parts_recognizer.h"
#include "rgbd_image.h"

void BodyPartsRecognizer::recognize(const RGBDImage & image, std::vector<signed char> & labels)
{
  labels.clear ();
  labels.resize (image.width * image.height);

  for (std::size_t i = 0; i < labels.size (); ++i)
    labels[i] = i % 4 == 0;
}
