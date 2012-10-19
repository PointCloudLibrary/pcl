#include <algorithm>
#include <limits>

#include "body_parts_recognizer.h"
#include "rgbd_image.h"

namespace
{
  Depth getDepth(const RGBD & pixel)
  {
    return pixel.d == 0 ? std::numeric_limits<Depth>::max() : pixel.d; // we consider 0 to be invalid depth
  }

  void applyThreshold(std::vector<Depth> & depths, int threshold)
  {
    Depth min_depth = *std::min_element(depths.begin (), depths.end ());

    for (std::size_t i = 0; i < depths.size (); ++i)
      depths[i] = depths[i] <= min_depth + threshold ? depths[i] : std::numeric_limits<Depth>::max();
  }
}

void BodyPartsRecognizer::recognize(const RGBDImage & image, std::vector<signed char> & labels)
{
  labels.clear ();
  labels.resize (image.width * image.height);

  for (std::size_t i = 0; i < labels.size (); ++i)
    labels[i] = image.pixels[i].d == 0;

  std::vector<Depth> depths (labels.size ());
  std::transform(image.pixels.begin (), image.pixels.end (), depths.begin (), getDepth);

  applyThreshold(depths, 500);

  for (std::size_t i = 0; i < labels.size (); ++i)
    labels[i] = depths[i] == std::numeric_limits<Depth>::max();

}
