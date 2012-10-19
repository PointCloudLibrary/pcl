#include <algorithm>
#include <cstring>
#include <limits>

#include <boost/multi_array.hpp>

#include "body_parts_recognizer.h"
#include "rgbd_image.h"

const Depth BACKGROUND_DEPTH = std::numeric_limits<Depth>::max();

struct DepthImage
{
private:

  unsigned width, height;
  std::vector<Depth> depths;

public:

  DepthImage(const RGBDImage & image)
    : width(image.width), height(image.height), depths(width * height)
  {
    for (std::size_t i = 0; i < depths.size(); ++i)
      depths[i] = image.pixels[i].d == 0 ? BACKGROUND_DEPTH : image.pixels[i].d; // we consider 0 to be invalid depth
  }

  void
  applyThreshold(int threshold)
  {
    Depth min_depth = *std::min_element(depths.begin (), depths.end ());

    for (std::size_t i = 0; i < depths.size (); ++i)
      depths[i] = depths[i] <= min_depth + threshold ? depths[i] : BACKGROUND_DEPTH;
  }

  Depth
  getDepth(float x, float y) const
  {
    int intx = int (x), inty = int (y);

    if (intx < 0 || intx >= width || inty < 0 || inty >= height)
      return BACKGROUND_DEPTH;

    return depths[intx + width * inty];
  }
};

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

  DecisionTree(const char * data)
  {
    std::memcpy (&depth, data, sizeof depth);
    data += sizeof depth;

    nodes.resize ((1 << depth) - 1);
    std::memcpy (&nodes.front (), data, nodes.size () * sizeof (DecisionTree::Node));
    data += nodes.size () * sizeof (DecisionTree::Node);

    leaves.resize (1 << depth);
    std::memcpy (&leaves.front (), data, leaves.size () * sizeof (Label));
    data += leaves.size () * sizeof (Label);
  }

  Label
  walk(const DepthImage & image, int x, int y) const
  {
    unsigned nid = 0;
    Depth d0 = image.getDepth(x, y);
    float scale = 1000.0f / d0;

    for(int node_depth = 0; node_depth < depth; ++node_depth)
    {
      const Node & node = nodes[nid];

      Depth d1 = image.getDepth (x + node.offsets.du1 * scale, y + node.offsets.dv1 * scale);
      Depth d2 = image.getDepth (x + node.offsets.du2 * scale, y + node.offsets.dv2 * scale);

      int feature = int (d1) - int (d2);

      if (feature > node.threshold)
        nid = nid * 2 + 2;
      else
        nid = nid * 2 + 1;
    }

    return leaves[nid - nodes.size()];
  }
};

BodyPartsRecognizer::BodyPartsRecognizer(std::size_t num_trees, const char * trees[])
{
  this->trees.resize(num_trees);

  for (std::size_t i = 0; i < num_trees; ++i)
    this->trees[i].reset(new DecisionTree(trees[i]));
}

void
BodyPartsRecognizer::recognize(const RGBDImage & image, std::vector<Label> & labels)
{
  labels.clear ();
  labels.resize (image.width * image.height);

  DepthImage depth_image (image);
  depth_image.applyThreshold (500);

  boost::multi_array<Label, 2> multi_labels (boost::extents[trees.size ()][labels.size ()]);

  for (std::size_t ti = 0; ti < trees.size (); ++ti)
  {
    for (std::size_t i = 0; i < labels.size (); ++i)
    {
      int x = i % image.width, y = i / image.width;
      multi_labels[ti][i] = depth_image.getDepth (x, y) != BACKGROUND_DEPTH
          ? trees[ti]->walk (depth_image, x, y) : Labels::Background;
    }
  }

  for (std::size_t i = 0; i < labels.size (); ++i)
  {
    int bins[Labels::NUM_LABELS] = { 0 };

    for (std::size_t ti = 0; ti < trees.size (); ++ti)
      ++bins[multi_labels[ti][i]];

    labels[i] = std::max_element (bins, bins + Labels::NUM_LABELS) - bins;

    if (std::count (bins, bins + Labels::NUM_LABELS, bins[labels[i]]) > 1)
    {
      std::fill (bins, bins + Labels::NUM_LABELS, 0);
      unsigned x = i % image.width, y = i / image.width;
      Depth d = depth_image.getDepth (x, y);

      for (int off_x = -1; off_x <= 1; ++off_x)
        for (int off_y = -1; off_y <= 1; ++off_y)
        {
          Depth off_d = depth_image.getDepth (x + off_x, y + off_y);

          if (std::abs (d - off_d) < 50)
            for (std::size_t ti = 0; ti < trees.size (); ++ti)
              ++bins[multi_labels[ti][i]];
        }

      labels[i] = std::max_element (bins, bins + Labels::NUM_LABELS) - bins;
    }
  }
}
