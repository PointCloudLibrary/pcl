#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>

#include <android/log.h>

#include <boost/format.hpp>
#include <boost/utility.hpp>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>

#include "body_parts_recognizer.h"
#include "rgbd_image.h"
#include "sources.h"
#include "stopwatch.h"

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
  getDepth(int x, int y) const
  {
    if (x < 0 || x >= int (width) || y < 0 || y >= int (height))
      return BACKGROUND_DEPTH;

    return depths[x + width * y];
  }

  unsigned getWidth() const { return width; }
  unsigned getHeight() const { return height; }
};

struct DecisionTreeCPU
{
  tbb::task_scheduler_init tbb_init;

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

  DecisionTreeCPU(const char * data)
  {
    std::memcpy (&depth, data, sizeof depth);
    data += sizeof depth;

    nodes.resize ((1 << depth) - 1);
    std::memcpy (&nodes.front (), data, nodes.size () * sizeof (Node));
    data += nodes.size () * sizeof (Node);

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

  struct WalkHelper
  {
  private:
    const DecisionTreeCPU & tree;
    const DepthImage & image;
    std::vector<Label> & labels;

  public:
    WalkHelper(const DecisionTreeCPU & tree, const DepthImage & image, std::vector<Label> & labels)
      : tree(tree), image(image), labels(labels)
    {
    }

    void
    operator () (const tbb::blocked_range2d<unsigned> & range) const
    {
      for (unsigned y = range.rows().begin(); y < range.rows().end(); ++y)
        for (unsigned x = range.cols().begin(); x < range.cols().end(); ++x)
          labels[x + y * image.getWidth()] = image.getDepth(x, y) == BACKGROUND_DEPTH ?
                Labels::Background : tree.walk(image, x, y);
    }
  };

  void
  eval(const DepthImage & image, std::vector<Label> & labels) const
  {
    tbb::parallel_for(
          tbb::blocked_range2d<unsigned>(0, image.getHeight(), 0, image.getWidth()),
          WalkHelper(*this, image, labels)
    );
  }
};

int maxElementNoTie(int num, unsigned * elements)
{
  int max_element = 0;
  unsigned max = elements[max_element];

  for (int i = 1; i < num; ++i)
  {
    unsigned val = elements[i];
    if (max < val) { max_element = i; max = val; }
    else if (max == val) { max_element = -1; }
  }

  return max_element;
}

struct ConsensusHelper
{
private:
  const std::vector<std::vector<Label> > & multi_labels;
  std::vector<Label> & labels;
  const DepthImage & depth_image;

public:
  ConsensusHelper(
      const std::vector<std::vector<Label> > & multi_labels,
      std::vector<Label> & labels,
      const DepthImage & depth_image
  )
    : multi_labels(multi_labels), labels(labels), depth_image(depth_image)
  {
  }

  void operator ()(const tbb::blocked_range2d<unsigned> & range) const
  {
    for (unsigned y = range.rows().begin(); y < range.rows().end(); ++y)
      for (unsigned x = range.cols().begin(); x < range.cols().end(); ++x)
      {
        std::size_t i = x + y * depth_image.getWidth();

        bool background = true;
        for (std::size_t ti = 0; ti < multi_labels.size (); ++ti)
          if (multi_labels[ti][i] != Labels::Background) background = false;

        if (background)
        {
          labels[i] = Labels::Background;
          continue;
        }

        unsigned bins[Labels::NUM_LABELS] = { 0 };

        for (std::size_t ti = 0; ti < multi_labels.size (); ++ti)
          ++bins[multi_labels[ti][i]];

        int consensus = maxElementNoTie(Labels::NUM_LABELS, bins);

        if (consensus == -1)
        {
          std::fill (bins, bins + Labels::NUM_LABELS, 0);
          Depth d = depth_image.getDepth (x, y);

          for (int off_x = -1; off_x <= 1; ++off_x)
            for (int off_y = -1; off_y <= 1; ++off_y)
            {
              Depth off_d = depth_image.getDepth (x + off_x, y + off_y);

              if (std::abs (d - off_d) < 50)
                for (std::size_t ti = 0; ti < multi_labels.size (); ++ti)
                  ++bins[multi_labels[ti][i]];
            }

          labels[i] = std::max_element (bins, bins + Labels::NUM_LABELS) - bins;
        }
        else
        {
          labels[i] = consensus;
        }
      }
  }
};

BodyPartsRecognizer::BodyPartsRecognizer(std::size_t num_trees, const char * trees[])
{
  this->trees.resize(num_trees);

  for (std::size_t i = 0; i < num_trees; ++i)
    this->trees[i].reset(new Tree(trees[i]));

#if GPU_CONSENSUS
  this->consensus_finder.reset(new ConsensusFinderGPU(num_trees));
#endif
}

void
BodyPartsRecognizer::recognize(const RGBDImage & image, std::vector<Label> & labels)
{
  labels.clear ();
  labels.resize (image.width * image.height);

  DepthImage depth_image (image);

  Stopwatch watch_threshold;

  depth_image.applyThreshold (500);

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Thresholding: %d ms", watch_threshold.elapsedMs());

  std::vector<std::vector<Label> > multi_labels (trees.size ());

  for (std::size_t ti = 0; ti < trees.size (); ++ti)
  {
    Stopwatch watch_evaluation;

    multi_labels[ti].resize (labels.size ());
    trees[ti]->eval (depth_image, multi_labels[ti]);

    __android_log_print(ANDROID_LOG_INFO, "BPR", "Evaluating tree %d: %d ms", ti, watch_evaluation.elapsedMs());
  }

  Stopwatch watch_consensus;

  tbb::parallel_for(
        tbb::blocked_range2d<unsigned>(0, depth_image.getHeight(), 0, depth_image.getWidth()),
        ConsensusHelper(multi_labels, labels, depth_image)
  );

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Finding consensus: %d ms", watch_consensus.elapsedMs());
}
