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
#include "gles_helper.h"
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

#if 0
  Depth
  getDepth(float x, float y) const
  {
    int intx = int (x), inty = int (y);

    if (intx < 0 || intx >= width || inty < 0 || inty >= height)
      return BACKGROUND_DEPTH;

    return depths[intx + width * inty];
  }
#endif

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
#if 0
    for (unsigned x = 0; x < image.getWidth(); ++x)
      for (unsigned y = 0; y < image.getHeight(); ++y)
        labels[x + y * image.getWidth()] = image.getDepth(x, y) == BACKGROUND_DEPTH ?
              Labels::Background : walk(image, x, y);
#else
    tbb::parallel_for(
          tbb::blocked_range2d<unsigned>(0, image.getHeight(), 0, image.getWidth()),
          WalkHelper(*this, image, labels)
    );
#endif
  }
};


struct DecisionTreeGPU : boost::noncopyable
{
private:
  std::auto_ptr<GlesHelper> helper;
  GLuint depth_image_tex, labels_tex;

public:
  DecisionTreeGPU(const char * data)
  {
    const int tree_width = 2048;
    boost::uint16_t tree_depth;
    std::memcpy (&tree_depth, data, sizeof tree_depth);
    data += sizeof tree_depth;
    const int nodes_height = (1 << tree_depth) / tree_width;

    const char * fs_source = reinterpret_cast<const char *> (source_tree_walk_fsh);
    std::string fs_macros =
        (boost::format("#define TREE_DEPTH %1%\n#define NODES_HEIGHT %2%\n") % tree_depth % nodes_height).str();
    std::vector<const char *> fs_sources;
    fs_sources.push_back(fs_macros.c_str());
    fs_sources.push_back(fs_source);

    helper.reset(new GlesHelper(fs_sources));

    std::vector<unsigned char> offsets1_buffer, offsets2_buffer, thresholds_buffer;
    offsets1_buffer.resize(4 * tree_width * tree_width);
    offsets2_buffer.resize(4 * tree_width * tree_width);
    thresholds_buffer.resize(4 * tree_width * tree_width);

    for (int i = 0; i < (1 << tree_depth) - 1; ++i)
    {
      std::memcpy(&offsets1_buffer[4 * i], data, 4);
      std::memcpy(&offsets2_buffer[4 * i], data + 4, 4);
      std::memcpy(&thresholds_buffer[4 * i], data + 8, 2);
      data += 10;
    }

    helper->bindTextureToUniform(
          helper->addTexture(tree_width, tree_width, &offsets1_buffer.front()),
          "offsets1");
    helper->bindTextureToUniform(
          helper->addTexture(tree_width, tree_width, &offsets2_buffer.front()),
          "offsets2");
    helper->bindTextureToUniform(
          helper->addTexture(tree_width, tree_width, &thresholds_buffer.front()),
          "thresholds");

    std::vector<unsigned char> leaves_buffer(4 * tree_width * tree_width);

    for (unsigned i = 0; i < (1 << 20); ++i)
    {
      std::memcpy(&leaves_buffer[4 * i], data, 1);
      data += 1;
    }

    helper->bindTextureToUniform(
          helper->addTexture(tree_width, tree_width, &leaves_buffer.front()),
          "leaves");

    depth_image_tex = helper->addTexture();
    helper->bindTextureToUniform(depth_image_tex, "depth_image");

    labels_tex = helper->addTexture();
    helper->bindTextureToOutput(labels_tex);
  }


  void
  eval(const DepthImage & image, std::vector<Label> & labels) const
  {
    std::vector<unsigned char> depth_image_buffer(4 * image.getWidth() * image.getHeight());

    for (std::size_t i = 0; i < image.getWidth() * image.getHeight(); ++i)
    {
      Depth d = image.getDepth(i % image.getWidth(), i / image.getWidth());
      *reinterpret_cast<Depth *> (&depth_image_buffer[4 * i]) = d;
    }

    helper->setTextureData(depth_image_tex,
                           image.getWidth(), image.getHeight(), &depth_image_buffer.front());

    helper->setTextureData(labels_tex,
                           image.getWidth(), image.getHeight(), NULL);

    helper->setUniform("image_width", image.getWidth());
    helper->setUniform("image_height", image.getHeight());

    std::vector<unsigned char> labels_buffer(depth_image_buffer.size());

    helper->run(image.getWidth(), image.getHeight(), &labels_buffer.front());

    for (std::size_t i = 0; i < image.getWidth() * image.getHeight(); ++i)
      labels[i] = labels_buffer[4 * i];
  }
};

int maxElementNoTie(int num, int * elements)
{
  int max_element = 0;
  int max = elements[max_element];

  for (int i = 1; i < num; ++i)
  {
    int val = elements[i];
    if (max < val) { max_element = i; max = val; }
    else if (max == val) { max_element = -1; }
  }

  return max_element;
}

BodyPartsRecognizer::BodyPartsRecognizer(std::size_t num_trees, const char * trees[])
{
  this->trees.resize(num_trees);

  for (std::size_t i = 0; i < num_trees; ++i)
    this->trees[i].reset(new Tree(trees[i]));
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
        int bins[Labels::NUM_LABELS] = { 0 };
        std::size_t i = x + y * depth_image.getWidth();

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

#if 0
  for (std::size_t i = 0; i < labels.size (); ++i)
  {
    int bins[Labels::NUM_LABELS] = { 0 };

    for (std::size_t ti = 0; ti < trees.size (); ++ti)
      ++bins[multi_labels[ti][i]];

    int consensus = maxElementNoTie(Labels::NUM_LABELS, bins);

    if (consensus == -1)
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
    else
    {
      labels[i] = consensus;
    }
  }
#else
  tbb::parallel_for(
        tbb::blocked_range2d<unsigned>(0, depth_image.getHeight(), 0, depth_image.getWidth()),
        ConsensusHelper(multi_labels, labels, depth_image)
  );
#endif

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Finding consensus: %d ms", watch_consensus.elapsedMs());
}
