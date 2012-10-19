#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>

#include <android/log.h>

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>

#include "body_parts_recognizer.h"
#include "cloud.h"
#include "stopwatch.h"

const Depth BACKGROUND_DEPTH = std::numeric_limits<Depth>::max();

template <typename Format> void
applyThreshold(ChannelRef<Format> & channel, Format threshold, Format bgValue)
{
  for (int i = 0; i < channel.size; ++i)
    if (channel.data[i] > threshold)
      channel.data[i] = bgValue;
}

template <typename Format> Format
selectThreshold(const ChannelRef<Format> & channel) {
  Format min = *std::min_element(channel.data, channel.data + channel.size);
  return min + 500;
}

template <typename Format> Format
replaceZeroes(ChannelRef<Format> & channel, Format replacement)
{
  std::replace(channel.data, channel.data + channel.size, Format(), replacement);
}

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
  walk(const ChannelRef<Depth> & depthChannel, int x, int y) const
  {
    unsigned nid = 0;
    Depth d0 = depthChannel.at(y, x);
    float scale = 1000.0f / d0;

    for(int node_depth = 0; node_depth < depth; ++node_depth)
    {
      const Node & node = nodes[nid];

      Depth d1 = depthChannel.atDef(y + node.offsets.dv1 * scale, x + node.offsets.du1 * scale, BACKGROUND_DEPTH);
      Depth d2 = depthChannel.atDef(y + node.offsets.dv2 * scale, x + node.offsets.du2 * scale, BACKGROUND_DEPTH);

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
    const ChannelRef<Depth> depth;
    std::vector<Label> & labels;

  public:
    WalkHelper(const DecisionTreeCPU & tree, Cloud & cloud, std::vector<Label> & labels)
      : tree(tree), depth(cloud.get<TagDepth>()), labels(labels)
    {
    }

    void
    operator () (const tbb::blocked_range2d<unsigned> & range) const
    {
      for (unsigned y = range.rows().begin(); y < range.rows().end(); ++y)
        for (unsigned x = range.cols().begin(); x < range.cols().end(); ++x)
          labels[x + y * depth.width] = depth.at(y, x) == BACKGROUND_DEPTH ?
                Labels::Background : tree.walk(depth, x, y);
    }
  };

  void
  eval(Cloud & cloud, std::vector<Label> & labels) const
  {
    tbb::parallel_for(
          tbb::blocked_range2d<unsigned>(0, cloud.getHeight(), 0, cloud.getWidth()),
          WalkHelper(*this, cloud, labels)
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

void filterLabels(Cloud & noisy, Cloud & output, int radius)
{
  int width = noisy.getWidth(), height = noisy.getHeight();
  ChannelRef<Label> noisy_labels = noisy.get<TagBPLabel>();
  ChannelRef<Label> labels = output.get<TagBPLabel>();

  for (unsigned j = 0; j < width; ++j)
    for (unsigned i = 0; i < height; ++i)
    {
      unsigned idx = i * width + j;

      if (i < radius || i >= height - radius || j < radius || j >= width - radius || noisy_labels.data[idx] == Labels::Background)
      {
        labels.data[idx] = Labels::Background;
        continue;
      }

      int bins[Labels::NUM_LABELS] = { 0 };
      Label mode = -1;
      Label mode_count = 0;

      for (int dx = -radius; dx <= radius; ++dx)
        for (int dy = -radius; dy <= radius; ++dy)
        {
          Label current = noisy_labels.data[idx + dx + dy * width];
          ++bins[current];
          if (bins[current] > mode_count) {
            mode_count = bins[current];
            mode = current;
          }
        }

      labels.data[idx] = mode;
    }
}

struct ConsensusHelper
{
private:
  const std::vector<std::vector<Label> > & multi_labels;
  ChannelRef<Label> labels;
  const ChannelRef<Depth> depths;

public:
  ConsensusHelper(
      const std::vector<std::vector<Label> > & multi_labels,
      Cloud & cloud
  )
    : multi_labels(multi_labels), labels(cloud.get<TagBPLabel>()), depths(cloud.get<TagDepth>())
  { }

  void
  operator ()(const tbb::blocked_range2d<unsigned> & range) const
  {
    for (unsigned y = range.rows().begin(); y < range.rows().end(); ++y)
      for (unsigned x = range.cols().begin(); x < range.cols().end(); ++x)
      {
        std::size_t i = &depths.at(y, x) - depths.data;

        bool background = true;
        for (std::size_t ti = 0; ti < multi_labels.size (); ++ti)
          if (multi_labels[ti][i] != Labels::Background) background = false;

        if (background)
        {
          labels.data[i] = Labels::Background;
          continue;
        }

        unsigned bins[Labels::NUM_LABELS] = { 0 };

        for (std::size_t ti = 0; ti < multi_labels.size (); ++ti)
          ++bins[multi_labels[ti][i]];

        int consensus = maxElementNoTie(Labels::NUM_LABELS, bins);

        if (consensus == -1)
        {
          std::fill (bins, bins + Labels::NUM_LABELS, 0);
          Depth d = depths.at(y, x);

          for (int off_x = -1; off_x <= 1; ++off_x)
            for (int off_y = -1; off_y <= 1; ++off_y)
            {
              Depth off_d = depths.atDef(y + off_y, x + off_x, BACKGROUND_DEPTH);

              if (std::abs (d - off_d) < 50)
                for (std::size_t ti = 0; ti < multi_labels.size (); ++ti)
                  ++bins[multi_labels[ti][i]];
            }

          labels.data[i] = std::max_element (bins, bins + Labels::NUM_LABELS) - bins;
        }
        else
        {
          labels.data[i] = consensus;
        }
      }
  }
};

BodyPartsRecognizer::BodyPartsRecognizer(std::size_t num_trees, const char * trees[])
{
  this->trees.resize(num_trees);

  for (std::size_t i = 0; i < num_trees; ++i)
    this->trees[i].reset(new Tree(trees[i]));
}

void
BodyPartsRecognizer::recognize(Cloud & cloud)
{
  ChannelRef<Depth> depth = cloud.get<TagDepth>();

  Stopwatch watch_threshold;

  replaceZeroes(depth, BACKGROUND_DEPTH);
  applyThreshold(depth, selectThreshold(depth), BACKGROUND_DEPTH);

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Thresholding: %d ms", watch_threshold.elapsedMs());

  std::vector<std::vector<Label> > multi_labels (trees.size ());

  for (std::size_t ti = 0; ti < trees.size (); ++ti)
  {
    Stopwatch watch_evaluation;

    multi_labels[ti].resize (cloud.getHeight() * cloud.getWidth());
    trees[ti]->eval (cloud, multi_labels[ti]);

    __android_log_print(ANDROID_LOG_INFO, "BPR", "Evaluating tree %d: %d ms", ti, watch_evaluation.elapsedMs());
  }

  Stopwatch watch_consensus;

  Cloud noisy;
  noisy.resize(cloud.getWidth(), cloud.getHeight());
  std::vector<Label> noisy_labels (cloud.getHeight() * cloud.getWidth());

  tbb::parallel_for(
        tbb::blocked_range2d<unsigned>(0, cloud.getHeight(), 0, cloud.getWidth()),
        ConsensusHelper(multi_labels, noisy)
  );

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Finding consensus: %d ms", watch_consensus.elapsedMs());

  Stopwatch watch_filtering;

  filterLabels(noisy, cloud, 2);

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Filtering labels: %d ms", watch_consensus.elapsedMs());

}
