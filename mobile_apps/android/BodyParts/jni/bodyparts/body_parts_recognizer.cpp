#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>

#include <android/log.h>

#include <boost/foreach.hpp>
#include <boost/integer_traits.hpp>

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
selectThreshold(const ChannelRef<Format> & channel)
{
  // We use Otsu's method here.
  const int num_bins = boost::integer_traits<Format>::const_max + 1;
  int bins[num_bins] = { 0 };
  int count = 0;

  for (int i = 0; i < channel.size; ++i)
    if (channel.data[i] != BACKGROUND_DEPTH)
    {
      ++bins[channel.data[i]];
      ++count;
    }

  double scale = 1. / count;
  double mu = 0;

  for (int i = 0; i < num_bins; ++i) mu += i * bins[i] * scale;

  double mu1 = 0, mu2, p1 = 0, p2;
  double max_var = 0;
  Format thresh = 0;

  for (int i = 0; i < num_bins; ++i)
  {
    p1 += bins[i] * scale;
    p2 = 1 - p1;

    mu1 += i * bins[i] * scale;
    mu2 = mu - mu1;

    double var = p1 * p2 * (mu1 - mu2) * (mu1 - mu2);
    if (var > max_var)
    {
      max_var = var;
      thresh = i;
    }
  }

  return thresh;
}

template <typename Format> Format
replaceZeroes(ChannelRef<Format> & channel, Format replacement)
{
  std::replace(channel.data, channel.data + channel.size, Format(), replacement);
}

DECLARE_CLOUD_TAG(TagVisited, bool)

struct Component
{
  int y, x, surface;
};

struct ComponentSurfaceCompare
{
  bool operator() (const Component & c1, const Component & c2)
  { return c1.surface < c2.surface; }
};

template <typename Format> int
calcComponentSurface(const ChannelRef<Format> channel, ChannelRef<bool> visited, Format empty, int startY, int startX)
{
  std::queue<std::pair<int, int> > to_visit;
  to_visit.push(std::make_pair(startY, startX));
  int surface = 0;

  while (!to_visit.empty())
  {
    const std::pair<int, int> yx = to_visit.front();
    to_visit.pop();

    int y = yx.first, x = yx.second;

    if (x < 0 || x >= channel.width || y < 0 || y >= channel.height || visited.at(y, x) || channel.at(y, x) == empty) continue;

    visited.at(y, x) = true;
    ++surface;

    to_visit.push(std::make_pair(y, x + 1));
    to_visit.push(std::make_pair(y, x - 1));
    to_visit.push(std::make_pair(y + 1, x));
    to_visit.push(std::make_pair(y - 1, x));
  }

  return surface;
}

template <typename Format> void
eraseComponent(ChannelRef<Format> channel, Format empty, int y, int x)
{
  std::queue<std::pair<int, int> > to_visit;
  to_visit.push(std::make_pair(y, x));

  while (!to_visit.empty())
  {
    const std::pair<int, int> yx = to_visit.front();
    to_visit.pop();

    int y = yx.first, x = yx.second;

    if (x < 0 || x >= channel.width || y < 0 || y >= channel.height || channel.at(y, x) == empty) continue;

    channel.at(y, x) = empty;

    to_visit.push(std::make_pair(y, x + 1));
    to_visit.push(std::make_pair(y, x - 1));
    to_visit.push(std::make_pair(y + 1, x));
    to_visit.push(std::make_pair(y - 1, x));
  }
}

template <typename Format> Format
keepBiggestComponent(const ChannelRef<Format> channel, Format empty)
{
  Cloud temp;
  temp.resize(channel.width, channel.height);

  ChannelRef<bool> visited = temp.get<TagVisited>();
  std::fill_n(visited.data, visited.size, false);

  std::vector<Component> components;

  for (int i = 0; i < channel.height; ++i)
    for (int j = 0; j < channel.width; ++j)
      if (channel.at(i, j) != empty && !visited.at(i, j))
      {
        Component component = { i, j, calcComponentSurface(channel, visited, empty, i, j) };
        components.push_back(component);
      }

  const Component & biggest = *std::max_element(components.begin(), components.end(), ComponentSurfaceCompare());

  int with = 0;

  BOOST_FOREACH(const Component & c, components)
    if (c.surface < biggest.surface)
      eraseComponent(channel, empty, c.y, c.x);
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

struct FilterHelper
{
private:
  ChannelRef<Label> noisy, good;
  int radius;

public:
  FilterHelper(ChannelRef<Label> noisy, ChannelRef<Label> good, int radius)
    : noisy(noisy), good(good), radius(radius)
  { }

  void
  operator ()(const tbb::blocked_range2d<int> & range) const
  {
    int width = noisy.width, height = noisy.height;

    for (int i = range.rows().begin(); i < range.rows().end(); ++i)
      for (int j = range.cols().begin(); j < range.cols().end(); ++j)
      {
        int idx = i * width + j;

        if (i < radius || i >= height - radius || j < radius || j >= width - radius || noisy.data[idx] == Labels::Background)
        {
          good.data[idx] = Labels::Background;
          continue;
        }

        int bins[Labels::NUM_LABELS] = { 0 };
        Label mode = -1;
        Label mode_count = 0;

        for (int dy = -radius; dy <= radius; ++dy)
          for (int dx = -radius; dx <= radius; ++dx)
          {
            Label current = noisy.data[idx + dx + dy * width];
            ++bins[current];
            if (bins[current] > mode_count) {
              mode_count = bins[current];
              mode = current;
            }
          }

        good.data[idx] = mode;
      }
  }
};

void filterLabels(Cloud & noisy, Cloud & output, int radius)
{
  int width = noisy.getWidth(), height = noisy.getHeight();
  ChannelRef<Label> noisy_labels = noisy.get<TagBPLabel>();
  ChannelRef<Label> good_labels = output.get<TagBPLabel>();

  tbb::parallel_for(
        tbb::blocked_range2d<int>(0, noisy.getHeight(), 0, noisy.getWidth()),
        FilterHelper(noisy_labels, good_labels, radius)
  );
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

void
findConsensus(const std::vector<std::vector<Label> > & multi_labels, Cloud & cloud)
{
  tbb::parallel_for(
        tbb::blocked_range2d<unsigned>(0, cloud.getHeight(), 0, cloud.getWidth()),
        ConsensusHelper(multi_labels, cloud)
  );
}

BodyPartsRecognizer::BodyPartsRecognizer(std::size_t num_trees, const char * trees[])
{
  this->trees.resize(num_trees);

  for (std::size_t i = 0; i < num_trees; ++i)
    this->trees[i].reset(new Tree(trees[i]));
}

void
BodyPartsRecognizer::recognize(Cloud & cloud) const
{
  ChannelRef<Depth> depth = cloud.get<TagDepth>();

  Stopwatch watch_threshold;

  replaceZeroes(depth, BACKGROUND_DEPTH);
  applyThreshold(depth, selectThreshold(depth), BACKGROUND_DEPTH);
  keepBiggestComponent(depth, BACKGROUND_DEPTH);

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

  findConsensus(multi_labels, noisy);

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Finding consensus: %d ms", watch_consensus.elapsedMs());

  Stopwatch watch_filtering;

  filterLabels(noisy, cloud, 2);

  __android_log_print(ANDROID_LOG_INFO, "BPR", "Filtering labels: %d ms", watch_consensus.elapsedMs());

}
