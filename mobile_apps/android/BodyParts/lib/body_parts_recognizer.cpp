#include <algorithm>
#include <cstring>
#include <limits>

#include <android/log.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>

#include <boost/format.hpp>

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

#define CHECK_GL(statement) do { statement; \
  { GLenum error = glGetError (); \
    if (error != GL_NO_ERROR) \
  __android_log_print(ANDROID_LOG_DEBUG, "GLES", "GLES error at line %d: 0x%x", __LINE__, error); \
  } } while (0)

struct DecisionTreeGPU
{
private:
  EGLDisplay dpy;
  EGLSurface dummy_surface;
  EGLContext ctx;

  GLuint prog;
  GLuint depth_image_tex, offsets1_tex, offsets2_tex, thresholds_tex, leaves_tex, labels_tex;
  GLuint fb;
  GLuint pos_buffer;
  static const GLenum depth_image_unit = GL_TEXTURE0, offsets1_unit = GL_TEXTURE1, offsets2_unit = GL_TEXTURE2,
    thresholds_unit = GL_TEXTURE3, leaves_unit = GL_TEXTURE4, labels_unit = GL_TEXTURE5;

  void logShaderLog(GLuint shader)
  {
    GLint log_len, shader_type;
    glGetShaderiv (shader, GL_INFO_LOG_LENGTH, &log_len);
    glGetShaderiv (shader, GL_SHADER_TYPE, &shader_type);
    const char * shader_type_str = shader_type == GL_VERTEX_SHADER ? "Vertex" : "Fragment";

    if (log_len > 0)
    {
      std::vector<char> log (log_len);
      glGetShaderInfoLog (shader, log_len, NULL, &log.front ());
      __android_log_print (ANDROID_LOG_DEBUG, "GLES", "%s shader: %s",
                           shader_type_str,
                           &log.front ());
    }
    else
    {
      __android_log_print (ANDROID_LOG_DEBUG, "GLES", "%s shader: No log.",
                           shader_type_str);
    }
  }

  void logProgramLog(GLuint program)
  {
    GLint log_len;
    glGetProgramiv (program, GL_INFO_LOG_LENGTH, &log_len);

    if (log_len > 0)
    {
      std::vector<char> log (log_len);
      glGetProgramInfoLog (program, log_len, NULL, &log.front ());
      __android_log_print (ANDROID_LOG_DEBUG, "GLES", "Program: %s", &log.front ());
    }
    else
    {
      __android_log_write (ANDROID_LOG_DEBUG, "GLES", "Program: No log.");
    }
  }

  GLuint createAndBindTexture(const unsigned char * buffer, int width, int height)
  {
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (buffer)
      CHECK_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer));
    return tex;
  }

public:
  DecisionTreeGPU(const char * data)
  {
    dpy = eglGetDisplay (EGL_DEFAULT_DISPLAY);
    eglBindAPI (EGL_OPENGL_API);

    EGLConfig config;
    EGLint num_configs;
    EGLint config_attribs[] = {
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
      EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
      EGL_NONE
    };
    eglChooseConfig (dpy, config_attribs, &config, 1, &num_configs);

    EGLint ctx_attribs[] = {
      EGL_CONTEXT_CLIENT_VERSION, 2,
      EGL_NONE
    };
    ctx = eglCreateContext (dpy, config, EGL_NO_CONTEXT, ctx_attribs);
    dummy_surface = eglCreatePbufferSurface (dpy, config, NULL);
    eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);

    const char * vs_source = reinterpret_cast<const char *> (source_generic_vsh);
    GLuint vs = glCreateShader (GL_VERTEX_SHADER);
    glShaderSource (vs, 1, &vs_source, NULL);
    glCompileShader (vs);
    logShaderLog(vs);

    const int tree_width = 2048;
    boost::uint16_t tree_depth;
    std::memcpy (&tree_depth, data, sizeof tree_depth);
    data += sizeof tree_depth;
    const int nodes_height = (1 << tree_depth) / tree_width;

    const char * fs_source = reinterpret_cast<const char *> (source_tree_walk_fsh);
    std::string fs_macros =
        (boost::format("#define TREE_DEPTH %1%\n#define NODES_HEIGHT %2%\n") % tree_depth % nodes_height).str();
    const char * fs_sources[] = { fs_macros.c_str(), fs_source };
    GLuint fs = glCreateShader (GL_FRAGMENT_SHADER);
    CHECK_GL(glShaderSource (fs, 2, fs_sources, NULL));
    CHECK_GL(glCompileShader (fs));
    CHECK_GL(logShaderLog(fs));

    prog = glCreateProgram ();
    CHECK_GL(glAttachShader (prog, vs));
    CHECK_GL(glAttachShader (prog, fs));
    CHECK_GL(glLinkProgram (prog));
    CHECK_GL(logProgramLog(prog));

    glDeleteShader(vs);
    glDeleteShader(fs);

    glUseProgram(prog);

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

    glActiveTexture(offsets1_unit);
    offsets1_tex = createAndBindTexture(&offsets1_buffer.front(), tree_width, tree_width);

    glActiveTexture(offsets2_unit);
    offsets2_tex = createAndBindTexture(&offsets2_buffer.front(), tree_width, tree_width);

    glActiveTexture(thresholds_unit);
    thresholds_tex = createAndBindTexture(&thresholds_buffer.front(), tree_width, tree_width);

    std::vector<unsigned char> leaves_buffer(4 * tree_width * tree_width);

    for (unsigned i = 0; i < (1 << 20); ++i)
    {
      std::memcpy(&leaves_buffer[4 * i], data, 1);
      data += 1;
    }

    glActiveTexture(leaves_unit);
    leaves_tex = createAndBindTexture(&leaves_buffer.front(), tree_width, tree_width);

    glActiveTexture(depth_image_unit);
    depth_image_tex = createAndBindTexture(NULL, 0, 0);

    glActiveTexture(labels_unit);
    labels_tex = createAndBindTexture(NULL, 0, 0);

    glGenFramebuffers(1, &fb);
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    CHECK_GL(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, labels_tex, 0));

    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "depth_image"), depth_image_unit - GL_TEXTURE0));
    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "offsets1"), offsets1_unit - GL_TEXTURE0));
    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "offsets2"), offsets2_unit - GL_TEXTURE0));
    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "thresholds"), thresholds_unit - GL_TEXTURE0));
    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "leaves"), leaves_unit - GL_TEXTURE0));

    CHECK_GL(glGenBuffers(1, &pos_buffer));
    CHECK_GL(glBindBuffer(GL_ARRAY_BUFFER, pos_buffer));
    float pos_array[] = { -1., -1., -1, 1., 1., -1., 1., 1. };
    CHECK_GL(glBufferData(GL_ARRAY_BUFFER, sizeof pos_array, pos_array, GL_STATIC_DRAW));

    int position_location = glGetAttribLocation(prog, "position");
    CHECK_GL(glVertexAttribPointer(position_location, 2, GL_FLOAT, GL_FALSE, 0, NULL));
    CHECK_GL(glEnableVertexAttribArray(position_location));
  }

  ~DecisionTreeGPU()
  {
    eglBindAPI (EGL_OPENGL_API);
    eglMakeCurrent (dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    eglDestroySurface (dpy, dummy_surface);
    eglDestroyContext (dpy, ctx);
  }

  void
  eval(const DepthImage & image, std::vector<Label> & labels) const
  {
    Stopwatch preparation;
    eglMakeCurrent (dpy, dummy_surface, dummy_surface, ctx);

    std::vector<unsigned char> depth_image_buffer(4 * image.getWidth() * image.getHeight());

    for (std::size_t i = 0; i < image.getWidth() * image.getHeight(); ++i)
    {
      Depth d = image.getDepth(i % image.getWidth(), i / image.getWidth());
      *reinterpret_cast<Depth *> (&depth_image_buffer[4 * i]) = d;
    }

    glActiveTexture(depth_image_unit);
    CHECK_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.getWidth(), image.getHeight(),
                          0, GL_RGBA, GL_UNSIGNED_BYTE, &depth_image_buffer.front()));

    glActiveTexture(labels_unit);
    CHECK_GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.getWidth(), image.getHeight(),
                          0, GL_RGBA, GL_UNSIGNED_BYTE, NULL));

    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "image_width"), image.getWidth()));
    CHECK_GL(glUniform1i(glGetUniformLocation(prog, "image_height"), image.getHeight()));

    glViewport(0, 0, image.getWidth(), image.getHeight());

    __android_log_print(ANDROID_LOG_INFO, "BPR", "Preparation: %d ms", preparation.elapsedMs());
    Stopwatch rendering;
    CHECK_GL(glDrawArrays(GL_TRIANGLE_STRIP, 0, 4));
    glFinish();

    __android_log_print(ANDROID_LOG_INFO, "BPR", "Rendering: %d ms", rendering.elapsedMs());
    Stopwatch retrieval;
    std::vector<unsigned char> labels_buffer(depth_image_buffer.size());
    CHECK_GL(glReadPixels(0, 0, image.getWidth(), image.getHeight(),
                          GL_RGBA, GL_UNSIGNED_BYTE, &labels_buffer.front()));

    for (std::size_t i = 0; i < image.getWidth() * image.getHeight(); ++i)
      labels[i] = labels_buffer[4 * i];
    __android_log_print(ANDROID_LOG_INFO, "BPR", "Retrieval: %d ms", retrieval.elapsedMs());
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
