#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/grabcut_segmentation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef GLUT_IS_A_FRAMEWORK
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#if defined(FREEGLUT)
#include <GL/freeglut_ext.h>
#elif defined(GLUI_OPENGLUT)
#include <GL/openglut.h>
#endif
#endif

class GrabCutHelper : public pcl::GrabCut<pcl::PointXYZRGB> {
  using pcl::GrabCut<pcl::PointXYZRGB>::n_links_;
  using pcl::GrabCut<pcl::PointXYZRGB>::graph_;
  using pcl::GrabCut<pcl::PointXYZRGB>::indices_;
  using pcl::GrabCut<pcl::PointXYZRGB>::hard_segmentation_;
  using pcl::GrabCut<pcl::PointXYZRGB>::width_;
  using pcl::GrabCut<pcl::PointXYZRGB>::height_;
  using pcl::GrabCut<pcl::PointXYZRGB>::graph_nodes_;
  using pcl::GrabCut<pcl::PointXYZRGB>::L_;
  using pcl::GrabCut<pcl::PointXYZRGB>::K_;
  using pcl::GrabCut<pcl::PointXYZRGB>::GMM_component_;
  using pcl::GrabCut<pcl::PointXYZRGB>::input_;

public:
  using Ptr = std::shared_ptr<GrabCutHelper>;
  using ConstPtr = std::shared_ptr<const GrabCutHelper>;

  GrabCutHelper(std::uint32_t K = 5, float lambda = 50.f)
  : pcl::GrabCut<pcl::PointXYZRGB>(K, lambda)
  {}

  ~GrabCutHelper() override = default;

  void
  setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) override;
  void
  setBackgroundPointsIndices(const pcl::PointIndices::ConstPtr& point_indices);
  void
  setBackgroundPointsIndices(int x1, int y1, int x2, int y2);
  void
  setTrimap(
      int x1, int y1, int x2, int y2, const pcl::segmentation::grabcut::TrimapValue& t);
  void
  refine() override;
  int
  refineOnce() override;
  void
  fitGMMs() override;
  void
  display(int display_type);
  void
  overlayAlpha();

private:
  void
  buildImages();

  // Clouds of various variables that can be displayed for debugging.
  pcl::PointCloud<float>::Ptr n_links_image_;
  pcl::segmentation::grabcut::Image::Ptr t_links_image_;
  pcl::segmentation::grabcut::Image::Ptr gmm_image_;
  pcl::PointCloud<float>::Ptr alpha_image_;

  int image_height_1_;
  int image_width_1_;
};

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
  pcl::GrabCut<pcl::PointXYZRGB>::setInputCloud(cloud);
  // Reset clouds
  n_links_image_.reset(new pcl::PointCloud<float>(cloud->width, cloud->height, 0));
  t_links_image_.reset(
      new pcl::segmentation::grabcut::Image(cloud->width, cloud->height));
  gmm_image_.reset(new pcl::segmentation::grabcut::Image(cloud->width, cloud->height));
  alpha_image_.reset(new pcl::PointCloud<float>(cloud->width, cloud->height, 0));
  image_height_1_ = cloud->height - 1;
  image_width_1_ = cloud->width - 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::setBackgroundPointsIndices(
    const pcl::PointIndices::ConstPtr& point_indices)
{
  pcl::GrabCut<pcl::PointXYZRGB>::setBackgroundPointsIndices(point_indices);
  buildImages();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::setBackgroundPointsIndices(int x1, int y1, int x2, int y2)
{
  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  point_indices->indices.reserve(input_->size());
  if (x1 > x2)
    std::swap(x1, x2);
  if (y1 > y2)
    std::swap(y1, y2);
  x1 = std::max(x1, 0);
  y1 = std::max(y1, 0);
  x2 = std::min(static_cast<int>(input_->width - 1), x2);
  y2 = std::min(static_cast<int>(input_->height - 1), y2);
  for (int y = y1; y <= y2; ++y)
    for (int x = x1; x <= x2; ++x)
      point_indices->indices.push_back(y * input_->width + x);
  setBackgroundPointsIndices(point_indices);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::setTrimap(
    int x1, int y1, int x2, int y2, const pcl::segmentation::grabcut::TrimapValue& t)
{
  using namespace pcl::segmentation::grabcut;
  if (x1 > x2)
    std::swap(x1, x2);
  if (y1 > y2)
    std::swap(y1, y2);
  x1 = std::max(x1, 0);
  y1 = std::max(y1, 0);
  x2 = std::min(static_cast<int>(image_height_1_), x2);
  y2 = std::min(static_cast<int>(image_width_1_), y2);
  for (int y = y1; y <= y2; ++y)
    for (int x = x1; x <= x2; ++x) {
      std::size_t idx = y * input_->width + x;
      trimap_[idx] = TrimapUnknown;
      // Immediately set the segmentation as well so that the display will update.
      if (t == TrimapForeground)
        hard_segmentation_[idx] = SegmentationForeground;
      else if (t == TrimapBackground)
        hard_segmentation_[idx] = SegmentationBackground;
    }

  // Build debugging images
  buildImages();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::refine()
{
  pcl::GrabCut<pcl::PointXYZRGB>::refine();
  buildImages();
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
GrabCutHelper::refineOnce()
{
  int result = pcl::GrabCut<pcl::PointXYZRGB>::refineOnce();
  buildImages();
  return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::fitGMMs()
{
  pcl::GrabCut<pcl::PointXYZRGB>::fitGMMs();
  buildImages();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::buildImages()
{
  using namespace pcl::segmentation::grabcut;
  std::fill(n_links_image_->begin(), n_links_image_->end(), 0.0f);
  for (int y = 0; y < static_cast<int>(image_->height); ++y) {
    for (int x = 0; x < static_cast<int>(image_->width); ++x) {
      std::size_t index = y * image_->width + x;

      if (x > 0 && y < image_height_1_) {
        (*n_links_image_)(x, y) += n_links_[index].weights[0];
        (*n_links_image_)(x - 1, y + 1) += n_links_[index].weights[0];
      }

      if (y < image_height_1_) {
        (*n_links_image_)(x, y) += n_links_[index].weights[1];
        (*n_links_image_)(x, y + 1) += n_links_[index].weights[1];
      }

      if (x < image_width_1_ && y < image_height_1_) {
        (*n_links_image_)(x, y) += n_links_[index].weights[2];
        (*n_links_image_)(x + 1, y + 1) += n_links_[index].weights[2];
      }

      if (x < image_width_1_) {
        (*n_links_image_)(x, y) += n_links_[index].weights[3];
        (*n_links_image_)(x + 1, y) += n_links_[index].weights[3];
      }

      // TLinks cloud
      pcl::segmentation::grabcut::Color& tlink_point = (*t_links_image_)[index];
      pcl::segmentation::grabcut::Color& gmm_point = (*gmm_image_)[index];
      float& alpha_point = (*alpha_image_)[index];
      double red = pow(graph_.getSourceEdgeCapacity(index) / L_, 0.25);   // red
      double green = pow(graph_.getTargetEdgeCapacity(index) / L_, 0.25); // green
      tlink_point.r = static_cast<float>(red);
      tlink_point.g = static_cast<float>(green);
      gmm_point.b = tlink_point.b = 0;
      // GMM cloud and Alpha cloud
      if (hard_segmentation_[index] == SegmentationForeground) {
        gmm_point.r =
            static_cast<float>(GMM_component_[index] + 1) / static_cast<float>(K_);
        alpha_point = 0;
      }
      else {
        gmm_point.g =
            static_cast<float>(GMM_component_[index] + 1) / static_cast<float>(K_);
        alpha_point = 0.75;
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::display(int display_type)
{
  switch (display_type) {
  case 0:
    glDrawPixels(image_->width, image_->height, GL_RGB, GL_FLOAT, &((*image_)[0]));
    break;

  case 1:
    glDrawPixels(
        gmm_image_->width, gmm_image_->height, GL_RGB, GL_FLOAT, &((*gmm_image_)[0]));
    break;

  case 2:
    glDrawPixels(n_links_image_->width,
                 n_links_image_->height,
                 GL_LUMINANCE,
                 GL_FLOAT,
                 &((*n_links_image_)[0]));
    break;

  case 3:
    glDrawPixels(t_links_image_->width,
                 t_links_image_->height,
                 GL_RGB,
                 GL_FLOAT,
                 &((*t_links_image_)[0]));
    break;

  default:
    // Do nothing
    break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
GrabCutHelper::overlayAlpha()
{
  glDrawPixels(alpha_image_->width,
               alpha_image_->height,
               GL_ALPHA,
               GL_FLOAT,
               &((*alpha_image_)[0]));
}

/* GUI interface */
int display_type = 0;
bool show_mask = false;
bool initialized = false;
// 2D stuff
int xstart, ystart, xend, yend;
bool box = false;
bool left = false, right = false;
bool refining_ = false;
std::uint32_t width, height;
GrabCutHelper grabcut;
pcl::segmentation::grabcut::Image::Ptr display_image;

/////////////////////////////////////////////////////////////////////////////////////////////
void
display()
{
  glClear(GL_COLOR_BUFFER_BIT);

  if (display_type == -1)
    glDrawPixels(display_image->width,
                 display_image->height,
                 GL_RGB,
                 GL_FLOAT,
                 &((*display_image)[0]));
  else
    grabcut.display(display_type);

  if (show_mask) {
    grabcut.overlayAlpha();
  }

  if (box) {
    glColor4f(1, 1, 1, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2d(xstart, ystart);
    glVertex2d(xstart, yend);
    glVertex2d(xend, yend);
    glVertex2d(xend, ystart);
    glEnd();
  }

  glFlush();
  glutSwapBuffers();
}

/////////////////////////////////////////////////////////////////////////
void
idle_callback()
{
  int changed = 0;

  if (refining_) {
    changed = grabcut.refineOnce();
    glutPostRedisplay();
  }

  if (!changed) {
    refining_ = false;
    glutIdleFunc(nullptr);
  }
}

/////////////////////////////////////////////////////////////////////////
void
motion_callback(int x, int y)
{
  y = height - y;

  if (box) {
    xend = x;
    yend = y;
    glutPostRedisplay();
  }

  if (initialized) {
    if (left)
      grabcut.setTrimap(
          x - 2, y - 2, x + 2, y + 2, pcl::segmentation::grabcut::TrimapForeground);

    if (right)
      grabcut.setTrimap(
          x - 2, y - 2, x + 2, y + 2, pcl::segmentation::grabcut::TrimapForeground);

    glutPostRedisplay();
  }
}

void
mouse_callback(int button, int state, int x, int y)
{
  y = height - y;

  switch (button) {
  case GLUT_LEFT_BUTTON:
    if (state == GLUT_DOWN) {
      left = true;

      if (!initialized) {
        xstart = x;
        ystart = y;
        box = true;
      }
    }

    if (state == GLUT_UP) {
      left = false;

      if (initialized) {
        grabcut.refineOnce();
        glutPostRedisplay();
      }
      else {
        xend = x;
        yend = y;
        grabcut.setBackgroundPointsIndices(xstart, ystart, xend, yend);
        box = false;
        initialized = true;
        show_mask = true;
        glutPostRedisplay();
      }
    }
    break;

  case GLUT_RIGHT_BUTTON:
    if (state == GLUT_DOWN) {
      right = true;
    }
    if (state == GLUT_UP) {
      right = false;

      if (initialized) {
        grabcut.refineOnce();
        glutPostRedisplay();
      }
    }
    break;

  default:
    break;
  }
}

/////////////////////////////////////////////////////////////////////////
void
keyboard_callback(unsigned char key, int, int)
{
  switch (key) {
  case ' ': // space bar show/hide alpha mask
    show_mask = !show_mask;
    break;
  case '0':
  case 'i':
  case 'I': // choose the RGB image
    display_type = 0;
    break;
  case '1':
  case 'g':
  case 'G': // choose GMM index mask
    display_type = 1;
    break;
  case '2':
  case 'n':
  case 'N': // choose N-Link mask
    display_type = 2;
    break;
  case '3':
  case 't':
  case 'T': // choose T-Link mask
    display_type = 3;
    break;
  case 'r': // run GrabCut refinement
    refining_ = true;
    glutIdleFunc(idle_callback);
    break;
  case 'o': // run one step of GrabCut refinement
    grabcut.refineOnce();
    glutPostRedisplay();
    break;
  case 'l': // rerun the Orchard-Bowman GMM clustering
    grabcut.fitGMMs();
    glutPostRedisplay();
    break;
  // case 's': case 'S':
  //   save ();
  //   break;
  case 'q':
  case 'Q':
#if defined(FREEGLUT) || defined(GLUI_OPENGLUT)
    glutLeaveMainLoop();
#else
    exit(0);
#endif
    break;
  case 27:
    refining_ = false;
    glutIdleFunc(nullptr);
  default:
    break;
  }
  glutPostRedisplay();
}

///////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char** argv)
{
  // Parse the command line arguments for .pcd files
  std::vector<int> parsed_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (parsed_file_indices.empty()) {
    // clang-format off
    pcl::console::print_error("Need at least an input PCD file (e.g. scene.pcd) to continue!\n\n");
    pcl::console::print_info("Ideally, need an input file, and two output PCD files, e.g., object.pcd, background.pcd\n");
    // clang-format on
    return -1;
  }

  pcl::PCDReader reader;
  // Test the header
  pcl::PCLPointCloud2 dummy;
  reader.readHeader(argv[parsed_file_indices[0]], dummy);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::getFieldIndex(dummy, "rgba") != -1) {
    if (pcl::io::loadPCDFile(argv[parsed_file_indices[0]], *scene) < 0) {
      pcl::console::print_error(stderr, "[error]\n");
      return -2;
    }
  }
  else if (pcl::getFieldIndex(dummy, "rgb") != -1) {
    if (pcl::io::loadPCDFile(argv[parsed_file_indices[0]], *scene) < 0) {
      pcl::console::print_error(stderr, "[error]\n");
      return -2;
    }
  }
  else {
    pcl::console::print_error(stderr, "[No RGB data found!]\n");
    return -1;
  }

  if (scene->isOrganized()) {
    pcl::console::print_highlight("Enabling 2D image viewer mode.\n");
  }

  width = scene->width;
  height = scene->height;

  display_type = -1;

  display_image.reset(
      new pcl::segmentation::grabcut::Image(scene->width, scene->height));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(
      new pcl::PointCloud<pcl::PointXYZRGB>(scene->width, scene->height));

  if (scene->isOrganized()) {
    std::uint32_t height_1 = scene->height - 1;
    for (std::size_t i = 0; i < scene->height; ++i) {
      for (std::size_t j = 0; j < scene->width; ++j) {
        const pcl::PointXYZRGB& p = (*scene)(j, i);
        std::size_t reverse_index = (height_1 - i) * scene->width + j;
        (*display_image)[reverse_index].r = static_cast<float>(p.r) / 255.0;
        (*display_image)[reverse_index].g = static_cast<float>(p.g) / 255.0;
        (*display_image)[reverse_index].b = static_cast<float>(p.b) / 255.0;
        (*tmp)[reverse_index] = p;
      }
    }
  }

  grabcut.setInputCloud(tmp);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

  glutInitWindowSize(width, height);
  glutInitWindowPosition(100, 100);

  glutCreateWindow("GrabCut");

  glOrtho(0, width, 0, height, -1, 1);

  // set the background color to black (RGBA)
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glutDisplayFunc(display);
  glutMouseFunc(mouse_callback);
  glutMotionFunc(motion_callback);
  glutKeyboardFunc(keyboard_callback);

  glutMainLoop();

  return 0;
}
