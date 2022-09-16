// C++
#include <iostream>
#include <string>
#include <cinttypes>

// PCL
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/geometry.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/viewport.h>

// VTK
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkObject.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

// Operators
// -----------------------------------------------------------------------------
Viewport::Viewport (vtkSmartPointer<vtkRenderWindow> window, double xmin/*=0.0*/, double ymin/*=0.0*/,
                    double xmax/*=1.0*/, double ymax/*=1.0*/)
{
  renderer_ = vtkSmartPointer<vtkRenderer>::New ();
  renderer_->SetViewport (xmin, ymin, xmax, ymax);
  renderer_->GradientBackgroundOn ();
  renderer_->SetBackground (.1, .1, .1);
  renderer_->SetBackground2 (.5, .6, .7);

  int *size = window->GetSize ();
  double viewport_xmin = size[0] * xmin;
  double viewport_xmax = size[0] * xmax;
//  double viewport_ymin = size[1] * ymin;
  double viewport_ymax = size[1] * ymax;

  // HUD - Camera Name
  camera_hud_actor_ = vtkSmartPointer<vtkTextActor>::New ();
  camera_hud_actor_->GetTextProperty ()->SetFontSize (12);
  camera_hud_actor_->GetTextProperty ()->SetColor (0.8, 0.8, 0.8);
  camera_hud_actor_->GetTextProperty ()->SetJustificationToCentered ();
  camera_hud_actor_->SetPosition ((viewport_xmax - viewport_xmin) / 2, 10);
  renderer_->AddActor2D (camera_hud_actor_);

  // HUD - FPS
  fps_hud_actor_ = vtkSmartPointer<vtkTextActor>::New ();
  fps_hud_actor_->GetTextProperty ()->SetFontSize (12);
  fps_hud_actor_->GetTextProperty ()->SetColor (0.8, 0.8, 0.8);
  fps_hud_actor_->GetTextProperty ()->SetJustificationToRight ();
  fps_hud_actor_->SetInput ("fps");
  fps_hud_actor_->SetPosition ((viewport_xmax - viewport_xmin) - 10, 10);
  renderer_->AddActor2D (fps_hud_actor_);

  // HUD - Points Loaded
  points_hud_actor_ = vtkSmartPointer<vtkTextActor>::New ();
  points_hud_actor_->GetTextProperty ()->SetFontSize (12);
  points_hud_actor_->GetTextProperty ()->SetColor (0.8, 0.8, 0.8);
  points_hud_actor_->GetTextProperty ()->SetJustificationToRight ();
  points_hud_actor_->SetInput ("points/mb");
  points_hud_actor_->SetPosition ((viewport_xmax - viewport_xmin) - 10, viewport_ymax - 20);
  renderer_->AddActor2D (points_hud_actor_);

  // Callback - Viewport Modified - Window Resize
  viewport_modified_callback_ = vtkSmartPointer<vtkCallbackCommand>::New ();
  viewport_modified_callback_->SetCallback (Viewport::viewportModifiedCallback);
  viewport_modified_callback_->SetClientData (this);

  renderer_->AddObserver (vtkCommand::ModifiedEvent, viewport_modified_callback_);

  // Callback - Actor Updates
  viewport_actor_update_callback_ = vtkSmartPointer<vtkCallbackCommand>::New ();
  viewport_actor_update_callback_->SetCallback (Viewport::viewportActorUpdateCallback);
  viewport_actor_update_callback_->SetClientData (this);

  renderer_->AddObserver (vtkCommand::StartEvent, viewport_actor_update_callback_);

  // Callback - FPS
  viewport_hud_callback_ = vtkSmartPointer<vtkCallbackCommand>::New ();
  viewport_hud_callback_->SetCallback (Viewport::viewportHudUpdateCallback);
  viewport_hud_callback_->SetClientData (this);

  renderer_->AddObserver (vtkCommand::EndEvent, viewport_hud_callback_);

  window->AddRenderer (renderer_);

  Scene *scene = Scene::instance ();
  scene->addViewport (this);
}

// Callbacks
// -----------------------------------------------------------------------------

// Viewport Modified
void
Viewport::viewportModifiedCallback (vtkObject* vtkNotUsed (caller), unsigned long int vtkNotUsed (eventId),
                                    void* clientData, void* vtkNotUsed (callData))
{
  auto *viewport = reinterpret_cast<Viewport*> (clientData);
  viewport->viewportModified ();
}

void
Viewport::viewportModified ()
{
  vtkRenderWindow *window = renderer_->GetRenderWindow ();

  int *size = window->GetSize ();
  double *viewport_size = renderer_->GetViewport ();

  double viewport_xmin = size[0] * viewport_size[0];
  double viewport_xmax = size[0] * viewport_size[2];

  camera_hud_actor_->SetPosition ((viewport_xmax - viewport_xmin) / 2, 10);
  fps_hud_actor_->SetPosition ((viewport_xmax - viewport_xmin) - 10, 10);
}

// Viewport Actor Update
void
Viewport::viewportActorUpdateCallback (vtkObject* /*caller*/, unsigned long int vtkNotUsed (eventId), void* clientData,
                                       void* vtkNotUsed (callData))
{
  auto *viewport = reinterpret_cast<Viewport*> (clientData);
  viewport->viewportActorUpdate ();
}

void
Viewport::viewportActorUpdate ()
{
  Scene *scene = Scene::instance ();

  std::vector<Camera*> cameras = scene->getCameras ();

  for (auto &camera : cameras)
  {
    camera->render (renderer_);
  }

  std::vector<Object*> objects = scene->getObjects ();
  for (auto &object : objects)
  {
    object->render (renderer_);
  }
}

// FPS HUD Update
void
Viewport::viewportHudUpdateCallback (vtkObject* vtkNotUsed (caller), unsigned long int vtkNotUsed (eventId),
                                     void* clientData, void* vtkNotUsed (callData))
{
  auto *viewport = reinterpret_cast<Viewport*> (clientData);
  viewport->viewportHudUpdate ();
}

void
Viewport::viewportHudUpdate ()
{
  // HUD - FPS
  double timeInSeconds = renderer_->GetLastRenderTimeInSeconds ();
  char fps_str[50];
  sprintf (fps_str, "%.2f fps", 1.0 / timeInSeconds);
  fps_hud_actor_->SetInput (fps_str);

  // HUD - Points Loaded
  Scene *scene = Scene::instance ();
  std::vector<Object*> objects = scene->getObjects ();

  std::uint64_t points_loaded = 0;
  std::uint64_t data_loaded = 0;
  for (const auto &object : objects)
  {
    const auto cloud = dynamic_cast<const OutofcoreCloud*> (object);
    if (cloud != nullptr)
    {
      points_loaded += cloud->getPointsLoaded ();
      data_loaded += cloud->getDataLoaded ();
    }
  }

  char points_loaded_str[50];
  snprintf (points_loaded_str, sizeof(points_loaded_str),
            "%" PRIu64 " points/%" PRIu64 " mb", points_loaded, data_loaded/1024);
  points_hud_actor_->SetInput (points_loaded_str);
}

