// PCL
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/viewport.h>

Scene* Scene::instance_ = NULL;

Scene::Scene ()
{

}

// Accessors - Cameras
// -----------------------------------------------------------------------------
void
Scene::addCamera (Camera *camera)
{
  cameras_.push_back (camera);
}

std::vector<Camera*>
Scene::getCameras ()
{
  return cameras_;
}

Camera*
Scene::getCamera (vtkCamera *camera)
{
  for (const auto &c : cameras_)
  {
    if (c->getCamera ().GetPointer () == camera)
    {
      return c;
    }
  }

  return NULL;
}

Camera*
Scene::getCamera (std::string name)
{
  for (const auto &camera : cameras_)
    if (camera->getName () == name)
      return camera;

  return NULL;
}

// Accessors - Objects
// -----------------------------------------------------------------------------
void
Scene::addObject (Object *object)
{
  objects_.push_back (object);
}

Object*
Scene::getObjectByName (std::string name)
{
  for (const auto &object : objects_)
    if (object->getName () == name)
      return object;

  return NULL;
}

std::vector<Object*>
Scene::getObjects ()
{
  return objects_;
}

//  void removeObject(Object *object)
//  {
//
//  }

// Accessors - Viewports
// -----------------------------------------------------------------------------

void
Scene::addViewport (Viewport *viewport)
{
  viewports_.push_back (viewport);
}

std::vector<Viewport*>
Scene::getViewports ()
{
  return viewports_;
}
