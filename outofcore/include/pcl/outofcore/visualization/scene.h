#ifndef PCL_OUTOFCORE_SCENE_H_
#define PCL_OUTOFCORE_SCENE_H_

// PCL
#include "camera.h"
#include "object.h"
#include "outofcore_cloud.h"
#include "viewport.h"

class Object;

class Scene
{
private:

  static Scene *instance_;

  Scene ();
  Scene (const Scene& op);
  Scene&
  operator= (const Scene& op);

public:

  // Singleton
  static Scene*
  instance ()
  {
    if (!Scene::instance_)
      Scene::instance_ = new Scene ();

    return Scene::instance_;
  }

  // Accessors - Cameras
  // -----------------------------------------------------------------------------
  void
  addCamera (Camera *camera);

  std::vector<Camera*>
  getCameras ();

  Camera*
  getCamera (vtkCamera *camera);

  Camera*
  getCamera (std::string name);

  // Accessors - Objects
  // -----------------------------------------------------------------------------
  void
  addObject (Object *object);

  Object*
  getObjectByName (std::string name);

  std::vector<Object*>
  getObjects ();

  // Accessors - Viewports
  // -----------------------------------------------------------------------------

  void
  addViewport (Viewport *viewport);

  std::vector<Viewport*>
  getViewports ();

  void
  lock ()
  {
    render_mutex_.lock ();
  }

  void
  unlock ()
  {
    render_mutex_.unlock ();
  }

private:
  std::vector<Camera*> cameras_;
  std::vector<Viewport*> viewports_;
  std::vector<Object*> objects_;

  boost::mutex render_mutex_;

};

#endif
