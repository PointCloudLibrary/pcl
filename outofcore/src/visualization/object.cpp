// PCL
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>

// Operators
// -----------------------------------------------------------------------------
Object::Object (std::string name): actors_(vtkSmartPointer<vtkActorCollection>::New ()), name_(name)
{
}

// Accessors
// -----------------------------------------------------------------------------
std::string
Object::getName () const
{
  return name_;
}

void
Object::setName (std::string name)
{
  name_ = name;
}

vtkSmartPointer<vtkActorCollection>
Object::getActors ()
{
  return actors_;
}

void
Object::render (vtkRenderer* renderer)
{
  std::lock_guard<std::mutex> lock (actors_mutex_);
  // Iterate over the objects actors
  actors_->InitTraversal ();
  for (vtkIdType i = 0; i < actors_->GetNumberOfItems (); i++)
  {
    vtkActor* actor = actors_->GetNextActor ();

    // If the actor hasn't been added to the renderer add it
    std::set<vtkRenderer*>::iterator renderer_it;
    renderer_it = associated_renderers_[actor].find (renderer);
    if (renderer_it == associated_renderers_[actor].end ())
    {
      associated_renderers_[actor].insert (renderer);
      renderer->AddActor (actor);
    }
  }
}

bool
Object::hasActor (vtkActor *actor)
{
  std::lock_guard<std::mutex> lock (actors_mutex_);

  return actors_->IsItemPresent (actor);
}

void
Object::addActor (vtkActor *actor)
{
//  Scene::instance ()->lock ();
  std::lock_guard<std::mutex> lock (actors_mutex_);

  if (!actors_->IsItemPresent (actor))
    actors_->AddItem (actor);

  // If the actor doesn't exist in the associated_renderers_ map add it
  std::map<vtkActor*, std::set<vtkRenderer*> >::iterator actor_it;
  actor_it = associated_renderers_.find (actor);
  if (actor_it == associated_renderers_.end ())
  {
    associated_renderers_[actor] = std::set<vtkRenderer*> ();
  }
//  Scene::instance ()->unlock ();

//  Scene *scene = Scene::instance();
//  std::vector<Viewport*> viewports = scene->getViewports();
//  for(int i=0; i < viewports.size(); i++)
//  {
//
//    vtkRenderer *renderer = viewports[i]->getRenderer();
//    // If the actor hasn't been added to the renderer add it
//    std::set<vtkRenderer*>::iterator renderer_it;
//    renderer_it = associated_renderers_[actor].find(renderer);
//    if (renderer_it == associated_renderers_[actor].end())
//    {
//      associated_renderers_[actor].insert(renderer);
//      renderer->AddActor(actor);
//    }
//  }
}

void
Object::removeActor (vtkActor *actor)
{
//  Scene::instance ()->lock ();
  //std::cout << "Removing Actor" << std::endl;
  std::lock_guard<std::mutex> lock (actors_mutex_);
  actors_->RemoveItem (actor);

  std::map<vtkActor*, std::set<vtkRenderer*> >::iterator actor_it;
  actor_it = associated_renderers_.find (actor);

  if (actor_it != associated_renderers_.end ())
  {
    for (auto renderer_it = associated_renderers_[actor].cbegin (); renderer_it != associated_renderers_[actor].cend ();
        ++renderer_it)
    {
      (*renderer_it)->RemoveActor (actor);
    }
    associated_renderers_.erase (actor);
  }
  //std::cout << "Removing Actor - DONE" << std::endl;
//  Scene::instance ()->unlock ();
}
