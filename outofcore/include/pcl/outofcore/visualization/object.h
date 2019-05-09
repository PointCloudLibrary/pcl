#pragma once

// C++
#include <map>
#include <mutex>
#include <set>
#include <string>

// VTK
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

//Forward Declaration
class Scene;

class Object
{
public:

  // Operators
  // -----------------------------------------------------------------------------
  Object (std::string name);

  virtual
  ~Object () { }


  // Accessors
  // -----------------------------------------------------------------------------
  std::string
  getName () const;

  void
  setName (std::string name);

  virtual void
  render (vtkRenderer* renderer);

  bool
  hasActor (vtkActor *actor);

  void
  addActor (vtkActor *actor);

  void
  removeActor (vtkActor *actor);

  vtkSmartPointer<vtkActorCollection>
  getActors ();

protected:
  vtkSmartPointer<vtkActorCollection> actors_;
  std::mutex actors_mutex_;

private:

  // Members
  // -----------------------------------------------------------------------------
  std::string name_;
  std::map<vtkActor*, std::set<vtkRenderer*> > associated_renderers_;

};
