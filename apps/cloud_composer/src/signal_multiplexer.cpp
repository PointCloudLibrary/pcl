#include <pcl/apps/cloud_composer/signal_multiplexer.h>
#include <pcl/apps/cloud_composer/project_model.h>

pcl::cloud_composer::SignalMultiplexer::SignalMultiplexer (QObject* parent)
  : QObject (parent)
{

}


void
pcl::cloud_composer::SignalMultiplexer::connect (QObject* sender, const char* signal, const char* slot)
{
  Connection conn;
  conn.sender = sender;
  conn.signal = signal;
  conn.slot = slot;

  connections << conn;
  connect (conn);
}


void
pcl::cloud_composer::SignalMultiplexer::connect (const char* signal, QObject* receiver, const char* slot)
{
  Connection conn;
  conn.receiver = receiver;
  conn.signal = signal;
  conn.slot = slot;

  connections << conn;
  connect (conn);
}


bool
pcl::cloud_composer::SignalMultiplexer::disconnect (QObject* sender, const char* signal, const char* slot)
{
  QMutableListIterator<Connection> it (connections);
  while (it.hasNext ())
  {
    Connection conn = it.next();
    if ( (QObject*) conn.sender == sender &&
          qstrcmp (conn.signal, signal) == 0 && qstrcmp (conn.slot, slot) == 0)
    {
      disconnect (conn);
      it.remove();
      return true;
    }
  }
  return false;
}


bool
pcl::cloud_composer::SignalMultiplexer::disconnect (const char* signal, QObject* receiver, const char* slot)
{
  QMutableListIterator<Connection> it (connections);
  while (it.hasNext ())
  {
    Connection conn = it.next();
    if ( (QObject*) conn.receiver == receiver &&
          qstrcmp (conn.signal, signal) == 0 && qstrcmp (conn.slot, slot) == 0)
    {
      disconnect (conn);
      it.remove();
      return true;
    }
  }
  return false;
}


void
pcl::cloud_composer::SignalMultiplexer::connect (const Connection& conn)
{
  if (!object)
    return;
  if (!conn.sender && !conn.receiver)
    return;

  if (conn.sender)
    QObject::connect ( (QObject*) conn.sender, conn.signal, (QObject*) object, conn.slot);
  else
    QObject::connect ( (QObject*) object, conn.signal, (QObject*) conn.receiver, conn.slot);
}


void 
pcl::cloud_composer::SignalMultiplexer::disconnect (const Connection& conn)
{
  if (!object)
    return;
  if (!conn.sender && !conn.receiver)
    return;

  if (conn.sender)
    QObject::disconnect ( (QObject*) conn.sender, conn.signal, (QObject*) object, conn.slot);
  else
    QObject::disconnect ( (QObject*) object, conn.signal, (QObject*) conn.receiver, conn.slot);

}


void 
pcl::cloud_composer::SignalMultiplexer::setCurrentObject (QObject* newObject)
{
  if (newObject == object)
    return;

  for (const auto &connection : connections)
    disconnect (connection);
  object = newObject;
  for (const auto &connection : connections)
    connect (connection);

  ProjectModel* model = dynamic_cast<ProjectModel*> (newObject);
  if (model)
    model->emitAllStateSignals ();
  
  //let the world know about who's on top now
  emit currentObjectChanged (object);
}
