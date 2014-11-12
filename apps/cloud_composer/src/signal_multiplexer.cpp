/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


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

  QList<Connection>::ConstIterator it;
  for (it = connections.begin (); it != connections.end (); ++it)
    disconnect (*it);
  object = newObject;
  for (it = connections.begin (); it != connections.end (); ++it)
    connect (*it);

  ProjectModel* model = dynamic_cast<ProjectModel*> (newObject);
  if (model)
    model->emitAllStateSignals ();
  
  //let the world know about who's on top now
  emit currentObjectChanged (object);
}