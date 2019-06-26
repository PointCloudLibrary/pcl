/*
 * Software License Agreement (BSD License)
 *
 * 
 *  Class to multiplex signal and slot connections. Qt 3 code from:
 *  http://doc.trolltech.com/qq/qq08-action-multiplexer.html.
 *  Converted to Qt 4 by a.somers@rathenau.nl.
 *  Modified for use in cloud_composer by jpapon@gmail.com  
 * 
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#pragma once

#include <QPointer>

namespace pcl
{
  namespace cloud_composer
  {
    class SignalMultiplexer : public QObject
    {
      Q_OBJECT

      public:
        SignalMultiplexer(QObject *parent = nullptr);

        /**
                Use this connect function instead of QObject::connect() to connect
                an actions activation signal to the receiving, multiplexed object.

                @param sender the sending action or object
                @param signal the signal in the sender object that is connected to
                @param slot the slot in the receiving object that is set through @ref
                        setCurrentObject that the signal should be connected to.

                @see connect(const char *signal, QObject *receiver, const char *slot)
                @see disconnect(QObject *sender, const char *signal, const char *slot)
        */
        void 
        connect (QObject *sender, const char *signal, const char *slot);

        /**
                Disconnects a signal connection from a sending object to a multiplexed
                receiving object.
                @see connect(const char *signal, QObject *receiver, const char *slot)
        */
        bool 
        disconnect (QObject *sender, const char *signal, const char *slot);

        /**
                Use this connect function instead of QObject::connect() to connect
                a multiplexed object's (status) signal to an action object's slot.

                @param signal the signal in the multiplexed sender object that is set through
                        @ref setCurrentObject that the connection should be from
                @param receiver the receiving action or object
                @param slot the slot in the receiving object

                @see connect(QObject *sender, const char *signal, const char *slot)
                @see disconnect(const char *signal, QObject *receiver, const char *slot)
        */
        void 
        connect (const char *signal, QObject *receiver, const char *slot);

        /**
                Disconencts a signal from a multiplexed object to a receiving (action)
                object.
                @see connect(const char *signal, QObject *receiver, const char *slot)
        */
        bool 
        disconnect (const char *signal, QObject *receiver, const char *slot);

        /**
                @returns the object the connections are currently made with.
        */
        QObject 
        *currentObject () const { return object; }

      public Q_SLOTS:
        /**
                Sets the current object the signals that are managed by the
                SignalMultiplexer instance should be connected to. Any connections
                of these signals to the previous object will be disconnected.
                After the connections are hooked to the new object, the
                @ref currentObjectChanged signal will be emitted.

                @param newObject the new object that the signals should be connected to
        */
        void 
        setCurrentObject (QObject *newObject);

      Q_SIGNALS:
        /**
                Emitted when a new object is set to receive the signals managed by
                this SignalMultiplexer instance.
        */
        void 
        currentObjectChanged (QObject* newObject);

      private:
        struct Connection
        {
                QPointer<QObject> sender;
                QPointer<QObject> receiver;
                const char *signal;
                const char *slot;
        };

        void 
        connect (const Connection &conn);
        void 
        disconnect (const Connection &conn);

        QPointer<QObject> object;
        QList<Connection> connections;
    };
  }
}
