/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Suat Gedikli (gedikli@willowgarage.com), Nico Blodow (blodow@cs.tum.edu)
 */

#include "pcl/pcl_config.h"
#ifdef HAVE_OPENNI

#ifndef __PCL_IO_GRABBER__
#define __PCL_IO_GRABBER__

// needed for the grabber interface / observers
#include <map>
#include <iostream>
#include <string>
#include <boost/signals2.hpp>
#include <boost/signals2/slot.hpp>
#include <typeinfo>
#include <vector>
#include <sstream>
#include <pcl/io/pcl_io_exception.h>

namespace pcl
{
  
/** \brief Grabber interface for PCL 1.x device drivers
  * \ingroup io
  */
class Grabber
{
  public:
    /**
     * @brief virtual desctructor.
     * @author Suat Gedikli
     */
    virtual inline ~Grabber () throw ();
    
    /**
     * @brief registers a callback function/method to a signal with the corresponding signature
     * @param callback: the callback function/method
     * @return Connection object, that can be used to disconnect the callback method from the signal again.
     * @author Suat Gedikli
     */
    template<typename T> boost::signals2::connection registerCallback (const boost::function<T>& callback) throw (pcl::PCLIOException);
    
    /**
     * @brief indicates whether a signal with given parameter-type exists or not
     * @return true if signal exists, false otherwise
     * @author Suat Gedikli
     */
    template<typename T> bool providesCallback () const;
    
    /**
     * @brief For devices that are streaming, the streams are started by calling this method.
     *        Trigger-based devices, just trigger the device once for each call of start.
     * @author Suat Gedikli
     */
    virtual void start () throw (pcl::PCLIOException) = 0 ;
    
    /**
     * @brief For devices that are streaming, the streams are stopped.
     *        This method has no effect for triggered devices.
     * @author Suat Gedikli
     */
    virtual void stop () throw (pcl::PCLIOException) = 0;
    
    /**
     * @brief returns the name of the concrete subclass.
     * @return the name of the concrete driver.
     * @author Suat Gedikli
     */
    virtual std::string getName () const = 0;
    
    /**
     * @brief Indicates whether the grabber is streaming or not. This value is not defined for triggered devices.
     * @return true if grabber is running / streaming. False otherwise.
     */
    virtual bool isRunning () const throw (pcl::PCLIOException) = 0;
  protected:
    virtual void signalsChanged () {}
    template<typename T> boost::signals2::signal<T>* find_signal () const;
    template<typename T> int num_slots () const;
    template<typename T> void disconnect_all_slots ();

    template<typename T> boost::signals2::signal<T>* createSignal ();
    std::map<std::string, boost::signals2::signal_base*> signals_;
};

Grabber::~Grabber () throw ()
{
  for (std::map<std::string, boost::signals2::signal_base*>::iterator signal_it = signals_.begin (); signal_it != signals_.end (); ++signal_it)
    delete signal_it->second;
}

template<typename T> boost::signals2::signal<T>* Grabber::find_signal () const
{
  typedef boost::signals2::signal<T> Signal;

  std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (typeid(T).name());
  if (signal_it != signals_.end ()) 
    return (dynamic_cast<Signal*> (signal_it->second));

  return (NULL);
}

template<typename T> void Grabber::disconnect_all_slots ()
{
  typedef boost::signals2::signal<T> Signal;

  if (signals_.find (typeid(T).name()) != signals_.end ()) 
  {
    Signal* signal = dynamic_cast<Signal*> (signals_[typeid(T).name()]);
    signal->disconnect_all_slots ();
  }
}

template<typename T> int Grabber::num_slots () const
{
  typedef boost::signals2::signal<T> Signal;

  // see if we have a signal for this type
  std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (typeid(T).name());
  if (signal_it != signals_.end())
  {
    Signal* signal = dynamic_cast<Signal*> (signal_it->second);
    return (signal->num_slots ());
  }
  return (0);
}

template<typename T> boost::signals2::signal<T>* Grabber::createSignal ()
{
  typedef boost::signals2::signal<T> Signal;

  if (signals_.find (typeid(T).name()) == signals_.end ())
  {
    Signal* signal = new Signal ();
    signals_[typeid(T).name ()] = signal;
    return (signal);
  }
  return (0);
}

template<typename T> boost::signals2::connection Grabber::registerCallback (const boost::function<T> & callback) throw (pcl::PCLIOException)
{
  typedef boost::signals2::signal<T> Signal;
  if (signals_.find (typeid(T).name()) == signals_.end ())
  {
    std::stringstream sstream;
    
    sstream << "no callback for type:" << typeid(T).name();
    /*
    sstream << "registered Callbacks are:" << std::endl;
    for( std::map<std::string, boost::signals2::signal_base*>::const_iterator cIt = signals_.begin (); 
         cIt != signals_.end (); ++cIt)
    {
      sstream << cIt->first << std::endl;
    }*/
    
    THROW_PCL_IO_EXCEPTION ("[%s] %s", getName ().c_str (), sstream.str ().c_str ());
    //return (boost::signals2::connection ());
  }
  Signal* signal = dynamic_cast<Signal*> (signals_[typeid(T).name()]);
  boost::signals2::connection ret = signal->connect (callback);

  signalsChanged ();
  return (ret);
}

template<typename T> bool Grabber::providesCallback () const
{
  if (signals_.find (typeid(T).name()) == signals_.end ())
    return (false);
  return (true);
}

} // namespace

#endif
#endif //HAVE_OPENNI
