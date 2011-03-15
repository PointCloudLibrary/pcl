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

#ifndef __PCL_IO_GRABBER__
#define __PCL_IO_GRABBER__

// needed for the grabber interface / observers
#include <map>
#include <iostream>
#include <boost/signals2.hpp>
#include <boost/signals2/slot.hpp>
#include <typeinfo>

namespace pcl
{

class Grabber
{
	public:
		virtual ~Grabber () {}
		//template<typename T> boost::signals2::connection registerCallback (const boost::function<T>& callback);
		template<typename T> boost::signals2::connection registerCallback (const T& callback);
		virtual unsigned start () = 0;
    virtual void stop () = 0;  
    virtual void signalsChanged () = 0;
    
    // TODO: implement me!
		//virtual bool isRunning () const = 0;
		//virtual bool trigger () = 0;
	protected:
    template<typename T> boost::signals2::signal<T>* find_signal () const;
    template<typename T> int num_slots () const;
    template<typename T> void disconnect_all_slots ();

		template<typename T> bool createCallback ();
		std::map<const std::type_info*, boost::signals2::signal_base*> signals_;
};

template<typename T> boost::signals2::signal<T>* Grabber::find_signal () const
{
  typedef boost::signals2::signal<T> Signal;

  std::map<const std::type_info*, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (&typeid(T));
  if (signal_it != signals_.end ()) 
    return dynamic_cast<Signal*> (signal_it->second);

  return NULL;
}

template<typename T> void Grabber::disconnect_all_slots ()
{
  typedef boost::signals2::signal<T> Signal;

  if (signals_.find (&typeid(T)) != signals_.end ()) 
  {
    Signal* signal = dynamic_cast<Signal*> (signals_[&typeid(T)]);
    signal->disconnect_all_slots ();
  }
}

template<typename T> int Grabber::num_slots () const
{
  typedef boost::signals2::signal<T> Signal;

  // see if we have a signal for this type
  std::map<const std::type_info*, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (&typeid(T));
  if (signal_it != signals_.end())
  {
    Signal* signal = dynamic_cast<Signal*> (signal_it->second);
    return signal->num_slots ();
  }
  return 0;
}

template<typename T> bool Grabber::createCallback ()
{
	typedef boost::signals2::signal<T> Signal;

	if (signals_.find (&typeid(T)) == signals_.end ())
	{
    std::cout << "registering Callback type: " << typeid(T).name() << std::endl;
		signals_[&typeid(T)] = new Signal ();
		return true;
	}
	return false;
}

//template<typename T> boost::signals2::connection Grabber::registerCallback (const boost::function<T> & callback)
//{
//	typedef boost::signals2::signal<T> Signal;
//	if (signals_.find (&typeid(T)) == signals_.end ())
//	{
//    std::cout << "no callback for type: void (" << typeid(T).name() << ")" << std::endl;
//    std::cout << "registered Callbacks are:" << std::endl;
//    for( std::map<const std::type_info*, boost::signals2::signal_base*>::const_iterator cIt = signals_.begin (); cIt != signals_.end (); ++cIt)
//    {
//      std::cout << "void (" << cIt->first->name() << ")" << std::endl;
//    }
//		return boost::signals2::connection ();
//	}
//	Signal* signal = dynamic_cast<Signal*> (signals_[&typeid(T)]);
//  boost::signals2::connection ret = signal->connect (callback);
//
//  signalsChanged ();
//  return ret;
//}

template<typename T> boost::signals2::connection Grabber::registerCallback (const T& callback)
{
	typedef boost::signals2::signal<T> Signal;
	if (signals_.find (&typeid(T)) == signals_.end ())
	{
    std::cout << "no callback for type: void (" << typeid(T).name() << ")" << std::endl;
    std::cout << "registered Callbacks are:" << std::endl;
    for( std::map<const std::type_info*, boost::signals2::signal_base*>::const_iterator cIt = signals_.begin (); cIt != signals_.end (); ++cIt)
    {
      std::cout << "void (" << cIt->first->name() << ")" << std::endl;
    }
		return boost::signals2::connection ();
	}
	Signal* signal = dynamic_cast<Signal*> (signals_[&typeid(T)]);
  boost::signals2::connection ret = signal->connect (callback);

  signalsChanged ();
  return ret;
}

} // namespace

#endif

