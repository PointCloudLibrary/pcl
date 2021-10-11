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
 */

#pragma once

#include <pcl/pcl_config.h>

// needed for the grabber interface / observers
#include <map>
#include <memory>
#include <iostream>
#include <string>
#include <typeinfo>
#include <tuple>
#include <vector>
#include <sstream>
#include <pcl/pcl_macros.h>
#include <pcl/exceptions.h>
#include <boost/signals2.hpp> // for connection, signal, ...

namespace pcl
{

  /** \brief Grabber interface for PCL 1.x device drivers
    * \author Suat Gedikli <gedikli@willowgarage.com>
    * \ingroup io
    */
  class PCL_EXPORTS Grabber
  {
    public:
      /**
       * \brief Default ctor
       */
      Grabber() = default;

      /**
       * \brief No copy ctor since Grabber can't be copied
       */
      Grabber(const Grabber&) = delete;

      /**
       * \brief No copy assign operator since Grabber can't be copied
       */
      Grabber& operator=(const Grabber&) = delete;

      /**
       * \brief Move ctor
       */
      Grabber(Grabber&&) = default;

      /**
       * \brief Move assign operator
       */
      Grabber& operator=(Grabber&&) = default;

      /** \brief virtual destructor. */
      #if defined(_MSC_VER)
        virtual inline ~Grabber () noexcept {}
      #else
        virtual inline ~Grabber () noexcept = default;
      #endif

      /** \brief registers a callback function/method to a signal with the corresponding signature
        * \param[in] callback: the callback function/method
        * \return Connection object, that can be used to disconnect the callback method from the signal again.
        */
      template<typename T> boost::signals2::connection
      registerCallback (const std::function<T>& callback);

      /** \brief indicates whether a signal with given parameter-type exists or not
        * \return true if signal exists, false otherwise
        */
      template<typename T> bool
      providesCallback () const noexcept;

      /** \brief For devices that are streaming, the streams are started by calling this method.
        *        Trigger-based devices, just trigger the device once for each call of start.
        */
      virtual void
      start () = 0;

      /** \brief For devices that are streaming, the streams are stopped.
        *        This method has no effect for triggered devices.
        */
      virtual void
      stop () = 0;

      /** \brief For devices that are streaming, stopped streams are started and running stream are stopped.
        *        For triggered devices, the behavior is not defined.
        * \return true if grabber is running / streaming. False otherwise.
        */
      inline bool
      toggle ();

      /** \brief returns the name of the concrete subclass.
        * \return the name of the concrete driver.
        */
      virtual std::string
      getName () const = 0;

      /** \brief Indicates whether the grabber is streaming or not. This value is not defined for triggered devices.
        * \return true if grabber is running / streaming. False otherwise.
        */
      virtual bool
      isRunning () const = 0;

      /** \brief returns fps. 0 if trigger based. */
      virtual float
      getFramesPerSecond () const = 0;

    protected:

      virtual void
      signalsChanged () { }

      template<typename T> boost::signals2::signal<T>*
      find_signal () const noexcept;

      template<typename T> int
      num_slots () const noexcept;

      template<typename T> void
      disconnect_all_slots ();

      template<typename T> void
      block_signal ();

      template<typename T> void
      unblock_signal ();

      inline void
      block_signals ();

      inline void
      unblock_signals ();

      template<typename T> boost::signals2::signal<T>*
      createSignal ();

      std::map<std::string, std::unique_ptr<boost::signals2::signal_base>> signals_;
      std::map<std::string, std::vector<boost::signals2::connection> > connections_;
      std::map<std::string, std::vector<boost::signals2::shared_connection_block> > shared_connections_;
  } ;

  bool
  Grabber::toggle ()
  {
    if (isRunning ())
    {
      stop ();
    } else
    {
      start ();
    }
    return isRunning ();
  }

  template<typename T> boost::signals2::signal<T>*
  Grabber::find_signal () const noexcept
  {
    using Signal = boost::signals2::signal<T>;

    const auto signal_it = signals_.find (typeid (T).name ());
    if (signal_it != signals_.end ())
    {
      return (static_cast<Signal*> (signal_it->second.get ()));
    }
    return nullptr;
  }

  template<typename T> void
  Grabber::disconnect_all_slots ()
  {
    const auto signal = find_signal<T> ();
    if (signal != nullptr)
    {
      signal->disconnect_all_slots ();
    }
  }

  template<typename T> void
  Grabber::block_signal ()
  {
    if (connections_.find (typeid (T).name ()) != connections_.end ())
      for (auto &connection : shared_connections_[typeid (T).name ()])
        connection.block ();
  }

  template<typename T> void
  Grabber::unblock_signal ()
  {
    if (connections_.find (typeid (T).name ()) != connections_.end ())
      for (auto &connection : shared_connections_[typeid (T).name ()])
        connection.unblock ();
  }

  void
  Grabber::block_signals ()
  {
    for (const auto &signal : signals_)
      for (auto &connection : shared_connections_[signal.first])
        connection.block ();
  }

  void
  Grabber::unblock_signals ()
  {
    for (const auto &signal : signals_)
      for (auto &connection : shared_connections_[signal.first])
        connection.unblock ();
  }

  template<typename T> int
  Grabber::num_slots () const noexcept
  {
    const auto signal = find_signal<T> ();
    if (signal != nullptr)
    {
      return static_cast<int> (signal->num_slots ());
    }
    return 0;
  }

  template<typename T> boost::signals2::signal<T>*
  Grabber::createSignal ()
  {
    using Signal = boost::signals2::signal<T>;
    using Base = boost::signals2::signal_base;
    // DefferedPtr serves 2 purposes:
    // * allows MSVC to copy it around, can't do that with unique_ptr<T>
    // * performs dynamic allocation only when required. If the key is found, this
    //   struct is a no-op, otherwise it allocates when implicit conversion operator
    //   is called inside emplace/try_emplace
    struct DefferedPtr {
      operator std::unique_ptr<Base>() const { return std::make_unique<Signal>(); }
    };
    // TODO: remove later for C++17 features: structured bindings and try_emplace
    #ifdef __cpp_structured_bindings
      const auto [iterator, success] =
    #else
      typename decltype(signals_)::const_iterator iterator;
      bool success;
      std::tie (iterator, success) =
    #endif

    #ifdef __cpp_lib_map_try_emplace
      signals_.try_emplace (
    #else
      signals_.emplace (
    #endif
                         std::string (typeid (T).name ()), DefferedPtr ());
    if (!success)
    {
      return nullptr;
    }
    return static_cast<Signal*> (iterator->second.get ());
  }

  template<typename T> boost::signals2::connection
  Grabber::registerCallback (const std::function<T> & callback)
  {
    const auto signal = find_signal<T> ();
    if (signal == nullptr)
    {
      std::stringstream sstream;

      sstream << "no callback for type:" << typeid (T).name ();

      PCL_THROW_EXCEPTION (pcl::IOException, "[" << getName () << "] " << sstream.str ());
      //return (boost::signals2::connection ());
    }
    boost::signals2::connection ret = signal->connect (callback);

    connections_[typeid (T).name ()].push_back (ret);
    shared_connections_[typeid (T).name ()].push_back (boost::signals2::shared_connection_block (connections_[typeid (T).name ()].back (), false));
    signalsChanged ();
    return (ret);
  }

  template<typename T> bool
  Grabber::providesCallback () const noexcept
  {
    return find_signal<T> ();
  }

} // namespace
