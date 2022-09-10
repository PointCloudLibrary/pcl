// -*- C++ -*-
//
// METS lib source file - observer.h
//
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2006-2012, Mirko Maischberger <mirko.maischberger@gmail.com>
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

#ifndef METS_OBSERVER_HH_
#define METS_OBSERVER_HH_

#include <set>
#include <algorithm> 

namespace mets {

  template<typename observed_subject>
  class observer; // forward declaration
  
  template<typename observed_subject>
  class subject; // forward declaration
  
  ///
  /// @brief Functor class to update observers with a for_each,
  ///        only intended for internal use.
  ///
  template<typename observed_subject>
  class update_observer 
  {
  public:
    /// @brief Ctor.
    update_observer(observed_subject* who) : who_m(who) {}
    update_observer() = delete;
    /// @brief Subscript operator to update an observer.
    void 
    operator()(observer<observed_subject>* o) { o->update(who_m); }
  private:
    observed_subject* who_m;
  };
  
  ///
  /// @brief template class for subjects (cfr. Observer Design Pattern).
  ///
  /// You must declare the subject of the observations with:
  ///
  ///   class my_observed_sbj : public subject<my_observed_sbj>
  ///
  /// Than you should call notify() manually or automatically 
  /// from every method that changes the subject status.
  ///
  ///  Only attached observers (cfr. attach() and detach() methods)
  ///  will be notified.
  ///
  template<typename observed_subject>
  class subject
  {
  public:
    virtual
    ~subject() = default;;
    /// @brief Attach a new observer to this subject.
    ///
    /// @param o: a new observer for this subject.
    ///           if the observer was already present 
    ///           nothing happens.
    virtual void
    attach(observer<observed_subject>& o);
    /// @brief Detach a new observer to this subject.
    ///
    /// @param o: observer to detach from this subject.
    ///           if the observer "o" was not present
    ///           nothing happens.
    virtual void
    detach(observer<observed_subject>& o);
    /// @brief Notify all attached observers.
    ///
    /// When this method is called every 
    /// observed_subject#update method is called
    /// and "this" subject is passed as a param.
    /// 
    virtual void
    notify();
  protected:
    subject();
    std::set<observer<observed_subject>*> observers_m;
  };
  
  ///
  /// @brief Template base class for the observers of some observed_subject
  /// 
  /// You should declare a new observer type of some my_subject this way:
  ///
  ///   class my_observer : public observer<my_subject>
  ///
  /// Every time notify() is called on the subject every attached
  /// observer is updated.
  ///
  template<typename observed_subject>
  class observer 
  {
  public:
    virtual
    ~observer() = default;;
    /// @brief This method is automatically called when this
    ///        observer is attached to a "notified" subject.
    ///
    /// @param subject: The subject that was notified and that
    ///                 called our update method.
    virtual void
    update(observed_subject*) = 0;
  protected:
    observer() = default;;
  };
  

  // Implementation of the template methods in STL
  
  template<typename observed_subject>
  subject<observed_subject>::subject() 
    : observers_m() { }
  
  template<typename observed_subject>
  void 
  subject<observed_subject>::attach(observer<observed_subject>& o)
  { observers_m.insert(&o); }
  
  template<typename observed_subject>
  void
  subject<observed_subject>::detach(observer<observed_subject>& o)
  { observers_m.erase(&o); }
  
  template<typename observed_subject>
  void
  subject<observed_subject>::notify() 
  {
    // upcast the object to the real observer_subject type
    auto* real_subject = static_cast<observed_subject*>(this);
    std::for_each(observers_m.begin(), observers_m.end(), 
		  update_observer<observed_subject>(real_subject));
  }
  
}

#endif // METS_OBSERVER_HH_
