/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_VISUALIZATION_KEYBOARD_EVENT_H_
#define	PCL_VISUALIZATION_KEYBOARD_EVENT_H_

#include <string>

namespace pcl
{
  namespace visualization
  {
    /** /brief Class representing key hit/release events */
    class KeyboardEvent
    {
      public:
        /** \brief bit patter for the ALT key*/
        static const unsigned int Alt   = 1;
        /** \brief bit patter for the Control key*/
        static const unsigned int Ctrl  = 2;
        /** \brief bit patter for the Shift key*/
        static const unsigned int Shift = 4;

        /** \brief Constructor
          * \param[in] action    true for key was pressed, false for released
          * \param[in] key_sym   the key-name that caused the action
          * \param[in] key       the key code that caused the action
          * \param[in] alt       whether the alt key was pressed at the time where this event was triggered
          * \param[in] ctrl      whether the ctrl was pressed at the time where this event was triggered
          * \param[in] shift     whether the shift was pressed at the time where this event was triggered
          */
        inline KeyboardEvent (bool action, const std::string& key_sym, unsigned char key, 
                              bool alt, bool ctrl, bool shift);

        /**
          * \return   whether the alt key was pressed at the time where this event was triggered
          */
        inline bool 
        isAltPressed () const;
        
        /**
          * \return whether the ctrl was pressed at the time where this event was triggered
          */
        inline bool 
        isCtrlPressed () const;
        
        /**
          * \return whether the shift was pressed at the time where this event was triggered
          */
        inline bool 
        isShiftPressed () const;

        /**
          * \return the ASCII Code of the key that caused the event. If 0, then it was a special key, like ALT, F1, F2,... PgUp etc. Then the name of the key is in the keysym field.
          */
        inline unsigned char 
        getKeyCode () const;
        
        /**
          * \return name of the key that caused the event
          */
        inline const std::string& 
        getKeySym () const;
        
        /**
          * \return true if a key-press caused the event, false otherwise
          */
        inline bool 
        keyDown () const;
        
        /**
          * \return true if a key-release caused the event, false otherwise
          */
        inline bool 
        keyUp () const;

      protected:

        bool action_;
        unsigned int modifiers_;
        unsigned char key_code_;
        std::string key_sym_;
    };

    KeyboardEvent::KeyboardEvent (bool action, const std::string& key_sym, unsigned char key, 
                                  bool alt, bool ctrl, bool shift)
      : action_ (action)
      , modifiers_ (0)
      , key_code_(key)
      , key_sym_ (key_sym)
    {
      if (alt)
        modifiers_ = Alt;

      if (ctrl)
        modifiers_ |= Ctrl;

      if (shift)
        modifiers_ |= Shift;
    }

    bool 
    KeyboardEvent::isAltPressed () const
    {
      return (modifiers_ & Alt) != 0;
    }

    bool 
    KeyboardEvent::isCtrlPressed () const
    {
      return (modifiers_ & Ctrl) != 0;
    }

    bool 
    KeyboardEvent::isShiftPressed () const
    {
      return (modifiers_ & Shift) != 0;
    }

    unsigned char 
    KeyboardEvent::getKeyCode () const
    {
      return (key_code_);
    }

    const std::string& 
    KeyboardEvent::getKeySym () const
    {
      return (key_sym_);
    }

    bool 
    KeyboardEvent::keyDown () const
    {
      return (action_);
    }

    bool 
    KeyboardEvent::keyUp () const
    {
      return (!action_);
    }  
  } // namespace visualization
} // namespace pcl

#endif	/* PCL_VISUALIZATION_KEYBOARD_EVENT_H_ */

