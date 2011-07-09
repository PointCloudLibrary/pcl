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
 * Author: Suat Gedikli (gedikli@willowgarage.com)
 *
 */
#include <string>

#ifndef __KEYBOARD_EVENT_H__
#define	__KEYBOARD_EVENT_H__

namespace pcl
{
  namespace visualization
  {
    class KeyboardEvent
    {
      public:
        static const unsigned int Alt   = 1;
        static const unsigned int Ctrl  = 2;
        static const unsigned int Shift = 4;

        inline KeyboardEvent (bool action, const std::string& key_sym, unsigned char key, bool alt, bool ctrl, bool shift);

        inline bool isAltPressed () const;
        inline bool isCtrlPressed () const;
        inline bool isShiftPressed () const;

        inline unsigned char getKeyCode () const;
        inline const std::string& getKeySym () const;
        inline bool keyDown () const;
        inline bool keyUp () const;
      protected:

        bool action_;
        unsigned int modifiers_;
        unsigned char key_code_;
        std::string key_sym_;
    };

    KeyboardEvent::KeyboardEvent (bool action, const std::string& key_sym, unsigned char key, bool alt, bool ctrl, bool shift)
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

    bool KeyboardEvent::isAltPressed () const
    {
      return (modifiers_ & Alt);
    }

    bool KeyboardEvent::isCtrlPressed () const
    {
      return (modifiers_ & Ctrl);
    }

    bool KeyboardEvent::isShiftPressed () const
    {
      return (modifiers_ & Shift);
    }

    unsigned char KeyboardEvent::getKeyCode () const
    {
      return key_code_;
    }

    const std::string& KeyboardEvent::getKeySym () const
    {
      return key_sym_;
    }

    bool KeyboardEvent::keyDown () const
    {
      return action_;
    }

    bool KeyboardEvent::keyUp () const
    {
      return !action_;
    }  
  } // namespace visualization
} // namespace pcl
#endif	/* __KEYBOARD_EVENT_H__ */

