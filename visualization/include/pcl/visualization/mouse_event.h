/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef PCL_VISUALIZATION_MOUSE_EVENT_H_
#define	PCL_VISUALIZATION_MOUSE_EVENT_H_

#include <pcl/visualization/keyboard_event.h>

namespace pcl
{
  namespace visualization
  {
    class MouseEvent
    {
      public:
        typedef enum
        {
          MouseMove = 1,
          MouseButtonPress,
          MouseButtonRelease,
          MouseScrollDown,
          MouseScrollUp,
          MouseDblClick
        } Type;

        typedef enum
        {
          NoButton      = 0,
          LeftButton,
          MiddleButton,
          RightButton,
          VScroll /*other buttons, scroll wheels etc. may follow*/
        } MouseButton;

        /** Constructor.
          * \param[in] type   event type
          * \param[in] button The Button that causes the event
          * \param[in] x      x position of mouse pointer at that time where event got fired
          * \param[in] y      y position of mouse pointer at that time where event got fired
          * \param[in] alt    whether the ALT key was pressed at that time where event got fired
          * \param[in] ctrl   whether the CTRL key was pressed at that time where event got fired
          * \param[in] shift  whether the Shift key was pressed at that time where event got fired
          * \param[in] selection_mode whether we are in selection mode
          */
        inline MouseEvent (const Type& type, const MouseButton& button, 
                           unsigned int x, unsigned int y, 
                           bool alt, bool ctrl, bool shift,
                           bool selection_mode = false);

        /**
          * \return type of mouse event
          */
        inline const Type& 
        getType () const;

        /**
          * \brief Sets the mouse event type
          */
        inline void 
        setType (const Type& type);
        
        /**
          * \return the Button that caused the action
          */
        inline const MouseButton& 
        getButton () const;

        /** \brief Set the button that caused the event */
        inline void 
        setButton (const MouseButton& button);

        /**
          * \return the x position of the mouse pointer at that time where the event got fired
          */
        inline unsigned int 
        getX () const;

        /**
          * \return the y position of the mouse pointer at that time where the event got fired
          */
        inline unsigned int 
        getY () const;

        /**
          * \return returns the keyboard modifiers state at that time where the event got fired
          */
        inline unsigned int 
        getKeyboardModifiers () const;

        /**
          * \return selection mode status
          */
        inline bool
        getSelectionMode () const;

      protected:
        Type type_;
        MouseButton button_;
        unsigned int pointer_x_;
        unsigned int pointer_y_;
        unsigned int key_state_;
        bool selection_mode_;
    };

    MouseEvent::MouseEvent (const Type& type, const MouseButton& button,
                            unsigned x, unsigned y, 
                            bool alt, bool ctrl, bool shift,
                            bool selection_mode)
    : type_ (type)
    , button_ (button)
    , pointer_x_ (x)
    , pointer_y_ (y)
    , key_state_ (0)
    , selection_mode_ (selection_mode)
    {
      if (alt)
        key_state_ = KeyboardEvent::Alt;

      if (ctrl)
        key_state_ |= KeyboardEvent::Ctrl;

      if (shift)
        key_state_ |= KeyboardEvent::Shift;
    }

    const MouseEvent::Type& 
    MouseEvent::getType () const
    {
      return (type_);
    }

    void 
    MouseEvent::setType (const Type& type)
    {
      type_ = type;
    }
    
    const MouseEvent::MouseButton& 
    MouseEvent::getButton () const
    {
      return (button_);
    }

    void 
    MouseEvent::setButton (const MouseButton& button)
    {
      button_ = button;
    }
    
    unsigned int 
    MouseEvent::getX () const
    {
      return (pointer_x_);
    }

    unsigned int 
    MouseEvent::getY () const
    {
      return (pointer_y_);
    }

    unsigned int 
    MouseEvent::getKeyboardModifiers () const
    {
      return (key_state_);
    }

    bool
    MouseEvent::getSelectionMode () const
    {
      return (selection_mode_);
    }

  } //namespace visualization
} //namespace pcl

#endif	/* PCL_VISUALIZATION_MOUSE_EVENT_H_ */

