/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012, Urban Robotics, Inc.
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

/*
  This code defines the octree used for point storage at Urban Robotics. Please
  contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions.
  http://www.urbanrobotics.net/
*/
#ifndef PCL_OUTOFCORE_OCTREE_EXCEPTIONS_H_
#define PCL_OUTOFCORE_OCTREE_EXCEPTIONS_H_

namespace pcl
{
  namespace outofcore
  {
    /** \brief OctreeException class is the class handing exceptions for the UR out-of-core code. This will likely be deprecated and integrated with PCL's error/console output and exceptions*/
    class OctreeException : public std::exception
    {
      public:
        enum OCTREE_ERRORS
          {
            OCT_BAD_EXTENTION, // A root was specified without an extention of .oct_idx
            OCT_CHILD_EXISTS,  // A root was specified that already has a folder named [0-9]. You cannot currently overwrite a tree in place. Yau can, however, open the tree using constructor octree(path, loadAll) adnd add points within the current BB
            OCT_MISSING_IDX,   // A node was trying to load and could not find a file named .oct_idx. the tree is now corrupted
            OCT_MISSING_DIR,   // Could not find specified root when trying to load a tree from disk.
            OCT_BAD_PATH,      // Generic file not found or could not create file message.
            OCT_BAD_PARENT,    // A inner node was created without a parent. You should not see this error.
            OCT_PARSE_FAILURE
          };

        OctreeException (const OCTREE_ERRORS e)
        {
          switch (e)
          {
            case OCT_BAD_EXTENTION:
              m_msg = "A root was specified without an extention of .oct_idx";
              break;
            case OCT_CHILD_EXISTS:
              m_msg = "A root was specified that already has a folder named [0-9]. You cannot currently overwrite a tree in place. You can, however, open the tree using constructor octree(path, loadAll) adnd add points within the current BB";
              break;
            case OCT_MISSING_IDX:
              m_msg = "A node was trying to load and could not find a file named .oct_idx. the tree is now corrupted";
              break;
            case OCT_MISSING_DIR:
              m_msg = "Could not find specified root when trying to load a tree from disk.";
              break;
            case OCT_BAD_PATH:
              m_msg = "Generic file not found or could not create file message.";
              break;
            case OCT_BAD_PARENT:
              m_msg = "A inner node was created without a parent. You should not see this error.";
              break;
            case OCT_PARSE_FAILURE:
              m_msg = "";
              break;
          }
        }

        OctreeException (const std::string& msg)
        {
          m_msg = msg;
        }

        ~OctreeException () throw () { }

        virtual const char*
        what () const throw ()
        {
          return m_msg.c_str ();
        }

      private:
        std::string m_msg;
    };
  }
}

#endif //PCL_OUTOFCORE_OCTREE_EXCEPTIONS_H_
