/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
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
 * $Id$
 *
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "pcl/surface/poisson/cmd_line_parser.h"

namespace pcl
{
  namespace surface
  {

#ifdef WIN32
    int strcasecmp(char* c1,char* c2)
    { return _stricmp(c1,c2);}
#endif

    cmdLineReadable::cmdLineReadable (void)
    {
      set = 0;
    }
    cmdLineReadable::~cmdLineReadable (void)
    {
      ;
    }
    int
    cmdLineReadable::read (char**, int)
    {
      set = 1;
      return 0;
    }

    cmdLineInt::cmdLineInt (void)
    {
      value = 0;
    }
    cmdLineInt::cmdLineInt (const int& v)
    {
      value = v;
    }
    int
    cmdLineInt::read (char** argv, int argc)
    {
      if (argc > 0)
      {
        value = atoi (argv[0]);
        set = 1;
        return 1;
      }
      else
      {
        return 0;
      }
    }
    cmdLineFloat::cmdLineFloat (void)
    {
      value = 0;
    }
    cmdLineFloat::cmdLineFloat (const float& v)
    {
      value = v;
    }
    int
    cmdLineFloat::read (char** argv, int argc)
    {
      if (argc > 0)
      {
        value = (float)atof (argv[0]);
        set = 1;
        return 1;
      }
      else
      {
        return 0;
      }
    }
    cmdLineString::cmdLineString (void)
    {
      value = NULL;
    }
    cmdLineString::~cmdLineString (void)
    {
      if (value)
      {
        delete[] value;
        value = NULL;
      }
    }
    int
    cmdLineString::read (char** argv, int argc)
    {
      if (argc > 0)
      {
        value = new char[strlen (argv[0]) + 1];
        strcpy (value, argv[0]);
        set = 1;
        return 1;
      }
      else
      {
        return 0;
      }
    }
    cmdLinePoint3D::cmdLinePoint3D (void)
    {
      value.coords[0] = value.coords[1] = value.coords[2] = 0;
    }
    cmdLinePoint3D::cmdLinePoint3D (const Point3D<float>& v)
    {
      value.coords[0] = v.coords[0];
      value.coords[1] = v.coords[1];
      value.coords[2] = v.coords[2];
    }
    cmdLinePoint3D::cmdLinePoint3D (const float& v0, const float& v1, const float& v2)
    {
      value.coords[0] = v0;
      value.coords[1] = v1;
      value.coords[2] = v2;
    }
    int
    cmdLinePoint3D::read (char** argv, int argc)
    {
      if (argc > 2)
      {
        value.coords[0] = (float)atof (argv[0]);
        value.coords[1] = (float)atof (argv[1]);
        value.coords[2] = (float)atof (argv[2]);
        set = 1;
        return 3;
      }
      else
      {
        return 0;
      }
    }

    char*
    GetFileExtension (char* fileName)
    {
      char* fileNameCopy;
      char* ext = NULL;
      char* temp;

      fileNameCopy = new char[strlen (fileName) + 1];
      assert(fileNameCopy);
      strcpy (fileNameCopy, fileName);
      temp = strtok (fileNameCopy, ".");
      while (temp != NULL)
      {
        if (ext != NULL)
        {
          delete[] ext;
        }
        ext = new char[strlen (temp) + 1];
        assert(ext);
        strcpy (ext, temp);
        temp = strtok (NULL, ".");
      }
      delete[] fileNameCopy;
      return ext;
    }

    void
    cmdLineParse (int argc, char **argv, char** names, int num, cmdLineReadable** readable, int dumpError)
    {
      int i, j;

      while (argc > 0)
      {
        if (argv[0][0] == '-' && argv[0][1] == '-')
        {
          for (i = 0; i < num; i++)
          {
            if (!strcmp (&argv[0][2], names[i]))
            {
              argv++, argc--;
              j = readable[i]->read (argv, argc);
              argv += j, argc -= j;
              break;
            }
          }
          if (i == num)
          {
            if (dumpError)
            {
              fprintf (stderr, "invalid option: %s\n", *argv);
              fprintf (stderr, "possible options are:\n");
              for (i = 0; i < num; i++)
              {
                fprintf (stderr, "  %s\n", names[i]);
              }
            }
            argv++, argc--;
          }
        }
        else
        {
          if (dumpError)
          {
            fprintf (stderr, "invalid option: %s\n", *argv);
            fprintf (stderr, "  options must start with a \'--\'\n");
          }
          argv++, argc--;
        }
      }
    }
  }
}
