/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#!/usr/bin/env python
# file: merge_cmake_install.py

import sys, math
import fnmatch
import os
import shutil

if len(sys.argv) != 2: 
  # stop the program and print an error message
  sys.exit("Must provide a cmake binary folder")

base_folder = sys.argv[1]

string_to_remove_debug = "IF(\"${CMAKE_INSTALL_CONFIG_NAME}\" MATCHES \"^([Dd][Ee][Bb][Uu][Gg])$\")"
string_to_remove_release = "IF(\"${CMAKE_INSTALL_CONFIG_NAME}\" MATCHES \"^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$\")"

matches = []
for root, dirnames, filenames in os.walk(base_folder):
  for filename in fnmatch.filter(filenames, 'cmake_install.cmake'):
      matches.append(os.path.join(root, filename))
	  
for one_match in matches:
	#print one_match, "\n"
	shutil.move( one_match, one_match+"~" )
	destination= open( one_match, "w" )
	source= open( one_match+"~", "r" )
	for line in source:
		if string_to_remove_debug in line:
			destination.write( "\n" )
		elif string_to_remove_release in line:
			destination.write( "\n" )
		else:
			destination.write( line )
	source.close()
	destination.close()