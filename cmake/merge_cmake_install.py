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