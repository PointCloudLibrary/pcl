###################################################################
#
# Copyright (c) 2006 Frederic Heem, <frederic.heem@telsey.it>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in
# the documentation and/or other materials provided with the
# distribution.
#
# * Neither the name of the Telsey nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
###################################################################
# - Find pcap
# Find the PCAP includes and library
# http://www.tcpdump.org/
#
# The environment variable PCAPDIR allows to specficy where to find
# libpcap in non standard location.
#
# PCAP_INCLUDE_DIRS - where to find pcap.h, etc.
# PCAP_LIBRARIES - List of libraries when using pcap.
# PCAP_FOUND - True if pcap found.


IF(EXISTS $ENV{PCAPDIR})
FIND_PATH(PCAP_INCLUDE_DIR
NAMES
pcap/pcap.h
pcap.h
PATHS
$ENV{PCAPDIR}
NO_DEFAULT_PATH
)

FIND_LIBRARY(PCAP_LIBRARY
NAMES
pcap
PATHS
$ENV{PCAPDIR}
NO_DEFAULT_PATH
)


ELSE(EXISTS $ENV{PCAPDIR})
FIND_PATH(PCAP_INCLUDE_DIR
NAMES
pcap/pcap.h
pcap.h
)

FIND_LIBRARY(PCAP_LIBRARY
NAMES
pcap
)

ENDIF(EXISTS $ENV{PCAPDIR})

SET(PCAP_INCLUDE_DIRS ${PCAP_INCLUDE_DIR})
SET(PCAP_LIBRARIES ${PCAP_LIBRARY})

#Functions
INCLUDE(CheckFunctionExists)
SET(CMAKE_REQUIRED_INCLUDES ${PCAP_INCLUDE_DIRS})
SET(CMAKE_REQUIRED_LIBRARIES ${PCAP_LIBRARIES})

#Is pcap found ?
IF(PCAP_INCLUDE_DIRS AND PCAP_LIBRARIES)
SET( PCAP_FOUND "YES" )
message(STATUS "PCAP found (include: ${PCAP_INCLUDE_DIRS}, lib: ${PCAP_LIBRARIES})")
ELSE()
SET( PCAP_FOUND "NO" )
message(STATUS "PCAP NOT found")
ENDIF(PCAP_INCLUDE_DIRS AND PCAP_LIBRARIES)


MARK_AS_ADVANCED(
PCAP_LIBRARIES
PCAP_INCLUDE_DIRS
)
