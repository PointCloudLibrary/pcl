#!/usr/bin/env perl
#
#  Copyright 2011-12 ARM Limited
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

#
# NE10 Library : nightly.pl
#

use warnings;
use strict;

# other variables
my $iterations = 2000; # how many iterations each test must go through ?
my $files_list=`cat projectfile | tr '\n' ';'`; #`find . -maxdepth 1 -type f -iname *.prj | grep './'`; # units to be built are listed in "projectfile"
my @files = split( /;/, $files_list ); # array of files to be built
my @built; # list of successfully built units
my @failed; # list of units that failed to build
my @warn; # list of units that did build but returned with an error message or too many warnings

my $units_count = 0;
my $units_succeeded = 0;
my $success_percentage = 0;

# get list of units and build them
foreach my $fl (@files) {
    $units_count ++;
    #print "<".$fl.">\n"; # debug print
    my $make_cmd = "make NE10_$fl.test_r.ex";
    system ( $make_cmd );
    if ( $? != 0 )
    {
      # failed to build
      push(@failed, $fl);
    }
    else
    {
       # built successfully...
       push(@built, $fl);
       $units_succeeded ++;
    }

}

$success_percentage = 100 * $units_succeeded / $units_count;


#get a test log to be stored in the "test_index_tbl"
system ( "./getlog.sh > ./testlog.txt" );
my $platform = `echo \$NE10PLATFORM`;
my $syslog = `cat ./testlog.txt`;
my $testlog; # this will keep the summary text that will be stored in the database
my $ACCEPTABLE_WARNS = 10; # note: this is defined in unit_test_common.h

# try and run perf on the successfully built units

foreach my $success (@built)
{
    my $perf_cmd = "./runperf.sh NE10_$success $iterations";
    system ( $perf_cmd );
    if ( ($? < 0) || ($? > $ACCEPTABLE_WARNS) )
    {
      # an error while running the test
      push(@warn, $success);
    }
}

# print out a summary of this run
if (scalar(@failed) == 0) {
  print "** No Build Failures\n";
} else {
  print "** BUILDS FAILED\n";
  for my $fail (@failed) {
    print "  $fail failed to build\n";
  }
}
if (scalar(@warn) == 0) {
  print "** No Test Failures\n";
} else {
  print "** TESTS FAILED!\n";
  for my $warned (@warn) {
    print "  $warned failed test\n";
  }
}
#print ( $testlog );
