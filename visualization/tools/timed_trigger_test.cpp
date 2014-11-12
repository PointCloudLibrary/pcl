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


#include <iostream>
#include <pcl/common/time_trigger.h>
#include <pcl/common/time.h>
#include <pcl/visualization/boost.h>

using namespace std;
using namespace pcl;

double global_time;

void callback ()
{
  static double last_time = pcl::getTime ();
  double elapsed = pcl::getTime () - last_time;
  last_time = pcl::getTime ();
  cout << "global fn: " << pcl::getTime () - global_time << " :: " << elapsed << endl;
  boost::this_thread::sleep(boost::posix_time::microseconds(1000));
}

class Dummy
{
  public:
    void myTimer ()
    {
      static double last_time = pcl::getTime ();
      double elapsed = pcl::getTime () - last_time;
      last_time = pcl::getTime ();
      cout << "member fn: " << pcl::getTime () - global_time << " :: " << elapsed << endl;
    }
};

int main ()
{
  TimeTrigger trigger (10.0, callback);
  Dummy dummy;
  global_time = pcl::getTime ();
  trigger.start ();
  boost::this_thread::sleep(boost::posix_time::seconds(2));
  trigger.registerCallback ( boost::bind(&Dummy::myTimer, dummy));
  boost::this_thread::sleep(boost::posix_time::seconds(3));
  trigger.setInterval (0.2);
  boost::this_thread::sleep(boost::posix_time::seconds(2));
  return 0;
}
