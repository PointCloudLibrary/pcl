/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <pcl/tracking/tracking.h>

#include <random>

double
pcl::tracking::sampleNormal(double mean, double sigma)
{
  static std::mt19937 rng([] {
    std::random_device rd;
    return rd();
  }());
  std::normal_distribution<> nd(mean, sqrt(sigma));

  return (nd(rng));
}

template<typename StateT>
StateT 
pcl::tracking::weightedAverage(typename PointCloud<StateT>::iterator begin, typename PointCloud<StateT>::iterator end)
{
	StateT wa;
  float wa_roll_sin = 0.0, wa_roll_cos = 0.0, wa_pitch_sin = 0.0, wa_yaw_sin = 0.0, wa_yaw_cos = 0.0;

	for(auto point = begin; point != end; point++)
	{
		wa.x += point->x * point->weight;
		wa.y += point->y * point->weight;
		wa.z += point->z * point->weight;

		if(std::cos(point->pitch) > 0)
		{
			wa_roll_sin += std::sin(point->roll) * point->weight;
			wa_roll_cos += std::cos(point->roll) * point->weight;
			wa_yaw_sin += std::sin(point->yaw) * point->weight;
			wa_yaw_cos += std::cos(point->yaw) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else if(std::cos(point->pitch) < 0)
		{
			wa_roll_sin -= std::sin(point->roll) * point->weight;
			wa_roll_cos -= std::cos(point->roll) * point->weight;
			wa_yaw_sin -= std::sin(point->yaw) * point->weight;
			wa_yaw_cos -= std::cos(point->yaw) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else
		{
			wa_pitch_sin += point->weight;
		}
  }

  wa.roll += std::atan2(wa_roll_sin, wa_roll_cos);
  wa.pitch += std::asin(wa_pitch_sin);
  wa.yaw += std::atan2(wa_yaw_sin, wa_yaw_cos);
  return wa;
};


template<>
pcl::tracking::ParticleXYZRPY 
pcl::tracking::weightedAverage<pcl::tracking::ParticleXYZRPY>(PointCloud<pcl::tracking::ParticleXYZRPY>::iterator begin, PointCloud<pcl::tracking::ParticleXYZRPY>::iterator end)
{
	pcl::tracking::ParticleXYZRPY wa;
  float wa_roll_sin = 0.0, wa_roll_cos = 0.0, wa_pitch_sin = 0.0, wa_yaw_sin = 0.0, wa_yaw_cos = 0.0;

	for(auto point = begin; point != end; point++)
	{
		wa.x += point->x * point->weight;
		wa.y += point->y * point->weight;
		wa.z += point->z * point->weight;

		if(std::cos(point->pitch) > 0)
		{
			wa_roll_sin += std::sin(point->roll) * point->weight;
			wa_roll_cos += std::cos(point->roll) * point->weight;
			wa_yaw_sin += std::sin(point->yaw) * point->weight;
			wa_yaw_cos += std::cos(point->yaw) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else if(std::cos(point->pitch) < 0)
		{
			wa_roll_sin -= std::sin(point->roll) * point->weight;
			wa_roll_cos -= std::cos(point->roll) * point->weight;
			wa_yaw_sin -= std::sin(point->yaw) * point->weight;
			wa_yaw_cos -= std::cos(point->yaw) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else
		{
			wa_pitch_sin += point->weight;
		}
  }

  wa.roll += std::atan2(wa_roll_sin, wa_roll_cos);
  wa.pitch += std::asin(wa_pitch_sin);
  wa.yaw += std::atan2(wa_yaw_sin, wa_yaw_cos);
  return wa;
}


template<>
pcl::tracking::ParticleXYRPY 
pcl::tracking::weightedAverage<pcl::tracking::ParticleXYRPY>(PointCloud<pcl::tracking::ParticleXYRPY>::iterator begin, PointCloud<pcl::tracking::ParticleXYRPY>::iterator end)
{
	pcl::tracking::ParticleXYRPY wa;
  float wa_roll_sin = 0.0, wa_roll_cos = 0.0, wa_pitch_sin = 0.0, wa_yaw_sin = 0.0, wa_yaw_cos = 0.0;

	for(auto point = begin; point != end; point++)
	{
		wa.x += point->x * point->weight;
		wa.y += point->y * point->weight;

		if(std::cos(point->pitch) > 0)
		{
			wa_roll_sin += std::sin(point->roll) * point->weight;
			wa_roll_cos += std::cos(point->roll) * point->weight;
			wa_yaw_sin += std::sin(point->yaw) * point->weight;
			wa_yaw_cos += std::cos(point->yaw) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else if(std::cos(point->pitch) < 0)
		{
			wa_roll_sin -= std::sin(point->roll) * point->weight;
			wa_roll_cos -= std::cos(point->roll) * point->weight;
			wa_yaw_sin -= std::sin(point->yaw) * point->weight;
			wa_yaw_cos -= std::cos(point->yaw) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else
		{
			wa_pitch_sin += point->weight;
		}
  }

  wa.z = 0.0;
  wa.roll += std::atan2(wa_roll_sin, wa_roll_cos);
  wa.pitch += std::asin(wa_pitch_sin);
  wa.yaw += std::atan2(wa_yaw_sin, wa_yaw_cos);
  return wa;
}

template<>
pcl::tracking::ParticleXYRP 
pcl::tracking::weightedAverage<pcl::tracking::ParticleXYRP>(PointCloud<pcl::tracking::ParticleXYRP>::iterator begin, PointCloud<pcl::tracking::ParticleXYRP>::iterator end)
{
	pcl::tracking::ParticleXYRP wa;
  float wa_roll_sin = 0.0, wa_roll_cos = 0.0, wa_pitch_sin = 0.0;

	for(auto point = begin; point != end; point++)
	{
		wa.x += point->x * point->weight;
		wa.y += point->y * point->weight;

		if(std::cos(point->pitch) > 0)
		{
			wa_roll_sin += std::sin(point->roll) * point->weight;
			wa_roll_cos += std::cos(point->roll) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else if(std::cos(point->pitch) < 0)
		{
			wa_roll_sin -= std::sin(point->roll) * point->weight;
			wa_roll_cos -= std::cos(point->roll) * point->weight;
			wa_pitch_sin += std::sin(point->pitch) * point->weight;
		}
		else
		{
			wa_pitch_sin += point->weight;
		}
  }

  wa.z = 0.0;
  wa.roll += std::atan2(wa_roll_sin, wa_roll_cos);
  wa.pitch += std::asin(wa_pitch_sin);
  wa.yaw = 0.0;
  return wa;
}

template<>
pcl::tracking::ParticleXYR 
pcl::tracking::weightedAverage<pcl::tracking::ParticleXYR>(PointCloud<pcl::tracking::ParticleXYR>::iterator begin, PointCloud<pcl::tracking::ParticleXYR>::iterator end)
{
	pcl::tracking::ParticleXYR wa;
  float wa_roll_sin = 0.0, wa_roll_cos = 0.0;

	for(auto point = begin; point != end; point++)
	{
		wa.x += point->x * point->weight;
		wa.y += point->y * point->weight;

		if(std::cos(point->pitch) > 0)
		{
			wa_roll_sin += std::sin(point->roll) * point->weight;
			wa_roll_cos += std::cos(point->roll) * point->weight;
		}
		else if(std::cos(point->pitch) < 0)
		{
			wa_roll_sin -= std::sin(point->roll) * point->weight;
			wa_roll_cos -= std::cos(point->roll) * point->weight;
		}
		else
		{
		}
  }

  wa.z = 0.0;
  wa.roll += std::atan2(wa_roll_sin, wa_roll_cos);
  wa.pitch = 0.0;
  wa.yaw = 0.0;
  return wa;
}

template<>
pcl::tracking::ParticleXYZR 
pcl::tracking::weightedAverage<pcl::tracking::ParticleXYZR>(PointCloud<pcl::tracking::ParticleXYZR>::iterator begin, PointCloud<pcl::tracking::ParticleXYZR>::iterator end)
{
	pcl::tracking::ParticleXYZR wa;
  float wa_roll_sin = 0.0, wa_roll_cos = 0.0;

	for(auto point = begin; point != end; point++)
	{
		wa.x += point->x * point->weight;
		wa.y += point->y * point->weight;
		wa.z += point->z * point->weight;

		if(std::cos(point->pitch) > 0)
		{
			wa_roll_sin += std::sin(point->roll) * point->weight;
			wa_roll_cos += std::cos(point->roll) * point->weight;
		}
		else if(std::cos(point->pitch) < 0)
		{
			wa_roll_sin -= std::sin(point->roll) * point->weight;
			wa_roll_cos -= std::cos(point->roll) * point->weight;
		}
		else
		{
		}
  }

  wa.roll += std::atan2(wa_roll_sin, wa_roll_cos);
  wa.pitch = 0.0;
  wa.yaw = 0.0;
  return wa;
}
