/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/default/cuda_executor.hpp>
#include <pcl/experimental/executor/default/inline_executor.hpp>
#include <pcl/experimental/executor/default/omp_executor.hpp>
#include <pcl/experimental/executor/default/sse_executor.hpp>
#include <pcl/experimental/executor/trait/can_prefer.hpp>
#include <pcl/experimental/executor/trait/can_query.hpp>
#include <pcl/experimental/executor/trait/can_require.hpp>
