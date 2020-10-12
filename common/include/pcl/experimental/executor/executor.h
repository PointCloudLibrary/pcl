/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
 *  Author: Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#pragma once

#include <pcl/experimental/executor/best_fit.hpp>
#include <pcl/experimental/executor/cuda_executor.hpp>
#include <pcl/experimental/executor/inline_executor.hpp>
#include <pcl/experimental/executor/omp_executor.hpp>
#include <pcl/experimental/executor/property.h>
#include <pcl/experimental/executor/sse_executor.hpp>
#include <pcl/experimental/executor/type_trait.h>

/**
 * \todo
 * 1. Enable executors to use the specified allocator
 * 2. Look Sender & Receivers from proposal to allow error handling
 * and cancellation
 * 3. Look into enabling asynchronous behaviour using futures and promises
 * 4. Introduce more properties
 * 5. Create a thread pool executor
 * 6. Currently, only derived executors are supported in PCL functions, investigate
 * allowing non derived-executors through execution policies like std::execution::seq
 * & std::execution::par
 */
