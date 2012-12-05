/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012
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

/* SynchronizedQueue Template, taken from
 * http://stackoverflow.com/questions/10139251/shared-queue-c
 */

#ifndef SYNCHRONIZED_QUEUE_H_
#define SYNCHRONIZED_QUEUE_H_

#include <queue>

namespace pcl {

template<typename T>
class SynchronizedQueue {
public:

	SynchronizedQueue() {
		RequestToEnd = false;
		EnqueueData = true;
	}
	void Enqueue(const T& data) {
		boost::unique_lock<boost::mutex> lock(m_mutex);

		if (EnqueueData) {
			m_queue.push(data);
			m_cond.notify_one();
		}

	}

	bool Dequeue(T& result) {
		boost::unique_lock<boost::mutex> lock(m_mutex);

		while (m_queue.empty() && (!RequestToEnd)) {
			m_cond.wait(lock);
		}

		if (RequestToEnd) {
			DoEndActions();
			return false;
		}

		result = m_queue.front();
		m_queue.pop();

		return true;
	}

	void StopQueue() {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        RequestToEnd =  true;
        m_cond.notify_one();
	}

	int Size() {
		boost::unique_lock<boost::mutex> lock(m_mutex);
		return m_queue.size();

	}

private:
	void DoEndActions() {
		EnqueueData = false;

		while (!m_queue.empty()) {
			m_queue.pop();
		}
	}

	std::queue<T> m_queue;              // Use STL queue to store data
	boost::mutex m_mutex;               // The mutex to synchronise on
	boost::condition_variable m_cond;            // The condition to wait for

	bool RequestToEnd;
	bool EnqueueData;
};
}
#endif /* SYNCHRONIZED_QUEUE_H_ */
