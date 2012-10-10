/**
 * threadpool.h
 *
 *  Created on: Dec 11, 2010
 *      Author: Tomer Heber (heber.tomer@gmail.com).
 */

#ifndef THREADPOOL_H_
#define THREADPOOL_H_

struct threadpool;


/**
 * This function creates a newly allocated thread pool.
 *
 * @param num_of_threads The number of worker thread used in this pool.
 * @return On success returns a newly allocated thread pool, on failure NULL is returned.
 */
struct threadpool* threadpool_init(int num_of_threads);

/**
 * This function adds a routine to be exexuted by the threadpool at some future time.
 *
 * @param pool The thread pool structure.
 * @param routine The routine to be executed.
 * @param data The data to be passed to the routine.
 * @param blocking The threadpool might be overloaded if blocking != 0 the operation will block until it is possible to add the routine to the thread pool. If blocking is 0 and the thread pool is overloaded, the call to this function will return immediately.
 *
 * @return 0 on success.
 * @return -1 on failure.
 * @return -2 when the threadpool is overloaded and blocking is set to 0 (non-blocking).
 */
int threadpool_add_task(struct threadpool *pool, void (*routine)(void*), void *data, int blocking);

/**
 * This function stops all the worker threads (stop & exit). And frees all the allocated memory.
 * In case blocking != 0 the call to this function will block until all worker threads have exited.
 *
 * @param pool The thread pool structure.
 * @param blocking If blocking != 0, the call to this function will block until all worker threads are done.
 */
void threadpool_free(struct threadpool *pool, int blocking);

#endif /* THREADPOOL_H_ */
