.. _executor_design:

Executor Design
------------------------------

In this document we

Why do we need executors?
=================================

Execution is a fundamental concern of C++ programmers. Every piece of every program executes somehow
and somewhere. For example, the iterations of a for loop execute in sequence on the current thread, while a
parallel algorithm may execute in parallel on a pool of threads. A C++ program’s performance depends
critically on the way its execution is mapped onto underlying execution resources. Naturally, the ability to
reason about and control execution is crucial to the needs of performance-conscious programmers.

In general, there is no standard and ubiquitous way for a C++ programmer to control execution, but there
should be. Instead, programmers control execution through diverse and non-uniform facilities which are often
coupled to low-level platform details. This lack of common interface is an obstacle to programmers that wish
to target multiple execution-creating facilities because each must be targeted separately.
For example, consider the obstacles a programmer must overcome when targeting a simple function at one of
many facilities for creating execution:

.. code-block:: cpp

   void parallel_for(int facility, int n, function<void(int)> f) {
      if(facility == OPENMP) {
         #pragma omp parallel for
         for(int i = 0; i < n; ++i) {
            f(i);
         }
      }
      else if(facility == GPU) {
         parallel_for_gpu_kernel<<<n>>>(f);
      }
      else if(facility == THREAD_POOL) {
         global_thread_pool_variable.submit(n, f);
      }
   }

The major obstacles with this approach that executors aim to solve are:

1. **Complexity**

The first obstacle highlighted by this example is that each facility’s unique interface necessitates
an entirely different implementation. As the library introduces new facilities, each introduction intrudes upon
parallel_for’s implementation. Moreover, the problem worsens as the library introduces new functions.
While the maintenance burden of a single simple function like parallel_for might be manageable, consider
the maintenance complexity of the cross product of a set of parallel algorithms with a set of facilities.

2. **Synchronization**

Execution created through different facilities has different synchronization properties.
For example, an OpenMP parallel for loop is synchronous because the spawning thread blocks until the loop
is complete due to an implicit barrier at the end of the parallel region by default. In contrast, the execution
of GPU kernels is typically asynchronous; kernel launches return immediately and the launching thread
continues its execution without waiting for the kernel’s execution to complete. Work submitted to a thread
pool may or may not block the submitting thread. Correct code must account for these synchronization
differences or suffer data races. To minimize the possibility of races, these differences should be exposed by
library interfaces.

3. **Non-Expressivity**

Our parallel_for example restricts its client to a few simple modes of execution
through the use of a single integer choosing which facility to use. These modes are so restrictive that even
simple generalizations are out of reach. For example, suppose the programmer wants to supply their own
thread pool rather than use the global thread pool, or perhaps the global pool augmented with some notion
or priority or affinity? Similarly, perhaps the programmer may wish to target a specific GPU or collection of
GPUs rather than some GPU implied by the surrounding environment. The vocabulary of parallel_for’s
interface is not rich enough to express these subtleties.

What are executors?
=================================

The goal of Executors is to act as a abstraction of the diverse underlying facilities that are responsible
for implementing the execution. This abstraction will introduce a uniform interface for execution across
various facilities.

Some of the terminologies used are:

**Execution Resource**

It is an instance of a hardware/or software facility capable of executing a callable
function object. Different resources may offer a broad array of functionality and semantics as well as exhibit
different performance characteristics of interest to the performance-conscious programmer.
For example, an implementation might expose different processor cores, with potentially non-uniform access to memory,
as separate resources to enable programmers to reason about locality
Examples of an execution resource can range from SIMD vector units accessible in a single thread to
an entire runtime managing a large collection of threads.

A program may require creating execution on multiple different kinds of execution resources, and these
resources may have significantly different capabilities. For example, callable function objects invoked on
a std::thread have the same capabilities as a Standard C++ program, including access to the facilities of the
operating system, file system, network, and similar. By contrast, GPUs do not create standard threads of
execution, and the callable function objects they execute may have limited access to these facilities.

**Execution Context**

It is a program object that represents a specific collection of execution resources and the
execution agents that exist within those resources i.e. an execution context represents a place
where function objects will be executed,
In the design, execution agents are units of execution, and a 1-to-1 mapping exists between an
execution agent and an invocation of a callable function object.
An agent is bound to an execution context, and hence to one or more of the resources that the context
represents.
Typical examples of an execution context are a thread pool or a runtime environment provided by a GPU.

**Execution Agent**

It is a unit of execution of a specific execution context that is mapped to a single invocation
of a callable function on an execution resource. An execution agent can too have different semantics which
are derived from the execution context.
Typical examples of an execution agent are a CPU thread or GPU execution unit.

**Executor**

An executor is an object associated with a specific execution context. It provides one or more execution
functions for creating execution agents from a callable function object. The execution agents created are
bound to the executor’s context, and hence to one or more of the resources that context represents.

Why does PCL need executors?
-----------------------------------------------



References
----------
- `Executors Design Document P0761R1 <http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2018/p0761r2.pdf>`_
