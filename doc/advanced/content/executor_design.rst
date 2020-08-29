.. _executor_design:

Executor Design
------------------------------

In this document we discuss about executors (which are part of C++ standard committee proposals)
their design, implementation and relevance in PCL.

Why do we need executors?
=================================

.. note::
   All of the content from the this section has been taken as is from the proposal
   `P0761R2 <https://wg21.link/P0761R2>`_ by Hoberock et al. with minor modifications where appropriate.

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
=================================

PCL has a a diverse collections of modules with various algorithms. Many of these implementations target
a diverse set of facilities such as SIMD, OpenMP, GPU (CUDA) etc. Since each facility has a unique set
of interfaces which are often coupled with low level implementation details, so some of them are required
to have separate implementations i.e. separate classes:.
The current implementation suffers from a few drawbacks such as:

1. **Divergent Implementation**
The distinct implementations of algorithms leads to disparity in the codebase over time. The more popular
implementation gets bug fixes, new features and undergoes refactoring while the other implementations remain
untouched.
Example: The parallelized version of an algorithm might be more popular so it will get bug fixes overtime while
those bugs continue to persist in the sequential implementation of that algorithm.

2. **Non Uniform API**
The API for one of the implementations might undergoes changes to accommodate interface peculiarities or facility
specific optimizations.
Example: Parallel implementations expose API's to allow configuring the degree of parallelism which is completely
absent from sequential implementations.

3. **Inextensible Design**
The current design doesn't support using new facilities like thread pools, multi-gpu support or nesting
them with one other. To add support for these facilities, completely new implementations will have to be written
for every algorithm.
Example: It isn't possible to run vectorized code (SIMD) while running a parallel implementation which
uses OpenMP.

4. **Code Duplication**
Even if different facilities might require slightly different implementations, a lot of the code can
be shared. Having different implementations just leads to a majority of the code base being duplicated
and only some of the code gets modified in order to adapt to the interface provided by the facility.
Example: Most OpenMP code is quite similar to the sequential implementation with only some additions.
So having separate classed for OpenMP classes is quite redundant.

5. **Maintenance Overhead**
Maintaining several implementations of the same algorithm is a labour and time intensive task.
Porting changes from one implementation to another and propagating bug fixes to all the implementations
is time consuming. Lack of time to propagate the changes will lead to divergent implementations as mentioned
above.

Design Considerations
=================================

The executor design proposal for C++, aims to build a generalized and extensible framework surrounding
executors. This is necessary since it needs to support a wide range of uses cases so as to attempt to cater
everyone's needs, in order for it to be accepted into the standard library.

In PCL, the need for executors is limited to certain features and there is no need for the entire feature set
as mentioned in the proposal. Concepts such asynchronous operations and task based parallelism are not present
or needed in PCL at the moment so creating a design which incorporates all those features would be unnecessary.
The main use cases in PCL are:

1. Provide a uniform API for executing existing algorithms on different facilities giving users the freedom
   to switch between facilities with ease.

2. Reducing code duplication by trying to avoid completely different implementations of the same algorithm

3. Provide a simple and easy to use mechanism to customize the execution context which users can also access.

4. Expose some of the underlying features offered by the various facilities in a standardized manner.

5. Provide a default, implicit mechanism to automatically choose the best facility to run an algorithm.

6. Be extensible enough to allows users to specify their own executors or customize the ones provided by PCL

7. Minimize runtime overhead by exploiting compile time resolution whenever appropriate. This is also ensure all
   errors are caught at compile time, providing guarantee an executor would work if the code compiles successfully.

8. Last but not the least be forward compatible with the upcoming executor design so that PCL is compatible with
   it when it becomes a part of the standard specification.

Accepted Design
=================================

The implemented design in PCL draws heavy influence from some of the implementations which are being
developed in light of the proposal which are:

* `executors-impl <https://github.com/executors/executors-impl>`_

* `cudex <https://github.com/jaredhoberock/cudex>`_

The two main elements of this design are namely executors themselves and  executor properties.


**Executor Design**

As per the proposal the technical definition of an executor is:
An executor should be a `CopyConstructible` and `EqualityComparable` type that provides a function named
`execute` that eagerly submits work on a single execution agent created for it by the executor.

There are two available execution functions in any executor:

1. `execute`
It takes a nullary callable (a callable which takes no arguments and returns void) and executes
the callable on a single execution agent exactly once.

2. `bulk_execute`
It takes a callable (which returns void but takes an index parameter as argument) and a shape which
corresponds to the number of invocations of the callable. This function generates execution agents
equal to the number of invocations in bulk and then each execution agent invokes the callable once.
The index of the execution agent is passed as argument to the callable, so that the callable
knows the invocation index.

The difference between simply calling execute repeatedly and bulk_execute is that bulk_execute
leverages the API of the facility to generate execution agents in bulk which is more efficient then creating
them one by one.

How these executors call the callable internally is dependent on the implementation of each executor
and some aspects of the execution can be customized through properties.

The index passed in bulk execute can be used to internally partition certain parts of the code to only run
on specific indexes.
Example: It can be used to split the iteration of a loop between the execution agents.

**Shape and Index**

The shape and index type will vary depending on the facilities, so a mechanism has been provided to customize
their types. By default it is `std::size_t`.

The shape or index can be specified by a type or an alias for a type inside the executor with the names
`shape_type` and `index_type`. There also exists type traits namely `executor_shape` and `executor_index`
to access the type of the executor's shape or index.

**Executor Properties**

Executor properties are objects associated with an executor. They are used to customize various aspects
of the executor related to execution and are also used to provide guarantees.
The properties which are currently implemented in PCL currently are:

* Blocking

This specifies whether or not execution inside an execution function should wait/block till all
execution agents are done executing. There are 3 mutually exclusive blocking properties
`blocking.always`, `blocking.never` and `blocking.possibly`. Their role can determined by their names
itself. The default is `blocking.always`.

* Allocators

It specifies the allocator to associate with an executor. This property can be used to specify
the preferred allocator when an executor needs to allocate some storage necessary
for execution. The default is the specialization `allocator_t<void>` which indicates to
use the default allocator available in the system.

As of now only these 2 properties are supported in PCL but even they are not fully supported by
the provided executors.

It is compulsory for a property to define a default property, which indicates the property
value even if an executor doesn't explicitly support that property.

**Property Customization Mechanism**

Properties of an executor are specified using the template parameters of an executor class template.
A user may introduce a new property to an executor by defining a property type and
specializing either the `require` or `prefer` and `query` member functions inside the executor.

An executor can be strongly or weakly associated to properties which it supports. This can be
achieved by a call to the require or prefer customization points. This operation might produce a new
executor of a different type. You can also query whether an executor supports a specific property
or not by a call to the query customization point.


**Customizing Executors**

Users are free to create their own executors or customize existing ones by inheriting the ones provided
by PCL. Users can even create their own custom properties add support for them in executors.

As of now only derived executors will work on PCL functions which support the base executor,
using your executor without deriving is not supported in PCL functions. Since this is an advanced
feature and is user dependent, PCL code cannot provide any guarantee that custom executors will work as
expected for PCL functions. Make sure to look and understand the code for the function in which
you are using a custom executor and determine whether the executor will provide the expected results.

**Implementation**

Below we show a simplified version of the inline executor. Look at the Code API for more details.

.. code-block:: cpp

   // Properties are specified as template parameters
   template <typename Blocking = blocking_t::always_t, typename ProtoAllocator = std::allocator<void>>
   struct inline_executor {

      // Shape and index are specified to the custom PCL type uindex_t
      using shape_type = uindex_t ;
      using index_type = uindex_t ;


      // execute invokes the callable exactly once
      template <typename F>
      void execute(F&& f) const {
        f();
      }

      // bulk_execute invokes the callable n times and passes the index of execution agent
      // In case of inline_executor each execution agent is mapped to the same thread is invoked sequentially
      template <typename F, typename... Args>
      void bulk_execute(F&& f, const shape_type& n) const {
        for (shape_type index = 0; index <  n; ++index)
          f(index);
      }

      // The query customization point being specialized to show that the executor supports
      // the blocking.always property
      static constexpr auto query(const blocking_t&) noexcept {
        return blocking_t::always;
      }

      // The require customization point being specialized to allow strongly associating the executor with
      // the blocking.always property
      inline_executor<blocking_t::always_t, ProtoAllocator> require(const blocking_t::always_t&) const {
        return {};
      }
    };


Best Fit Executor
=================================

In most scenarios users of PCL do not care much about performance and simply want their code to run
fast without any additional steps performed from their side. For this reason their was a need for
mechanism called best fit, which automatically selects best possible executor and its customizes its
property in order to give the user good performance out of the box based on their system configurations
and the function they are calling.

The mechanism works be choosing an executor based on the availability of an executor (depends on
hardware/software of a system) and the priority of executors specified in function by PCL
maintainers & contributors which will give good performance. The executor properties can also be
customized on for a better fit for certain scenarios. Besides this runtime checks are also be specified
and on the basis of these runtime checks the executors are further filtered to selected most appropriate
one. The two supported mechanisms in PCL currently are `enable_exec_with_priority` and
`enable_exec_on_desc_priority`. You can read more about them in the code API.

Alternative Designs Considered
=================================

There has been a lot of deliberation and discussion regarding all the design aspects of executors in PCL.
The design has gone through multiple iterations before a consensus was reached. Some of the major alternate
design proposals that were rejected were:

* Tag Dispatching

This was was one of the initial design considerations. It seemed like an attractive choice due to simplicity,
could choose the allowed overload resolution to choose the best option at compile time by allowing
tags to be inherited. These tags would serve as placeholders till executors were standardized.
They lacked all the features of executors and offered no customization. Once executors were even standardized
large parts of the codebase would need to be refactored to support executors.

* Base Executor

The idea was to have single base executor from which all executors would derive from. This would allow
all the common cod to be shared among all the executors. The base executors has a CRTP based design which
is used to access the properties of the derived executor. It also allowed simplification in many areas
such as being able to reference any derived executor. Basic properties of all executors like
copy constructors, overloaded equality operators.
The CRTP mechanism had some restrictions in the sense that there was still a need for templates and
their were concerns regarding explicitly passing all the properties to the base class using CRTP as
it was felt to be unnecessary. Having inheritance also introduces runtime polymorphism which was against
one of the design consideration i.e. have everything compile time and avoid any overhead.
The base executor didn't add a lot besides allowing code sharing of a few common functionality and thus
it was discarded.

* Property Inheritance

In this design, the executor inherits from the properties it supports (which are defined as template parameters
for the executor class). With this design, executors would not need to provide the `require` or `prefer`
and `query` member functions inside the executor for each property it wants to allow association with and
support customizing. This design was not fully explored, so the potential drawbacks are not know completely.
A snippet is available `here <https://godbolt.org/z/zhKM6e>`_. It was decided to not go with this design forward
due to the potential complications, minimal support of properties for executors in PCL and deviation
from the property customization mechanism design implemented in other implementations of the executor design
proposal mentioned above in the document. Once more properties to be used w.r.t PCL are made available and
use cases for these properties comes up. This design can be considered and looked at.

* Unified Shape with Compile Time Support

Since each executor could have a different shape type, a unified shape type would provide a more uniform
API. The compile time support for shapes would also bring performance benefits. The compile time aspect
was ditched since in most scenarios the value of the shape was determined at runtime and rarely at compile
time. The unified shape was also deemed unnecessary since for most executors `std::size_t` or `int` was
sufficient and only a custom shape was needed for GPU for which the shape could be explicitly specified.

Conclusion
=================================

Executors in PCL is a very new concept and will mature as they are more widely used across PCL.
The design will also go though more changes and iterations as feedback from the community is received
and the C++ executor proposal itself evolves.

References
----------
- `Executors Design Document P0761R1 <http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2018/p0761r2.pdf>`_
