# Contributing to PCL

Please take a moment to review this document in order to make the contribution
process easy and effective for everyone involved.

Following these guidelines helps to communicate that you respect the time of
the developers managing and developing this open source project. In return,
they should reciprocate that respect in addressing your issue or assessing
patches and features.


## Using the issue tracker

The [issue tracker](https://github.com/PointCloudLibrary/pcl/issues) is
the preferred channel for submitting [pull requests](#pull-requests) and
[bug reports](#bugs), but please respect the following
restrictions:

* Please **do not** use the issue tracker for personal support requests. Please
  consider one of the following alternatives instead:
  * [Stack Overflow](https://stackoverflow.com/questions/tagged/point-cloud-library)
  for Q&A as well as support for troubleshooting, installation and debugging. Do
  remember to tag your questions with the tag `point-cloud-library`.
  * [Discord Server](https://discord.gg/JFFMAXS) for live chat with
  other members of the PCL community and casual discussions

<!--
  * Mailing list: The [PCL Google Group](https://groups.google.com/forum/#!forum/point-cloud-library)
-->

* Please **do not** derail or troll issues. Keep the discussion on topic and
  respect the opinions of others.


<a name="pull-requests"></a>
## Pull requests

Good pull requests - patches, improvements, new features - are a fantastic
help. They should remain focused in scope and avoid containing unrelated
commits.

**Please ask first** before embarking on any significant pull request (e.g.
implementing features, refactoring code), otherwise you risk spending a lot of
time working on something that the project's developers might not want to merge
into the project. Please read the [tutorial on writing a new PCL class](https://pcl.readthedocs.io/projects/tutorials/en/latest/writing_new_classes.html) if you want to contribute a
brand new feature.

If you are new to Git, GitHub, or contributing to an open-source project, you
may want to consult the [step-by-step guide on preparing and submitting a pull request](https://github.com/PointCloudLibrary/pcl/wiki/A-step-by-step-guide-on-preparing-and-submitting-a-pull-request).


<a name="checklist"></a>
### Checklist

Please use the following checklist to make sure that your contribution is well
prepared for merging into PCL:

1. Source code adheres to the coding conventions described in [PCL Style Guide](http://pcl.readthedocs.io/projects/advanced/en/latest/pcl_style_guide.html).
   But if you modify existing code, do not change/fix style in the lines that
   are not related to your contribution.

2. Commit history is tidy (no merge commits, commits are [squashed](http://davidwalsh.name/squash-commits-git)
   into logical units).

3. Each contributed file has a [license](#license) text on top.


<a name="bugs"></a>
## Bug reports

A bug is a _demonstrable problem_ that is caused by the code in the repository.
Good bug reports are extremely helpful - thank you!

Guidelines for bug reports:

1. **Check if the issue has been reported** &mdash; use GitHub issue search and
   mailing list archive search.

2. **Check if the issue has been fixed** &mdash; try to reproduce it using the
   latest `master` branch in the repository.

3. **Isolate the problem** &mdash; ideally create a reduced test
   case.

A good bug report shouldn't leave others needing to chase you up for more
information. Please try to be as detailed as possible in your report. What is
your environment? What steps will reproduce the issue? What would you expect to
be the outcome? All these details will help people to fix any potential bugs.

Example:

> Short and descriptive example bug report title
>
> A summary of the issue and the OS environment in which it occurs. If
> suitable, include the steps required to reproduce the bug.
>
> 1. This is the first step
> 2. This is the second step
> 3. Further steps, etc.
>
> Any other information you want to share that is relevant to the issue being
> reported. This might include the lines of code that you have identified as
> causing the bug, and potential solutions (and your opinions on their
> merits).


<a name="license"></a>
## License

PCL is 100% [BSD licensed](LICENSE.txt), and by submitting a patch, you agree to
allow Open Perception, Inc. to license your work under the terms of the BSD
License. The corpus of the license should be inserted as a C++ comment on top
of each `.h` and `.cpp` file:

```cpp
/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */
```

Please note that if the academic institution or company you are affiliated with
does not allow to give up the rights, you may insert an additional copyright
line.
