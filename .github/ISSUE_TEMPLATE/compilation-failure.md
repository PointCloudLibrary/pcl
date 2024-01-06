---
name: Compilation failure
about: Help! My code doesn't compile (but it should)
title: '[compile error] "Please add a short description here"'
labels: 'status: triage, kind: compile error'
assignees: ''

---

<!--- WARNING: This is an issue tracker. Before opening a new issue make sure you read https://github.com/PointCloudLibrary/pcl/blob/master/CONTRIBUTING.md#using-the-issue-tracker. -->
**Describe the error**

Please paste the compilation results/errors.

**To Reproduce**

Provide a link to a live example, or an unambiguous set of steps to reproduce this bug. A reproducible example helps to provide faster answers. Custom OS, custom PCL configs or custom build systems make the issue difficult to reproduce.
* Best reproducibility: A docker image + code snippets provided here
* Good reproducibility: Common Linux OS + default PCL config + code snippets provided here
* Poor reproducibility: code snippets

Remember to reproduce the error in a clean rebuild (removing all build objects and starting build from scratch)

**Screenshots/Code snippets/Build information**

In order to help explain your problem, please consider adding
* screenshots of the GUI issues
* code snippets: [syntax for code](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#code) with correct language highlights
* debugging information:
  * the output of cmake using `cmake --debug`
  * the failing build command(s) using `VERBOSE=1 make`, `ninja -v` or similar
  * steps taken to narrow down the problem

**Your Environment (please complete the following information):**

 - OS: [e.g. Ubuntu 16.04]
 - Compiler: [:eg GCC 8.1]
 - PCL Version [e.g. 1.10, HEAD] (NB: please ensure you don't have multiple versions available)
 - PCL Type: [e.g. Installed with VCPKG/Installed with apt/Compiled from source]

If PCL was compiled from source or failure in compiling PCL itself:
 - GPU, Kinfu, CUDA enabled? Yes/No
 - List and Version of dependencies used
 - Compilation flags are used

If compiling against PCL:
 - Checked `CMakeLists.txt` for simple errors like missing `find_package` or `target_link_libraries`?
  * [CMakeLists.txt for PCL >= 1.9](https://github.com/kunaltyagi/pcl-cmake-minimum/blob/master/CMakeLists.txt)
  * [CMakeLists.txt for older versions](https://github.com/PointCloudLibrary/pcl/blob/update-issue-templates/doc/tutorials/content/sources/concatenate_clouds/CMakeLists.txt)

**Possible Solution**

Not obligatory, but suggest a fix/reason for the bug. Feel free to create a PR if you feel comfortable.

**Additional context**

Add any other context about the problem here.
