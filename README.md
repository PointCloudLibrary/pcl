# Point Cloud Library

<p align="center"><img src="pcl.png" height="100"></p>

[![Release][release-image]][releases]
[![License][license-image]][license]

[release-image]: https://img.shields.io/badge/release-1.13.0-green.svg?style=flat
[releases]: https://github.com/PointCloudLibrary/pcl/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt

Website
-------

The new website is now online at https://pointclouds.org and is open to [contributions](https://github.com/PointCloudLibrary/PointCloudLibrary.github.io) :hammer_and_wrench:.

If you really need access to the old website, please use [the copy made by the internet archive](https://web.archive.org/web/20191017164724/http://www.pointclouds.org/). Please be aware that the website was hacked before and could still be hosting some malicious code.

Continuous integration
----------------------
[ci-latest-build]: https://dev.azure.com/PointCloudLibrary/pcl/_build/latest?definitionId=9&branchName=master
[ci-ubuntu-18.04]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20GCC&jobName=Ubuntu&configuration=Ubuntu%2018.04%20GCC&label=Ubuntu%2018.04%20GCC
[ci-ubuntu-20.04]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Clang&jobName=Ubuntu&configuration=Ubuntu%2020.04%20Clang&label=Ubuntu%2020.04%20Clang
[ci-ubuntu-22.04]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20GCC&jobName=Ubuntu&configuration=Ubuntu%2022.04%20GCC&label=Ubuntu%2022.04%20GCC
[ci-windows-x86]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20MSVC&jobName=Windows%20Build&configuration=Windows%20Build%20x86&label=Windows%20VS2019%20x86
[ci-windows-x64]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20MSVC&jobName=Windows%20Build&configuration=Windows%20Build%20x64&label=Windows%20VS2019%20x64
[ci-macos-11]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Clang&jobName=macOS&configuration=macOS%20Big%20Sur%2011&label=macOS%20Big%20Sur%2011
[ci-macos-12]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Clang&jobName=macOS&configuration=macOS%20Monterey%2012&label=macOS%20Monterey%2012
[ci-docs]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/Documentation?branchName=master
[ci-latest-docs]: https://dev.azure.com/PointCloudLibrary/pcl/_build/latest?definitionId=14&branchName=master

Build Platform           | Status
------------------------ | ------------------------------------------------------------------------------------------------- |
Ubuntu                   | [![Status][ci-ubuntu-18.04]][ci-latest-build] <br> [![Status][ci-ubuntu-20.04]][ci-latest-build] <br> [![Status][ci-ubuntu-22.04]][ci-latest-build] |
Windows                  | [![Status][ci-windows-x86]][ci-latest-build]  <br> [![Status][ci-windows-x64]][ci-latest-build]   |
macOS                    | [![Status][ci-macos-11]][ci-latest-build]  <br> [![Status][ci-macos-12]][ci-latest-build]   |
Documentation            | [![Status][ci-docs]][ci-latest-docs] |

Community
---------
[![Discord][discord-image]][discord-server]
[![StackOverflow][so-question-count]][stackoverflow]
[![Website][website-status]][website]


[discord-image]: https://img.shields.io/discord/694824801977630762?color=7289da&label=community%20chat&logo=discord&style=plastic
[discord-server]: https://discord.gg/JFFMAXS
[website-status]: https://img.shields.io/website/https/pointcloudlibrary.github.io.svg?down_color=red&down_message=is%20down&up_color=green&up_message=is%20new
[website]: https://pointclouds.org/

[so-question-count]: https://img.shields.io/stackexchange/stackoverflow/t/point-cloud-library.svg?logo=stackoverflow
[stackoverflow]: https://stackoverflow.com/questions/tagged/point-cloud-library

Distribution
---------
[![Packaging status](https://repology.org/badge/tiny-repos/pcl-pointclouds.svg)](https://repology.org/project/pcl-pointclouds/badges)
[![latest packaged version(s)](https://repology.org/badge/latest-versions/pcl-pointclouds.svg)](https://repology.org/project/pcl-pointclouds/versions)

<details>
<summary>Click to see all</summary>
<p>
<a href="https://repology.org/project/pcl-pointclouds/packages">
    <img src="https://repology.org/badge/vertical-allrepos/pcl-pointclouds.svg?columns=3"
         alt="Packaging status">
</a>
</p>
</details>

Description
-----------
The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

PCL is released under the terms of the BSD license, and thus free for commercial and research use. We are financially supported by a consortium of commercial companies, with our own non-profit organization, Open Perception. We would also like to thank individual donors and contributors that have been helping the project.

Compiling
---------
Please refer to the platform specific tutorials:
 - [Linux](https://pcl-tutorials.readthedocs.io/en/latest/compiling_pcl_posix.html)
 - [Mac OS X](https://pcl-tutorials.readthedocs.io/en/latest/compiling_pcl_macosx.html)
 - [Microsoft Windows](https://pcl-tutorials.readthedocs.io/en/latest/compiling_pcl_windows.html)

Documentation
-------------
- [Tutorials](https://pcl-tutorials.readthedocs.io/)
- [PCL trunk documentation](https://pointclouds.org/documentation/)

Contributing
------------
Please read [CONTRIBUTING.md](https://github.com/PointCloudLibrary/pcl/blob/master/CONTRIBUTING.md).

Issues
------
To report issues, please read [CONTRIBUTING.md#bug-reports](https://github.com/PointCloudLibrary/pcl/blob/master/CONTRIBUTING.md#bug-reports).

For general questions on how to use the PCL, please consider one of the following alternatives instead:
* [Stack Overflow](https://stackoverflow.com/questions/tagged/point-cloud-library)
for Q&A as well as support for troubleshooting, installation and debugging. Do
remember to tag your questions with the tag `point-cloud-library`.
* [Discord Server](https://discord.gg/JFFMAXS) for live chat with
other members of the PCL community and casual discussions

Citation
--------
We encourage other researchers to cite PCL if they use PCL or its components for their work or baselines. The bibtex entry for the same is
```
@InProceedings{Rusu_ICRA2011_PCL,
  author    = {Radu Bogdan Rusu and Steve Cousins},
  title     = {{3D is here: Point Cloud Library (PCL)}},
  booktitle = {{IEEE International Conference on Robotics and Automation (ICRA)}},
  month     = {May 9-13},
  year      = {2011},
  address   = {Shanghai, China},
  publisher = {IEEE}
}
```
