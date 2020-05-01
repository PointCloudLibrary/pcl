# Point Cloud Library

<p align="center"><img src="pcl.png" height="100"></p>

[![Release][release-image]][releases]
[![License][license-image]][license]

[release-image]: https://img.shields.io/badge/release-1.10.1-green.svg?style=flat
[releases]: https://github.com/PointCloudLibrary/pcl/releases

[license-image]: https://img.shields.io/badge/license-BSD-green.svg?style=flat
[license]: https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt

:bangbang:Website:bangbang:
-------

The website (http://pointclouds.org) is down currently :broken_heart:, and we are working on bringing it back online :hammer_and_wrench:. In the meanwhile, please use [the copy made by the internet archive](https://web.archive.org/web/20191017164724/http://www.pointclouds.org/).

Continuous integration
----------------------
[ci-latest-build]: https://dev.azure.com/PointCloudLibrary/pcl/_build/latest?definitionId=9&branchName=master
[ci-ubuntu-16.04]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Ubuntu&jobName=Ubuntu&configuration=Ubuntu%2016.04%20GCC&label=Ubuntu%2016.04
[ci-ubuntu-19.10]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Ubuntu&jobName=Ubuntu&configuration=Ubuntu%2019.10%20GCC&label=Ubuntu%2019.10
[ci-windows-x86]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Windows&jobName=Windows%20VS2017%20Build&configuration=Windows%20VS2017%20Build%20x86&label=Windows%20VS2017%20x86
[ci-windows-x64]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20Windows&jobName=Windows%20VS2017%20Build&configuration=Windows%20VS2017%20Build%20x64&label=Windows%20VS2017%20x64
[ci-macos-10.14]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20macOS&jobName=macOS&configuration=macOS%20Mojave%2010.14&label=macOS%20Mojave%2010.14
[ci-macos-10.15]: https://dev.azure.com/PointCloudLibrary/pcl/_apis/build/status/9?branchName=master&stageName=Build%20macOS&jobName=macOS&configuration=macOS%20Catalina%2010.15&label=macOS%20Catalina%2010.15

Build Platform           | Status                                                                                                                                                                                                                                                                                                                                       
------------------------ | ------------------------------------------------------------------------------------------------- |
Ubuntu                   | [![Status][ci-ubuntu-16.04]][ci-latest-build] <br> [![Status][ci-ubuntu-19.10]][ci-latest-build]  |
Windows                  | [![Status][ci-windows-x86]][ci-latest-build]  <br> [![Status][ci-windows-x64]][ci-latest-build]   |                                                                                                                                                                    
macOS                    | [![Status][ci-macos-10.14]][ci-latest-build]  <br> [![Status][ci-macos-10.15]][ci-latest-build]   |                                                                                                                                                                    

Community
---------
[![Discord][discord-image]][discord-server]
[![StackOverflow][so-question-count]][stackoverflow]
[![Website][website-status]][website]


[discord-image]: https://img.shields.io/discord/694824801977630762?color=7289da&label=community%20chat&logo=discord&style=plastic
[discord-server]: https://discord.gg/JFFMAXS
[website-status]: https://img.shields.io/website/http/www.pointclouds.org.svg?down_color=red&down_message=is%20down&up_color=yellow&up_message=is%20outdated
[website]: http://www.pointclouds.org

[so-question-count]: https://img.shields.io/stackexchange/stackoverflow/t/point-cloud-library.svg?logo=stackoverflow
[stackoverflow]: https://stackoverflow.com/questions/tagged/point-cloud-library

Distribution
---------
[![Packaging status](https://repology.org/badge/tiny-repos/pcl-pointclouds.svg)](https://repology.org/project/pcl-pointclouds/badges)
<details>
<summary>Click to see all</summary>
<p>
<a href="https://repology.org/project/pcl-pointclouds/versions"><img src="https://repology.org/badge/vertical-allrepos/pcl-pointclouds.svg?header=pcl-pointclouds"></a>
</p>
</details>

Description
-----------
The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

PCL is released under the terms of the BSD license, and thus free for commercial and research use. We are financially supported by a consortium of commercial companies, with our own non-profit organization, Open Perception. We would also like to thank individual donors and contributors that have been helping the project.

Compiling
---------
Please refer to the platform specific tutorials:
 - [Linux](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php)
 - [Mac OS X](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
 - [Microsoft Windows](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_windows.php)

Documentation
-------------
- [Tutorials](http://www.pointclouds.org/documentation/tutorials/)
- [PCL trunk documentation](https://pointcloudlibrary.github.io/documentation/) (updated nightly)

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

<!-- 
  * Mailing list: The [PCL Google Group](https://groups.google.com/forum/#!forum/point-cloud-library)
-->

<!-- There's an option of creating our own compatibility tracker

API/ABI Compatibility Report
------
For details about API/ABI changes over the timeline please check PCL's page at [ABI Laboratory](https://abi-laboratory.pro/tracker/timeline/pcl/).
-->
