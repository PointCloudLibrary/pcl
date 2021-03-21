# License

PCL carries a custom (old) version of METSlib which was added (after relicensing for compatibility with PCL) under BSD 3 clause. The details for the addition are:
* author: Mirko Maischberger <mirko.maischberger@gmail.com>
* commit: e4d6501af048215ce84b4ee436ff0e18dba2d30d
* URL: https://projects.coin-or.org/metslib

This has been modified to work with system installed METSlib as well.
Though METSlib is available primarily under GPLv3, it is dually licensed to be available under EPL-1.0 (a commercial friendly license) as well.

# TL;DR
This implies the following:
* While using PCL with the METSlib shipped with it, everything is licensed under BSD
* While using PCL with system installed METSlib, PCL adheres to the API available under the EPL-1.0 license
* PCL doesn't use the system installed METSlib released under GPLv3 for testing/reference/linking
