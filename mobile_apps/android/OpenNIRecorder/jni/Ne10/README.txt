Copyright 2011-12 ARM Limited

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


NE10
====

The objective of this library is to provide a set of common, useful functions
which have been heavily optimised for ARM, provide consistent well tested
behaviour and that can be easily incorporated into applications.

The primary API provided is C. The primary OS targeted is Android, although
the library is tested with Linaro Linux.

The design is intended to be available as a simple 'drop and go' pre-built
library and as a set of modular functions that can be incorporated in a more
modular pick and mix form where binary size might be an issue.

Each function is implemented in C, ARM Assembly and NEON code as a basis for
comparison. Assembly versions, while efficient, are not intended as 
best-practice examples.

Future releases are intended to expand on the functions provided and possibly
the supported languages (C++ being near the top of that list).

Licensed under the Apache License, Version 2.0

(See LICENSE for details)

Usage
=====

See USAGE.txt file


