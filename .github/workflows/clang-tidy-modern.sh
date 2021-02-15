#!/bin/bash
# kiss_fft ignored for now
# Will be used in future PR as clang-tidy is applied up the PCL dependency graph
# Tracked in issue #2731
run-clang-tidy-10.py -config='' -header-filter='^((?!.*kiss_fft\.h.*).)*$,.*' -fix
