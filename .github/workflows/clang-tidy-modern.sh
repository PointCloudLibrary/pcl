#!/bin/bash
run-clang-tidy-10.py -config='' -header-filter='^((?!.*kiss_fft\.h.*).)*$,.*' -fix
