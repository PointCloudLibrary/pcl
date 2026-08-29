#! /usr/bin/env sh

# TODO: detect run-clang-tidy
# similar to format
run-clang-tidy-8 -header-filter='.*' -checks='-*,misc-misplaced-const' -export-fixes=clang-tidy_result.yaml
python ./yml2xml.py -i clang-tidy_result.yaml -o clang-tidy_result.xml

# TODO: feed to CI result
