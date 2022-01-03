#! /usr/bin/env sh

# sample command line usage: $0 clang-format(version >= 6.0) $PCL_SOURCE_DIR
# $ sh ./.dev/format.sh `which clang-format` ./
# $ sh format.sh `which clang-format` ../
# $ sh ~/pcl/format.sh `which clang-format` ~/pcl
# $ sh /pcl/format.sh `which clang-format` /pcl

format() {
    # don't use a directory with whitespace
    local whitelist="apps/3d_rec_framework apps/in_hand_scanner apps/include apps/modeler apps/src benchmarks 2d geometry ml octree simulation stereo tracking registration gpu/containers gpu/segmentation"

    local PCL_DIR="${2}"
    local formatter="${1}"

    if [ ! -f "${formatter}" ]; then
        echo "Could not find a clang-format. Please specify one as the first argument"
        exit 166
    fi

    # check for self
    if [ ! -f "${PCL_DIR}/.dev/format.sh" ]; then
        echo "Please ensure that PCL_SOURCE_DIR is passed as the second argument"
        exit 166
    fi

    for dir in ${whitelist}; do
        path=${PCL_DIR}/${dir}
        find ${path} -type f -iname *.[ch] -o -iname *.[ch]pp -o -iname *.[ch]xx \
            -iname *.cu | xargs -n1 ${formatter} -i -style=file
    done
}

format $@
