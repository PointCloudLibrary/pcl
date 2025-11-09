#! /usr/bin/env sh

# sample command line usage: $0 clang-format(version >= 6.0) $PCL_SOURCE_DIR
# $ sh ./.dev/format.sh `which clang-format` ./
# $ sh format.sh `which clang-format` ../
# $ sh ~/pcl/format.sh `which clang-format` ~/pcl
# $ sh /pcl/format.sh `which clang-format` /pcl

format() {
    local PCL_DIR="${2}"
    local formatter="${1}"

    if [ ! -f "${formatter}" ]; then
        echo "Could not find a clang-format. Please specify one as the first argument"
        exit 166
    fi

    if [ ! -f "${PCL_DIR}/.dev/format.sh" ]; then
        echo "Please ensure that PCL_SOURCE_DIR is passed as the second argument"
        exit 166
    fi

    local whitelist_file="${PCL_DIR}/.dev/whitelist.txt"
    if [ ! -f "${whitelist_file}" ]; then
        echo "Could not find whitelist file at ${whitelist_file}"
        exit 167
    fi

    while IFS= read -r dir || [ -n "$dir" ]; do
        [ -z "$dir" ] && continue
        path=${PCL_DIR}/${dir}
        find ${path} -type f \( -iname "*.[ch]" -o -iname "*.[ch]pp" -o -iname "*.[ch]xx" -o -iname "*.cu" \) | xargs -n1 ${formatter} -i -style=file
    done < "${whitelist_file}"
}

format "$@"
