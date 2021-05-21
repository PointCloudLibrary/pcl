#!/usr/bin/env bash

show_help() {
cat << EOF
Usage: ${0##*/} [OPTIONS] INSTALL_DIR SOURCE_DIR BUILD_DIR

This script builds source code projects of PCL tutorials.

Options:

  -h             Dispaly this help and exit.
  -k             Keep going after a configuration/build error.
  -s             Print summary in the end.
  -e NAMES       Exclude tutorials from the build.
                 NAMES is a comma-separated list of tutorial names.

Arguments:

  INSTALL_DIR    Path to the directory where PCLConfig.cmake in installed.
  SOURCE_DIR     Path to the root of PCL repository.
  BUILD_DIR      Path to the directory where the tutorials should be built.
EOF
}

while getopts "hkse:" option; do
  case "${option}" in
    h) show_help
       exit 0
       ;;
    k) KEEP_GOING=1
       ;;
    s) SUMMARY=1
       ;;
    e) EXCLUDE=${OPTARG}
       ;;
    *) show_help
       exit 1
       ;;
  esac
done

shift $((OPTIND-1))

if [[ $# -ne 3 ]]; then
  show_help
  exit 1
fi

INSTALL_DIR=$1
SOURCE_DIR=$2/doc/tutorials/content/sources
BUILD_DIR=$3

if [[ ! -f "$INSTALL_DIR/PCLConfig.cmake" ]]; then
  echo "Invalid install directory"
  exit 2
fi

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "Invalid source directory"
  exit 3
fi

mkdir -p "$BUILD_DIR"

TUTORIALS=()

for DIRECTORY in "$SOURCE_DIR"/*/ ; do
  NAME=$(basename "$DIRECTORY")
  if [[ "$EXCLUDE" == *$NAME* ]]; then
    STATUS="excluded"
  elif [[ -z ${SKIP+x} ]]; then
    TUTORIAL_SOURCE_DIR=$(realpath "$DIRECTORY")
    TUTORIAL_BUILD_DIR="$BUILD_DIR/$NAME"
    mkdir -p "$TUTORIAL_BUILD_DIR" && cd "$TUTORIAL_BUILD_DIR" || exit
    echo "Configuring tutorial: $NAME"
    if ! cmake "$TUTORIAL_SOURCE_DIR" -DPCL_DIR="$INSTALL_DIR" -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Werror"; then
      STATUS="cmake error"
    else
      echo "Building tutorial: $NAME"
      if ! cmake --build . -- -j2; then
        STATUS="build error"
      fi
    fi
    if [[ -n $STATUS ]]; then
      FAILED=1
      if [[ $KEEP_GOING -ne 1 ]]; then
        SKIP=1
      fi
    fi
    cd - || exit
  else
    STATUS="skipped"
  fi
  TUTORIALS+=("$NAME")
  STATUSES+=("$STATUS")
  unset STATUS
done

if [[ $SUMMARY -eq 1 ]]; then
  echo ""
  echo "Tutorial build summary"
  echo "----------------------"
  echo ""
  SUCCEEDED=0
  EXCLUDED=0
  for i in "${!TUTORIALS[@]}"; do
    if [[ "${STATUSES[$i]}" == "" ]]; then
      MARK="🗸"
      SUCCEEDED=$((SUCCEEDED+1))
    elif [[ "${STATUSES[$i]}" == *error* ]]; then
      MARK="𐄂"
    else
      MARK="-"
      EXCLUDED=$((EXCLUDED+1))
    fi
    printf "%-46s  %s  %s\\n" "${TUTORIALS[$i]}" "$MARK" "${STATUSES[$i]}"
  done
  echo ""
  echo "Succeeded building $SUCCEEDED out of ${#TUTORIALS[@]} tutorials"
  echo "Excluded or skipped $EXCLUDED tutorials"
fi

exit $FAILED
