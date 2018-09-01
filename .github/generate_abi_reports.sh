#! /bin/bash

if ! hash cmake 2>/dev/null \
  || ! hash abi-dumper 2>/dev/null \
  || ! hash abi-compliance-checker 2>/dev/null; then
  echo "This script requires cmake, abi-dumper and abi-compliance-checker"
  echo "On Ubuntu: apt-get install cmake abi-dumper abi-compliance-checker"
  exit 1
fi

usage ()
{
  echo "Usage: $0 [-o <output_folder> ] [-j <number of workers>] <old_rev> <new_rev>" 1>&2;
  exit 1;
}

# Set defaults
ROOT_FOLDER="$(pwd)/abi-reports"
N_WORKERS=4

# Parse arguments
while getopts ":o:j:" o; do
    case "${o}" in
        o)
            ROOT_FOLDER=${OPTARG}
            ;;
        j)
            N_WORKERS=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

# Shift positional arguments
shift $((OPTIND-1))

# Check if positional arguments were parsed
OLD_VERSION=$1
NEW_VERSION=$2

if [ -z "${OLD_VERSION}" ] || [ -z "${NEW_VERSION}" ]; then
    usage
    exit 1
fi

# create the top folders
mkdir -p "$ROOT_FOLDER"
cd "$ROOT_FOLDER"
PCL_FOLDER="$ROOT_FOLDER/pcl"

if [ ! -d "$PCL_FOLDER" ]; then
  # Clone if needed
  git clone https://github.com/PointCloudLibrary/pcl.git
fi

function build_and_dump
{
  # Store current folder
  local PWD=$(pwd)

  # If it is a version prepend pcl-
  local TAG=$1
  if [ $(echo "$1" | grep "^[0-9]\+\.[0-9]\+\.[0-9]\+$") ]; then
    TAG="pcl-$TAG"
  fi

  # Checkout correct version
  cd "$PCL_FOLDER"
  git checkout $TAG

  # Escape version
  local VERSION_ESCAPED=$(echo "$1" | sed -e 's/\./_/g')

  # Create the install folders
  local INSTALL_FOLDER="$ROOT_FOLDER/install/$VERSION_ESCAPED"
  mkdir -p "$INSTALL_FOLDER"

  # create the build folders
  local BUILD_FOLDER="$ROOT_FOLDER/build/$VERSION_ESCAPED"
  mkdir -p  "$BUILD_FOLDER"
  cd "$BUILD_FOLDER"

  # Build
  cmake -DCMAKE_BUILD_TYPE="Debug" \
        -DCMAKE_CXX_FLAGS="-Og" \
        -DCMAKE_C_FLAGS="-Og" \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_FOLDER" \
        -DPCL_ONLY_CORE_POINT_TYPES=ON \
        -DBUILD_tools=OFF \
        -DBUILD_global_tests=OFF \
        "$PCL_FOLDER"
        # -DBUILD_2d=OFF \
        # -DBUILD_features=OFF \
        # -DBUILD_filters=OFF \
        # -DBUILD_geometry=OFF \
        # -DBUILD_io=OFF \
        # -DBUILD_kdtree=OFF \
        # -DBUILD_keypoints=OFF \
        # -DBUILD_ml=OFF \
        # -DBUILD_octree=OFF \
        # -DBUILD_outofcore=OFF \
        # -DBUILD_people=OFF \
        # -DBUILD_recognition=OFF \
        # -DBUILD_registration=OFF \
        # -DBUILD_sample_consensus=OFF \
        # -DBUILD_search=OFF \
        # -DBUILD_segmentation=OFF \
        # -DBUILD_stereo=OFF \
        # -DBUILD_surface=OFF \
        # -DBUILD_tracking=OFF \
        # -DBUILD_visualization=OFF \
  make -j${N_WORKERS} install

  # create ABI dump folder
  local ABI_DUMP_FOLDER="$ROOT_FOLDER/abi_dump/$VERSION_ESCAPED"
  mkdir -p  "$ABI_DUMP_FOLDER"

  # Sweep through available modules
  cd "$INSTALL_FOLDER/lib"
  ls *.so | sed -e "s/^libpcl_//" -e "s/.so//" \
    | awk '{print "libpcl_"$1".so -o '"$ABI_DUMP_FOLDER"'/"$1".dump -lver '"$1"'"}' \
    | xargs -n5 -P${N_WORKERS} abi-dumper

  # Restore folder
  cd $PWD
}

build_and_dump "$OLD_VERSION"
build_and_dump "$NEW_VERSION"


# Move to root folder and generate reports
OLD_VERSION_ESCAPED=$(echo "$OLD_VERSION" | sed -e 's/\./_/g')
NEW_VERSION_ESCAPED=$(echo "$NEW_VERSION" | sed -e 's/\./_/g')
OLD_INSTALL_FOLDER="$ROOT_FOLDER/install/$OLD_VERSION_ESCAPED"
OLD_ABI_DUMP_FOLDER="$ROOT_FOLDER/abi_dump/$OLD_VERSION_ESCAPED"
NEW_ABI_DUMP_FOLDER="$ROOT_FOLDER/abi_dump/$NEW_VERSION_ESCAPED"

cd "$ROOT_FOLDER"
find $OLD_INSTALL_FOLDER/lib -name '*.so' -printf '%P\n' | sed -e "s/^libpcl_//" -e "s/.so//" \
    | awk '{print "-lib "$1" -old '"$OLD_ABI_DUMP_FOLDER"'/"$1".dump -new '"$NEW_ABI_DUMP_FOLDER"'/"$1".dump"}' \
    | xargs -n6 -P${N_WORKERS} abi-compliance-checker
