# set EXTRA_CMAKE_FLAGS in the including Makefile in order to add tweaks
CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# The all target does the heavy lifting, creating the build directory and
# invoking CMake
all:
	@mkdir -p build
	-mkdir -p bin
	@rm -rf msg/cpp srv/cpp  # make sure there are no msg/cpp or srv/cpp directories
	cd build && $(PRE_CMAKE_FLAGS) cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE) $(ROS_PARALLEL_JOBS)
else
	cd build && make $(ROS_PARALLEL_JOBS)
endif

PACKAGE_NAME=$(shell basename $(PWD))

# The clean target blows everything away
# It also removes auto-generated message/service code directories, 
# to handle the case where the original .msg/.srv file has been removed,
# and thus CMake no longer knows about it.
clean:
	-cd build && make clean
	rm -rf build

# All other targets are just passed through
test: all
	if cd build && make -k $@; then make test-results; else make test-results && exit 1; fi
test-nobuild:
	@mkdir -p build
	cd build && $(PRE_CMAKE_FLAGS) ccmake $(CMAKE_FLAGS) -Drosbuild_test_nobuild=1 ..
	if cd build && make clean-test-results && make -k test; then make test-results; else make test-results && exit 1; fi
tests: all
	cd build && make $@
test-future: all
	cd build && make -k $@
gcoverage: all
	cd build && make $@

eclipse-project: 
	mv Makefile Makefile.ros
	cmake -G"Eclipse CDT4 - Unix Makefiles" -Wno-dev .
	rm Makefile
	rm CMakeCache.txt
	rm -rf CMakeFiles
	mv Makefile.ros Makefile
	mv .project .project-cmake
	awk -f $(shell rospack find mk)/eclipse.awk .project-cmake > .project
	rm .project-cmake
	python $(shell rospack find mk)/make_pydev_project.py


include $(shell rospack find mk)/buildtest.mk
