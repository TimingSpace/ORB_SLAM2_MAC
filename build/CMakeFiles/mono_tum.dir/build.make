# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.2.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.2.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/build

# Include any dependencies generated for this target.
include CMakeFiles/mono_tum.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mono_tum.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono_tum.dir/flags.make

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o: CMakeFiles/mono_tum.dir/flags.make
CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o: ../Examples/Monocular/mono_tum.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o -c /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/Examples/Monocular/mono_tum.cc

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/Examples/Monocular/mono_tum.cc > CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.i

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/Examples/Monocular/mono_tum.cc -o CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.s

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires:
.PHONY : CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires
	$(MAKE) -f CMakeFiles/mono_tum.dir/build.make CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides.build
.PHONY : CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides

CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.provides.build: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o

# Object files for target mono_tum
mono_tum_OBJECTS = \
"CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o"

# External object files for target mono_tum
mono_tum_EXTERNAL_OBJECTS =

../Examples/Monocular/mono_tum: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o
../Examples/Monocular/mono_tum: CMakeFiles/mono_tum.dir/build.make
../Examples/Monocular/mono_tum: ../lib/libORB_SLAM2.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_videostab.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_ts.a
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_superres.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_stitching.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_contrib.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_nonfree.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_ocl.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_gpu.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_photo.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_objdetect.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_legacy.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_video.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_ml.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_calib3d.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_features2d.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_highgui.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_imgproc.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_flann.2.4.11.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libopencv_core.2.4.11.dylib
../Examples/Monocular/mono_tum: /Users/shikw/Documents/Program/Academic/SLAM/ORBSLAM_DEPENDENCY/Pangolin/build/src/libpangolin.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libGLEW.dylib
../Examples/Monocular/mono_tum: /usr/lib/libpython2.7.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libavcodec.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libavformat.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libavutil.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libswscale.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libpng.dylib
../Examples/Monocular/mono_tum: /usr/lib/libz.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libjpeg.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libtiff.dylib
../Examples/Monocular/mono_tum: /usr/local/lib/libIlmImf.dylib
../Examples/Monocular/mono_tum: ../Thirdparty/DBoW2/lib/libDBoW2.dylib
../Examples/Monocular/mono_tum: ../Thirdparty/g2o/lib/libg2o.dylib
../Examples/Monocular/mono_tum: CMakeFiles/mono_tum.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../Examples/Monocular/mono_tum"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono_tum.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono_tum.dir/build: ../Examples/Monocular/mono_tum
.PHONY : CMakeFiles/mono_tum.dir/build

CMakeFiles/mono_tum.dir/requires: CMakeFiles/mono_tum.dir/Examples/Monocular/mono_tum.cc.o.requires
.PHONY : CMakeFiles/mono_tum.dir/requires

CMakeFiles/mono_tum.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono_tum.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono_tum.dir/clean

CMakeFiles/mono_tum.dir/depend:
	cd /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2 /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2 /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/build /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/build /Users/shikw/Documents/Program/Academic/SLAM/ORB_SLAM2/build/CMakeFiles/mono_tum.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono_tum.dir/depend

