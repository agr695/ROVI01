# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/Downloads/ROVI01/vision/miniproject1/image4_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build

# Include any dependencies generated for this target.
include CMakeFiles/image4_2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/image4_2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/image4_2.dir/flags.make

CMakeFiles/image4_2.dir/src/image4_2.cpp.o: CMakeFiles/image4_2.dir/flags.make
CMakeFiles/image4_2.dir/src/image4_2.cpp.o: ../src/image4_2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/image4_2.dir/src/image4_2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/image4_2.dir/src/image4_2.cpp.o -c /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/src/image4_2.cpp

CMakeFiles/image4_2.dir/src/image4_2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image4_2.dir/src/image4_2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/src/image4_2.cpp > CMakeFiles/image4_2.dir/src/image4_2.cpp.i

CMakeFiles/image4_2.dir/src/image4_2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image4_2.dir/src/image4_2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/src/image4_2.cpp -o CMakeFiles/image4_2.dir/src/image4_2.cpp.s

CMakeFiles/image4_2.dir/src/image4_2.cpp.o.requires:

.PHONY : CMakeFiles/image4_2.dir/src/image4_2.cpp.o.requires

CMakeFiles/image4_2.dir/src/image4_2.cpp.o.provides: CMakeFiles/image4_2.dir/src/image4_2.cpp.o.requires
	$(MAKE) -f CMakeFiles/image4_2.dir/build.make CMakeFiles/image4_2.dir/src/image4_2.cpp.o.provides.build
.PHONY : CMakeFiles/image4_2.dir/src/image4_2.cpp.o.provides

CMakeFiles/image4_2.dir/src/image4_2.cpp.o.provides.build: CMakeFiles/image4_2.dir/src/image4_2.cpp.o


# Object files for target image4_2
image4_2_OBJECTS = \
"CMakeFiles/image4_2.dir/src/image4_2.cpp.o"

# External object files for target image4_2
image4_2_EXTERNAL_OBJECTS =

image4_2: CMakeFiles/image4_2.dir/src/image4_2.cpp.o
image4_2: CMakeFiles/image4_2.dir/build.make
image4_2: /usr/local/lib/libopencv_stitching.so.3.4.2
image4_2: /usr/local/lib/libopencv_superres.so.3.4.2
image4_2: /usr/local/lib/libopencv_videostab.so.3.4.2
image4_2: /usr/local/lib/libopencv_aruco.so.3.4.2
image4_2: /usr/local/lib/libopencv_bgsegm.so.3.4.2
image4_2: /usr/local/lib/libopencv_bioinspired.so.3.4.2
image4_2: /usr/local/lib/libopencv_ccalib.so.3.4.2
image4_2: /usr/local/lib/libopencv_cvv.so.3.4.2
image4_2: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.2
image4_2: /usr/local/lib/libopencv_dpm.so.3.4.2
image4_2: /usr/local/lib/libopencv_face.so.3.4.2
image4_2: /usr/local/lib/libopencv_freetype.so.3.4.2
image4_2: /usr/local/lib/libopencv_fuzzy.so.3.4.2
image4_2: /usr/local/lib/libopencv_hdf.so.3.4.2
image4_2: /usr/local/lib/libopencv_hfs.so.3.4.2
image4_2: /usr/local/lib/libopencv_img_hash.so.3.4.2
image4_2: /usr/local/lib/libopencv_line_descriptor.so.3.4.2
image4_2: /usr/local/lib/libopencv_optflow.so.3.4.2
image4_2: /usr/local/lib/libopencv_reg.so.3.4.2
image4_2: /usr/local/lib/libopencv_rgbd.so.3.4.2
image4_2: /usr/local/lib/libopencv_saliency.so.3.4.2
image4_2: /usr/local/lib/libopencv_sfm.so.3.4.2
image4_2: /usr/local/lib/libopencv_stereo.so.3.4.2
image4_2: /usr/local/lib/libopencv_structured_light.so.3.4.2
image4_2: /usr/local/lib/libopencv_surface_matching.so.3.4.2
image4_2: /usr/local/lib/libopencv_tracking.so.3.4.2
image4_2: /usr/local/lib/libopencv_xfeatures2d.so.3.4.2
image4_2: /usr/local/lib/libopencv_ximgproc.so.3.4.2
image4_2: /usr/local/lib/libopencv_xobjdetect.so.3.4.2
image4_2: /usr/local/lib/libopencv_xphoto.so.3.4.2
image4_2: /usr/local/lib/libopencv_photo.so.3.4.2
image4_2: /usr/local/lib/libopencv_shape.so.3.4.2
image4_2: /usr/local/lib/libopencv_calib3d.so.3.4.2
image4_2: /usr/local/lib/libopencv_viz.so.3.4.2
image4_2: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.2
image4_2: /usr/local/lib/libopencv_video.so.3.4.2
image4_2: /usr/local/lib/libopencv_datasets.so.3.4.2
image4_2: /usr/local/lib/libopencv_plot.so.3.4.2
image4_2: /usr/local/lib/libopencv_text.so.3.4.2
image4_2: /usr/local/lib/libopencv_dnn.so.3.4.2
image4_2: /usr/local/lib/libopencv_features2d.so.3.4.2
image4_2: /usr/local/lib/libopencv_flann.so.3.4.2
image4_2: /usr/local/lib/libopencv_highgui.so.3.4.2
image4_2: /usr/local/lib/libopencv_ml.so.3.4.2
image4_2: /usr/local/lib/libopencv_videoio.so.3.4.2
image4_2: /usr/local/lib/libopencv_imgcodecs.so.3.4.2
image4_2: /usr/local/lib/libopencv_objdetect.so.3.4.2
image4_2: /usr/local/lib/libopencv_imgproc.so.3.4.2
image4_2: /usr/local/lib/libopencv_core.so.3.4.2
image4_2: CMakeFiles/image4_2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable image4_2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image4_2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/image4_2.dir/build: image4_2

.PHONY : CMakeFiles/image4_2.dir/build

CMakeFiles/image4_2.dir/requires: CMakeFiles/image4_2.dir/src/image4_2.cpp.o.requires

.PHONY : CMakeFiles/image4_2.dir/requires

CMakeFiles/image4_2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/image4_2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/image4_2.dir/clean

CMakeFiles/image4_2.dir/depend:
	cd /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/Downloads/ROVI01/vision/miniproject1/image4_2 /home/student/Downloads/ROVI01/vision/miniproject1/image4_2 /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build /home/student/Downloads/ROVI01/vision/miniproject1/image4_2/build/CMakeFiles/image4_2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/image4_2.dir/depend

