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
CMAKE_SOURCE_DIR = /home/stitch0001/catkin_WS/src/web_video_server

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stitch0001/catkin_WS/src/web_video_server

# Include any dependencies generated for this target.
include CMakeFiles/web_video_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/web_video_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/web_video_server.dir/flags.make

CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o: src/web_video_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/web_video_server.cpp

CMakeFiles/web_video_server.dir/src/web_video_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/web_video_server.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/web_video_server.cpp > CMakeFiles/web_video_server.dir/src/web_video_server.cpp.i

CMakeFiles/web_video_server.dir/src/web_video_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/web_video_server.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/web_video_server.cpp -o CMakeFiles/web_video_server.dir/src/web_video_server.cpp.s

CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.requires

CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.provides: CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.provides

CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o


CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o: src/image_streamer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/image_streamer.cpp

CMakeFiles/web_video_server.dir/src/image_streamer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/image_streamer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/image_streamer.cpp > CMakeFiles/web_video_server.dir/src/image_streamer.cpp.i

CMakeFiles/web_video_server.dir/src/image_streamer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/image_streamer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/image_streamer.cpp -o CMakeFiles/web_video_server.dir/src/image_streamer.cpp.s

CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.requires

CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.provides: CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.provides

CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o


CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o: src/libav_streamer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/libav_streamer.cpp

CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/libav_streamer.cpp > CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.i

CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/libav_streamer.cpp -o CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.s

CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.requires

CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.provides: CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.provides

CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o


CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o: src/vp8_streamer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/vp8_streamer.cpp

CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/vp8_streamer.cpp > CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.i

CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/vp8_streamer.cpp -o CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.s

CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.requires

CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.provides: CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.provides

CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o


CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o: src/h264_streamer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/h264_streamer.cpp

CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/h264_streamer.cpp > CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.i

CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/h264_streamer.cpp -o CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.s

CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.requires

CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.provides: CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.provides

CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o


CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o: src/vp9_streamer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/vp9_streamer.cpp

CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/vp9_streamer.cpp > CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.i

CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/vp9_streamer.cpp -o CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.s

CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.requires

CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.provides: CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.provides

CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o


CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o: src/multipart_stream.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/multipart_stream.cpp

CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/multipart_stream.cpp > CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.i

CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/multipart_stream.cpp -o CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.s

CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.requires

CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.provides: CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.provides

CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o


CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o: src/ros_compressed_streamer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/ros_compressed_streamer.cpp

CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/ros_compressed_streamer.cpp > CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.i

CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/ros_compressed_streamer.cpp -o CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.s

CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.requires

CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.provides: CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.provides

CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o


CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o: src/jpeg_streamers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/jpeg_streamers.cpp

CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/jpeg_streamers.cpp > CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.i

CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/jpeg_streamers.cpp -o CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.s

CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.requires

CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.provides: CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.provides

CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o


CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o: CMakeFiles/web_video_server.dir/flags.make
CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o: src/png_streamers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o -c /home/stitch0001/catkin_WS/src/web_video_server/src/png_streamers.cpp

CMakeFiles/web_video_server.dir/src/png_streamers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/web_video_server.dir/src/png_streamers.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stitch0001/catkin_WS/src/web_video_server/src/png_streamers.cpp > CMakeFiles/web_video_server.dir/src/png_streamers.cpp.i

CMakeFiles/web_video_server.dir/src/png_streamers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/web_video_server.dir/src/png_streamers.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stitch0001/catkin_WS/src/web_video_server/src/png_streamers.cpp -o CMakeFiles/web_video_server.dir/src/png_streamers.cpp.s

CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.requires:

.PHONY : CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.requires

CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.provides: CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.requires
	$(MAKE) -f CMakeFiles/web_video_server.dir/build.make CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.provides.build
.PHONY : CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.provides

CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.provides.build: CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o


# Object files for target web_video_server
web_video_server_OBJECTS = \
"CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o" \
"CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o" \
"CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o" \
"CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o" \
"CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o" \
"CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o" \
"CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o" \
"CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o" \
"CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o" \
"CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o"

# External object files for target web_video_server
web_video_server_EXTERNAL_OBJECTS =

devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/build.make
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libimage_transport.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/web_video_server/web_video_server: /usr/lib/libPocoFoundation.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libdl.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libroscpp.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/librosconsole.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libroslib.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/librospack.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libpython2.7.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libasync_web_server_cpp.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_regex.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/librostime.so
devel/lib/web_video_server/web_video_server: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_system.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_thread.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libpthread.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_thread.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_system.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libpthread.so
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_gapi.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_stitching.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_aruco.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_bgsegm.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_bioinspired.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_ccalib.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudabgsegm.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudafeatures2d.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudaobjdetect.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudastereo.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cvv.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_dnn_objdetect.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_dpm.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_face.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_freetype.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_fuzzy.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_hdf.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_hfs.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_img_hash.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_line_descriptor.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_quality.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_reg.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_rgbd.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_saliency.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_stereo.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_structured_light.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_superres.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_surface_matching.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_tracking.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_videostab.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_viz.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_xfeatures2d.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_xobjdetect.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_xphoto.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_shape.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_datasets.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_plot.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_text.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_dnn.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_highgui.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_ml.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_phase_unwrapping.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudacodec.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_videoio.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudaoptflow.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudalegacy.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudawarping.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_optflow.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_video.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_ximgproc.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_imgcodecs.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_objdetect.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_calib3d.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_features2d.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_flann.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_photo.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudaimgproc.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudafilters.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_imgproc.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudaarithm.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_core.so.4.1.1
devel/lib/web_video_server/web_video_server: /usr/local/lib/libopencv_cudev.so.4.1.1
devel/lib/web_video_server/web_video_server: CMakeFiles/web_video_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable devel/lib/web_video_server/web_video_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/web_video_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/web_video_server.dir/build: devel/lib/web_video_server/web_video_server

.PHONY : CMakeFiles/web_video_server.dir/build

CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/web_video_server.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/image_streamer.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/libav_streamer.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/vp8_streamer.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/h264_streamer.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/vp9_streamer.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/multipart_stream.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/ros_compressed_streamer.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/jpeg_streamers.cpp.o.requires
CMakeFiles/web_video_server.dir/requires: CMakeFiles/web_video_server.dir/src/png_streamers.cpp.o.requires

.PHONY : CMakeFiles/web_video_server.dir/requires

CMakeFiles/web_video_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/web_video_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/web_video_server.dir/clean

CMakeFiles/web_video_server.dir/depend:
	cd /home/stitch0001/catkin_WS/src/web_video_server && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stitch0001/catkin_WS/src/web_video_server /home/stitch0001/catkin_WS/src/web_video_server /home/stitch0001/catkin_WS/src/web_video_server /home/stitch0001/catkin_WS/src/web_video_server /home/stitch0001/catkin_WS/src/web_video_server/CMakeFiles/web_video_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/web_video_server.dir/depend

