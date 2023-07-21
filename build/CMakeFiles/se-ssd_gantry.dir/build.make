# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/icodes/Documents/TA_13319004/se-ssd_gantry

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/icodes/Documents/TA_13319004/build

# Include any dependencies generated for this target.
include CMakeFiles/se-ssd_gantry.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/se-ssd_gantry.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/se-ssd_gantry.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/se-ssd_gantry.dir/flags.make

CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o: CMakeFiles/se-ssd_gantry.dir/flags.make
CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o: /home/icodes/Documents/TA_13319004/se-ssd_gantry/se-ssd-ai-trt.cpp
CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o: CMakeFiles/se-ssd_gantry.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/icodes/Documents/TA_13319004/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o -MF CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o.d -o CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o -c /home/icodes/Documents/TA_13319004/se-ssd_gantry/se-ssd-ai-trt.cpp

CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/icodes/Documents/TA_13319004/se-ssd_gantry/se-ssd-ai-trt.cpp > CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.i

CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/icodes/Documents/TA_13319004/se-ssd_gantry/se-ssd-ai-trt.cpp -o CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.s

# Object files for target se-ssd_gantry
se__ssd_gantry_OBJECTS = \
"CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o"

# External object files for target se-ssd_gantry
se__ssd_gantry_EXTERNAL_OBJECTS =

se-ssd_gantry: CMakeFiles/se-ssd_gantry.dir/se-ssd-ai-trt.cpp.o
se-ssd_gantry: CMakeFiles/se-ssd_gantry.dir/build.make
se-ssd_gantry: libvoxelGeneratorlayer.so
se-ssd_gantry: libsubmConv3dlayer.so
se-ssd_gantry: libsparseConv3dlayer.so
se-ssd_gantry: libsparse2Denselayer.so
se-ssd_gantry: libzeroPad2dlayer.so
se-ssd_gantry: libgenerateAnchorDecodelayer.so
se-ssd_gantry: libfilterBoxByScorelayer.so
se-ssd_gantry: /usr/local/cuda-11.8/lib64/libcudart.so
se-ssd_gantry: CMakeFiles/se-ssd_gantry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/icodes/Documents/TA_13319004/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable se-ssd_gantry"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/se-ssd_gantry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/se-ssd_gantry.dir/build: se-ssd_gantry
.PHONY : CMakeFiles/se-ssd_gantry.dir/build

CMakeFiles/se-ssd_gantry.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/se-ssd_gantry.dir/cmake_clean.cmake
.PHONY : CMakeFiles/se-ssd_gantry.dir/clean

CMakeFiles/se-ssd_gantry.dir/depend:
	cd /home/icodes/Documents/TA_13319004/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/icodes/Documents/TA_13319004/se-ssd_gantry /home/icodes/Documents/TA_13319004/se-ssd_gantry /home/icodes/Documents/TA_13319004/build /home/icodes/Documents/TA_13319004/build /home/icodes/Documents/TA_13319004/build/CMakeFiles/se-ssd_gantry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/se-ssd_gantry.dir/depend
