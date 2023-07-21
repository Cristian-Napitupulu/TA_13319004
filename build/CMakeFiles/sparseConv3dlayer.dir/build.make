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
include CMakeFiles/sparseConv3dlayer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/sparseConv3dlayer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/sparseConv3dlayer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sparseConv3dlayer.dir/flags.make

CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o: CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o.depend
CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o: CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o.Release.cmake
CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o: /home/icodes/Documents/TA_13319004/se-ssd_gantry/sparseConv3dlayer.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/icodes/Documents/TA_13319004/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building NVCC (Device) object CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o"
	cd /home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir && /usr/bin/cmake -E make_directory /home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir//.
	cd /home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir && /usr/bin/cmake -D verbose:BOOL=$(VERBOSE) -D build_configuration:STRING=Release -D generated_file:STRING=/home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir//./sparseConv3dlayer_generated_sparseConv3dlayer.cu.o -D generated_cubin_file:STRING=/home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir//./sparseConv3dlayer_generated_sparseConv3dlayer.cu.o.cubin.txt -P /home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir//sparseConv3dlayer_generated_sparseConv3dlayer.cu.o.Release.cmake

# Object files for target sparseConv3dlayer
sparseConv3dlayer_OBJECTS =

# External object files for target sparseConv3dlayer
sparseConv3dlayer_EXTERNAL_OBJECTS = \
"/home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o"

libsparseConv3dlayer.so: CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o
libsparseConv3dlayer.so: CMakeFiles/sparseConv3dlayer.dir/build.make
libsparseConv3dlayer.so: /usr/local/cuda-11.8/lib64/libcudart.so
libsparseConv3dlayer.so: CMakeFiles/sparseConv3dlayer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/icodes/Documents/TA_13319004/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsparseConv3dlayer.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sparseConv3dlayer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sparseConv3dlayer.dir/build: libsparseConv3dlayer.so
.PHONY : CMakeFiles/sparseConv3dlayer.dir/build

CMakeFiles/sparseConv3dlayer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sparseConv3dlayer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sparseConv3dlayer.dir/clean

CMakeFiles/sparseConv3dlayer.dir/depend: CMakeFiles/sparseConv3dlayer.dir/sparseConv3dlayer_generated_sparseConv3dlayer.cu.o
	cd /home/icodes/Documents/TA_13319004/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/icodes/Documents/TA_13319004/se-ssd_gantry /home/icodes/Documents/TA_13319004/se-ssd_gantry /home/icodes/Documents/TA_13319004/build /home/icodes/Documents/TA_13319004/build /home/icodes/Documents/TA_13319004/build/CMakeFiles/sparseConv3dlayer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sparseConv3dlayer.dir/depend
