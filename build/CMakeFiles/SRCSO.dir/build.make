# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lmx/new_detection

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lmx/new_detection/build

# Include any dependencies generated for this target.
include CMakeFiles/SRCSO.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SRCSO.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SRCSO.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SRCSO.dir/flags.make

CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o: CMakeFiles/SRCSO.dir/flags.make
CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o: ../src/ArmorDetector.cpp
CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o: CMakeFiles/SRCSO.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmx/new_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o -MF CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o.d -o CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o -c /home/lmx/new_detection/src/ArmorDetector.cpp

CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmx/new_detection/src/ArmorDetector.cpp > CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.i

CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmx/new_detection/src/ArmorDetector.cpp -o CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.s

CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o: CMakeFiles/SRCSO.dir/flags.make
CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o: ../src/DNN_detect.cpp
CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o: CMakeFiles/SRCSO.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmx/new_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o -MF CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o.d -o CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o -c /home/lmx/new_detection/src/DNN_detect.cpp

CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmx/new_detection/src/DNN_detect.cpp > CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.i

CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmx/new_detection/src/DNN_detect.cpp -o CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.s

CMakeFiles/SRCSO.dir/src/camera.cpp.o: CMakeFiles/SRCSO.dir/flags.make
CMakeFiles/SRCSO.dir/src/camera.cpp.o: ../src/camera.cpp
CMakeFiles/SRCSO.dir/src/camera.cpp.o: CMakeFiles/SRCSO.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmx/new_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/SRCSO.dir/src/camera.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SRCSO.dir/src/camera.cpp.o -MF CMakeFiles/SRCSO.dir/src/camera.cpp.o.d -o CMakeFiles/SRCSO.dir/src/camera.cpp.o -c /home/lmx/new_detection/src/camera.cpp

CMakeFiles/SRCSO.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SRCSO.dir/src/camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmx/new_detection/src/camera.cpp > CMakeFiles/SRCSO.dir/src/camera.cpp.i

CMakeFiles/SRCSO.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SRCSO.dir/src/camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmx/new_detection/src/camera.cpp -o CMakeFiles/SRCSO.dir/src/camera.cpp.s

# Object files for target SRCSO
SRCSO_OBJECTS = \
"CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o" \
"CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o" \
"CMakeFiles/SRCSO.dir/src/camera.cpp.o"

# External object files for target SRCSO
SRCSO_EXTERNAL_OBJECTS =

libSRCSO.a: CMakeFiles/SRCSO.dir/src/ArmorDetector.cpp.o
libSRCSO.a: CMakeFiles/SRCSO.dir/src/DNN_detect.cpp.o
libSRCSO.a: CMakeFiles/SRCSO.dir/src/camera.cpp.o
libSRCSO.a: CMakeFiles/SRCSO.dir/build.make
libSRCSO.a: CMakeFiles/SRCSO.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lmx/new_detection/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libSRCSO.a"
	$(CMAKE_COMMAND) -P CMakeFiles/SRCSO.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SRCSO.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SRCSO.dir/build: libSRCSO.a
.PHONY : CMakeFiles/SRCSO.dir/build

CMakeFiles/SRCSO.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SRCSO.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SRCSO.dir/clean

CMakeFiles/SRCSO.dir/depend:
	cd /home/lmx/new_detection/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lmx/new_detection /home/lmx/new_detection /home/lmx/new_detection/build /home/lmx/new_detection/build /home/lmx/new_detection/build/CMakeFiles/SRCSO.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SRCSO.dir/depend

