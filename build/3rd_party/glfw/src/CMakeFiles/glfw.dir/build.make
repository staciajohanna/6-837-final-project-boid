# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.15.3/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.15.3/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build"

# Include any dependencies generated for this target.
include 3rd_party/glfw/src/CMakeFiles/glfw.dir/depend.make

# Include the progress variables for this target.
include 3rd_party/glfw/src/CMakeFiles/glfw.dir/progress.make

# Include the compile flags for this target's objects.
include 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make

3rd_party/glfw/src/CMakeFiles/glfw.dir/context.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/context.c.o: ../3rd_party/glfw/src/context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/context.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/context.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/context.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/context.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/context.c" > CMakeFiles/glfw.dir/context.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/context.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/context.c" -o CMakeFiles/glfw.dir/context.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/init.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/init.c.o: ../3rd_party/glfw/src/init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/init.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/init.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/init.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/init.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/init.c" > CMakeFiles/glfw.dir/init.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/init.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/init.c" -o CMakeFiles/glfw.dir/init.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/input.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/input.c.o: ../3rd_party/glfw/src/input.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/input.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/input.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/input.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/input.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/input.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/input.c" > CMakeFiles/glfw.dir/input.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/input.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/input.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/input.c" -o CMakeFiles/glfw.dir/input.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/monitor.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/monitor.c.o: ../3rd_party/glfw/src/monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/monitor.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/monitor.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/monitor.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/monitor.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/monitor.c" > CMakeFiles/glfw.dir/monitor.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/monitor.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/monitor.c" -o CMakeFiles/glfw.dir/monitor.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/vulkan.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/vulkan.c.o: ../3rd_party/glfw/src/vulkan.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/vulkan.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/vulkan.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/vulkan.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/vulkan.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/vulkan.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/vulkan.c" > CMakeFiles/glfw.dir/vulkan.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/vulkan.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/vulkan.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/vulkan.c" -o CMakeFiles/glfw.dir/vulkan.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/window.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/window.c.o: ../3rd_party/glfw/src/window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/window.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/window.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/window.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/window.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/window.c" > CMakeFiles/glfw.dir/window.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/window.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/window.c" -o CMakeFiles/glfw.dir/window.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_init.m.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_init.m.o: ../3rd_party/glfw/src/cocoa_init.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_7) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_init.m.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/cocoa_init.m.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_init.m"

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_init.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/cocoa_init.m.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_init.m" > CMakeFiles/glfw.dir/cocoa_init.m.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_init.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/cocoa_init.m.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_init.m" -o CMakeFiles/glfw.dir/cocoa_init.m.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_joystick.m.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_joystick.m.o: ../3rd_party/glfw/src/cocoa_joystick.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_8) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_joystick.m.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/cocoa_joystick.m.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_joystick.m"

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_joystick.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/cocoa_joystick.m.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_joystick.m" > CMakeFiles/glfw.dir/cocoa_joystick.m.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_joystick.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/cocoa_joystick.m.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_joystick.m" -o CMakeFiles/glfw.dir/cocoa_joystick.m.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_monitor.m.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_monitor.m.o: ../3rd_party/glfw/src/cocoa_monitor.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_9) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_monitor.m.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/cocoa_monitor.m.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_monitor.m"

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_monitor.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/cocoa_monitor.m.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_monitor.m" > CMakeFiles/glfw.dir/cocoa_monitor.m.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_monitor.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/cocoa_monitor.m.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_monitor.m" -o CMakeFiles/glfw.dir/cocoa_monitor.m.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_window.m.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_window.m.o: ../3rd_party/glfw/src/cocoa_window.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_10) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_window.m.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/cocoa_window.m.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_window.m"

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_window.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/cocoa_window.m.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_window.m" > CMakeFiles/glfw.dir/cocoa_window.m.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_window.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/cocoa_window.m.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_window.m" -o CMakeFiles/glfw.dir/cocoa_window.m.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_time.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_time.c.o: ../3rd_party/glfw/src/cocoa_time.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_11) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_time.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/cocoa_time.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_time.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_time.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/cocoa_time.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_time.c" > CMakeFiles/glfw.dir/cocoa_time.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_time.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/cocoa_time.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/cocoa_time.c" -o CMakeFiles/glfw.dir/cocoa_time.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o: ../3rd_party/glfw/src/posix_tls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_12) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/posix_tls.c.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/posix_tls.c"

3rd_party/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/posix_tls.c.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/posix_tls.c" > CMakeFiles/glfw.dir/posix_tls.c.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/posix_tls.c.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/posix_tls.c" -o CMakeFiles/glfw.dir/posix_tls.c.s

3rd_party/glfw/src/CMakeFiles/glfw.dir/nsgl_context.m.o: 3rd_party/glfw/src/CMakeFiles/glfw.dir/flags.make
3rd_party/glfw/src/CMakeFiles/glfw.dir/nsgl_context.m.o: ../3rd_party/glfw/src/nsgl_context.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_13) "Building C object 3rd_party/glfw/src/CMakeFiles/glfw.dir/nsgl_context.m.o"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw.dir/nsgl_context.m.o   -c "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/nsgl_context.m"

3rd_party/glfw/src/CMakeFiles/glfw.dir/nsgl_context.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw.dir/nsgl_context.m.i"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/nsgl_context.m" > CMakeFiles/glfw.dir/nsgl_context.m.i

3rd_party/glfw/src/CMakeFiles/glfw.dir/nsgl_context.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw.dir/nsgl_context.m.s"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src/nsgl_context.m" -o CMakeFiles/glfw.dir/nsgl_context.m.s

# Object files for target glfw
glfw_OBJECTS = \
"CMakeFiles/glfw.dir/context.c.o" \
"CMakeFiles/glfw.dir/init.c.o" \
"CMakeFiles/glfw.dir/input.c.o" \
"CMakeFiles/glfw.dir/monitor.c.o" \
"CMakeFiles/glfw.dir/vulkan.c.o" \
"CMakeFiles/glfw.dir/window.c.o" \
"CMakeFiles/glfw.dir/cocoa_init.m.o" \
"CMakeFiles/glfw.dir/cocoa_joystick.m.o" \
"CMakeFiles/glfw.dir/cocoa_monitor.m.o" \
"CMakeFiles/glfw.dir/cocoa_window.m.o" \
"CMakeFiles/glfw.dir/cocoa_time.c.o" \
"CMakeFiles/glfw.dir/posix_tls.c.o" \
"CMakeFiles/glfw.dir/nsgl_context.m.o"

# External object files for target glfw
glfw_EXTERNAL_OBJECTS =

3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/context.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/init.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/input.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/monitor.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/vulkan.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/window.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_init.m.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_joystick.m.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_monitor.m.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_window.m.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/cocoa_time.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/posix_tls.c.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/nsgl_context.m.o
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/build.make
3rd_party/glfw/src/libglfw3.a: 3rd_party/glfw/src/CMakeFiles/glfw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_14) "Linking C static library libglfw3.a"
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean_target.cmake
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glfw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
3rd_party/glfw/src/CMakeFiles/glfw.dir/build: 3rd_party/glfw/src/libglfw3.a

.PHONY : 3rd_party/glfw/src/CMakeFiles/glfw.dir/build

3rd_party/glfw/src/CMakeFiles/glfw.dir/clean:
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean.cmake
.PHONY : 3rd_party/glfw/src/CMakeFiles/glfw.dir/clean

3rd_party/glfw/src/CMakeFiles/glfw.dir/depend:
	cd "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid" "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/3rd_party/glfw/src" "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build" "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src" "/Users/stacia/documents/University/MIT/Class/03-Junior/Fall/6-837 Computer Graphics/Final Project/code/6-837-final-project-boid/build/3rd_party/glfw/src/CMakeFiles/glfw.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : 3rd_party/glfw/src/CMakeFiles/glfw.dir/depend

