# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/jannik/eigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jannik/eigen/build

# Utility rule file for kronecker_product.

# Include the progress variables for this target.
include unsupported/test/CMakeFiles/kronecker_product.dir/progress.make

kronecker_product: unsupported/test/CMakeFiles/kronecker_product.dir/build.make

.PHONY : kronecker_product

# Rule to build all files generated by this target.
unsupported/test/CMakeFiles/kronecker_product.dir/build: kronecker_product

.PHONY : unsupported/test/CMakeFiles/kronecker_product.dir/build

unsupported/test/CMakeFiles/kronecker_product.dir/clean:
	cd /home/jannik/eigen/build/unsupported/test && $(CMAKE_COMMAND) -P CMakeFiles/kronecker_product.dir/cmake_clean.cmake
.PHONY : unsupported/test/CMakeFiles/kronecker_product.dir/clean

unsupported/test/CMakeFiles/kronecker_product.dir/depend:
	cd /home/jannik/eigen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jannik/eigen /home/jannik/eigen/unsupported/test /home/jannik/eigen/build /home/jannik/eigen/build/unsupported/test /home/jannik/eigen/build/unsupported/test/CMakeFiles/kronecker_product.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unsupported/test/CMakeFiles/kronecker_product.dir/depend

