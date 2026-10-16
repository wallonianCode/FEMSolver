cmake_minimum_required(VERSION 3.14)

project(MyFEM LANGUAGES CXX)

# Compile C++ code using the C++20 language standard
set(CMAKE_CXX_STANDARD 20)
# Do not allow CMake to spontaneously decay to an older standards as the one specified.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Use standard C++ compiler modes rather than compiler-specific extensions, where the compiler
# supports that extension.
set(CMAKE_CXX_EXTENSIONS OFF)


include(FetchContent)

# Windows runtime settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
)

# Include GoogleMock
FetchContent_MakeAvailable(googletest)


# Enables CTest testing support in the build directory
enable_testing()

# Defining the FEM library target 
add_library(FEM 
            src/fem/Node.cpp
            src/fem/Element.cpp
            src/fem/Mesh.cpp)

# Let the FEM implementation files find their headers and propagates the include 
# directories to the test targets that link against FEM
target_include_directories(FEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fem
)

# Defining build targets
# source code -> object code
add_executable(NodeTests tests/NodeTests.cpp)
# resolves references to GoogleTest's compiled functions and test runner + 
# to necessary local libraries
# (link-time dependencies of the target)
# GTest::gmock_main provides GoogleMock, GoogleTest, and the entry point (main).
target_link_libraries(NodeTests PRIVATE FEM GTest::gtest_main)

add_executable(Tri3Tests tests/Tri3Tests.cpp)
target_link_libraries(Tri3Tests PRIVATE FEM GTest::gtest_main)

add_executable(MeshTests tests/MeshTests.cpp)
target_link_libraries(MeshTests PRIVATE FEM GTest::gmock_main)

# Register individual GoogleTest cases with CTest
include(GoogleTest)

# Automatically detect the individual test cases inside a GoogleTest executable 
# and register them with CTest.
# CTest runs registered tests and reports results.
# without this command, we would be limited to run every independent test source file 
# one after the other.

# Tests are registeres in the following way: NodeTests.DefaultConstruction, NodeTests.Coordinates...
gtest_discover_tests(NodeTests)
gtest_discover_tests(Tri3Tests)
gtest_discover_tests(MeshTests)

# cmake -S . -B build
#[[ 
    reads the contents of this file
    identify C++ compiler/build environment
    download and configure GoogleTest (if FetchContent() is used)
    generate the build files in the build directory.

    --> to be run again every time you change the CMakeList.txt file
]]

# cmake --build build --config Debug
#[[
    compiles the targets defined in this file along with their dependencies

    --build: build a build directory
    build  : the directory containing your generated build files
    --config Debug: selects the Debug configuration
        -> Includes debugging symbols, enables fewer compiler optimizations that may 
           clutter the debugging process by changing how the code executes.

    The other option is --config release
    
]]

# ctest --test-dir build --output-on-failure
#[[
    executes the tests registered with CTest and prints details about failures.
]]