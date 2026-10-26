cmake_minimum_required(VERSION 3.14)

project(FEMSolver LANGUAGES CXX)

# Compile C++ code using the C++20 language standard
set(CMAKE_CXX_STANDARD 20)
# Do not allow CMake to spontaneously decay to an older standards as the one specified.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Use standard C++ compiler modes rather than compiler-specific extensions, where the compiler
# supports that extension.
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the Eigen3 package
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

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

# Tells CMake to build an executable named FEM from those source files.
add_library(FEM 
            src/fem/Node.cpp
            src/fem/Element.cpp
            src/fem/Mesh.cpp
            src/fem/boundary/BoundaryCondition.cpp
            src/fem/loads/LoadConditions.cpp
            src/fem/Material.cpp
            src/fem/DofManager.cpp
            src/fem/assembly/ElementStiffness.cpp
            src/fem/assembly/GlobalStiffnessAssembler.cpp
            )

# specifies which libraries a particular target depends on
# tells cmake to locate the Eigen package
# Public: dependency propagated to ev. dependent targets
target_link_libraries(FEM PUBLIC Eigen3::Eigen)

# Let the FEM implementation files find their headers and propagates 
# the include directories to the test targets that link against FEM
target_include_directories(FEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fem
)

# W4: Enable all warnings, including level 4 warnings, which are the most verbose and include warnings about code that is not necessarily incorrect but may indicate potential issues or bad practices.
# WX: Treat all warnings as errors, which means that if the compiler generates any warnings, the build will fail. This is useful for enforcing strict code quality standards and ensuring that all warnings are addressed before the code is considered complete.
#-Werror can make builds fail after compiler upgrades introduce new warnings, so it is not used in production code.

#-Wall: Enable all the commonly used warning messages that are considered important for catching potential issues in the code.
#-Wextra: Enable additional warning messages that are not included in -Wall but are still useful for catching potential issues.
#-Wpedantic: Enforce strict compliance with the C++ standard and generate warnings for any code that does not conform to the standard. This can help catch potential issues related to non-standard code or compiler-specific extensions.
target_compile_options(FEM PRIVATE 
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
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

add_executable(BoundaryConditionTests tests/BoundaryConditionTests.cpp)
target_link_libraries(BoundaryConditionTests PRIVATE FEM GTest::gmock_main)

add_executable(LoadConditionTests tests/LoadConditionTests.cpp)
target_link_libraries(LoadConditionTests PRIVATE FEM GTest::gmock_main)

add_executable(ElementStiffnessTests tests/ElementStiffnessTests.cpp)
target_link_libraries(ElementStiffnessTests PRIVATE FEM GTest::gmock_main)

add_executable(solver src/main.cpp)
target_link_libraries(solver PRIVATE FEM) #one shared definition of the FEM library is linked to the solver executable
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
gtest_discover_tests(BoundaryConditionTests)
gtest_discover_tests(LoadConditionTests)
gtest_discover_tests(ElementStiffnessTests)

# cmake -S . -B build-gcc
#[[ 
    = runs the cmake build-generator
    
    reads the contents of this file
    identify C++ compiler/build environment
    download and configure GoogleTest (if FetchContent() is used)
    generate the build files in the build directory.

    --> to be run again every time you change the CMakeList.txt file

    explicitely select GCC and GNU Makefiles: 

        cmake -S . -B build-gcc -G "MinGW Makefiles" 
        -DCMAKE_CXX_COMPILER="C:/msys64/ucrt64/bin/g++.exe"

    S: source directory

]]

# cmake --build build-gcc --config Debug
#[[
    compiles the targets defined in this file along with their dependencies

    --build: build a build directory
    build  : the directory containing your generated build files
    --config Debug: selects the Debug configuration
        -> Includes debugging symbols, enables fewer compiler optimizations that may 
           clutter the debugging process by changing how the code executes.

    The other option is --config release

    -clean-first: cleans the build directory before building the targets. 
                This ensures that any previous build artifacts are removed, 
                and the build starts from a clean state. 
                This can be useful when you want to ensure that the build 
                is not affected by any previous builds or changes in the source code.
                Without this option, CMake will only rebuild the targets that have 
                changed since the last build, which can be faster but may not catch 
                all issues if there are dependencies between targets.
]]

# ctest --test-dir build-gcc --output-on-failure
#[[
    executes the tests registered with CTest and prints details about failures.

    --verbose flag: see individual test cases and their output, even if they pass. 
]]

# ctest --test-dir build-gcc -R **pattern**
#[[
	runs tests defined by **pattern**
]]

