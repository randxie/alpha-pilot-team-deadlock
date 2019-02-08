cmake_minimum_required(VERSION 3.10.2)

project(ALPHA_PILOT VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# --------------------------------------------------------------------------------
# Compiler flags
# --------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)

# Things to always include as flags. Change as needed.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")  

# Build-type specific flags. Change as needed.
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

message(STATUS "Building with the following extra flags: ${CMAKE_CXX_FLAGS}")

# --------------------------------------------------------------------------------
# External packages
# --------------------------------------------------------------------------------
set(COMPILE_ROSCLIENT OFF)

# Fix cuda linking issues
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Find CUDA
find_package(CUDA)

# Find OpenCV
find_package( OpenCV REQUIRED )

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Only compile ROS bindings if asked.
if(NOT COMPILE_ROSCLIENT)
  message(STATUS "Not building ROSClient since COMPILE_ROSCLIENT=OFF. Edit the top level CMakeLists.txt to compile ROS bindings.")
  set(COMPILE_ROSCLIENT OFF)
else()
  message(STATUS "COMPILE_ROSCLIENT=ON, so building ROSClient.")
  
  # Init Catkin
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    tf2
    nav_msgs
    sensor_msgs
    geometry_msgs
  )
  catkin_package(
    LIBRARIES simulator
    CATKIN_DEPENDS roscpp rospy std_msgs
    # DEPENDS system_lib
    tf
    tf2
  )
endif()

# --------------------------------------------------------------------------------
# include all files
# --------------------------------------------------------------------------------
# We make sure that CMake sees all the files.
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/tests
    ${PROJECT_SOURCE_DIR}/third_party/googletest
    ${PROJECT_SOURCE_DIR}/third_party/googletest/include
    ${PROJECT_SOURCE_DIR}/third_party/flight_goggle_client/src)

# Make variables referring to all the sources and test files.
file(GLOB SOURCES "src/*.cc")
file(GLOB TESTFILES "tests/*.cc")
set(TEST_MAIN unit_tests.x)  # Default name for test executable

# --------------------------------------------------------------------------------
# Add FlightGoggleClient lib
# --------------------------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/flight_goggle_client/src)

# --------------------------------------------------------------------------------
# Build
# --------------------------------------------------------------------------------
# Compile all sources into a library. Called engine here (change if you wish).
add_library( engine ${SOURCES} )

# Add an executable for the file main.cpp, here called main.x.
# If you add more executables, copy these three lines accordingly.
add_executable(main.x src/main.cc)   # Name of exec. and location of file.
add_dependencies(main.x engine)       # main.cpp uses the 'engine', add dep.
target_link_libraries(main.x engine FlightGogglesClientLib pthread ${OpenCV_LIBS} ${Eigen3_LIBS})  # Link the executable to the 'engine'.

# --------------------------------------------------------------------------------
# Make Tests
# --------------------------------------------------------------------------------
# Add a make target 'gtest', that runs the tests (and builds all dependencies).
# The setup of Google Test is done at the very end of this file.
add_executable(${TEST_MAIN} ${TESTFILES})
add_dependencies(${TEST_MAIN} googletest engine)
target_link_libraries(${TEST_MAIN} googletest engine pthread ${OpenCV_LIBS} ${Eigen3_LIBS})
add_custom_target(gtest 
    COMMAND "${PROJECT_BINARY_DIR}/${TEST_MAIN}"
    DEPENDS engine ${TEST_MAIN})


# Add a standard make target 'test' that runs the tests under CTest (only as an alt. to gtest).
include(CTest)
enable_testing()
add_test(unit_tests ${PROJECT_BINARY_DIR}/${TEST_MAIN})

# --------------------------------------------------------------------------------
# Add Google Test
# --------------------------------------------------------------------------------
# The following makes sure that an up-to-date version of googletest is available,
# and built so that it may be used by your tests. 
add_custom_target( git_update
    COMMAND git submodule init
    COMMAND git submodule update
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} )
add_library( googletest
    ${PROJECT_SOURCE_DIR}/third_party/googletest/src/gtest-all.cc
    ${PROJECT_SOURCE_DIR}/third_party/googletest/src/gtest_main.cc )
add_dependencies(googletest git_update)
set_source_files_properties(${PROJECT_SOURCE_DIR}/third_party/googletest/src/gtest-all.cc  PROPERTIES GENERATED 1)
set_source_files_properties(${PROJECT_SOURCE_DIR}/third_party/googletest/src/gtest_main.cc PROPERTIES GENERATED 1)