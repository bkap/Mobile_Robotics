cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(MathyStuff src/MathyStuff.cpp)
rosbuild_add_library(CSpaceFuncs src/CSpaceFuncs.cpp)
rosbuild_add_library(camera_funcs src/camera_funcs.cpp)
rosbuild_add_library(Vision src/vision/cvblob.cpp src/vision/cvaux.cpp src/vision/cvcolor.cpp src/vision/cvcontour.cpp src/vision/cvlabel.cpp src/vision/cvtrack.cpp src/vision/lib_demo.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(command_publisher src/command_publisher.cpp)
rosbuild_add_executable(goalpublisher src/goalpublisher.cpp)
rosbuild_add_executable(planner src/planner.cpp)
rosbuild_add_executable(profiler src/profiler.cpp)
rosbuild_add_executable(mapper src/mapper.cpp)
rosbuild_add_executable(lidar src/lidar.cpp)
target_link_libraries(planner CSpaceFuncs)
target_link_libraries(mapper CSpaceFuncs)
target_link_libraries(profiler CSpaceFuncs)
rosbuild_add_executable(desiredpathcrawler src/desiredpathcrawler.cpp)
rosbuild_add_executable(steering src/steering.cpp)
rosbuild_add_executable(camera src/camera.cpp)
target_link_libraries(camera Vision)
target_link_libraries(camera camera_funcs)
rosbuild_add_executable(calib_camera src/vision/calib_camera.cpp)
target_link_libraries(calib_camera Vision)
rosbuild_add_executable(robust_calib src/vision/robust_calibration.cpp)
target_link_libraries(robust_calib Vision)
rosbuild_add_executable(orangelines src/orangelines.cpp)
target_link_libraries(orangelines camera_funcs)
rosbuild_add_executable(wesOrangeLines src/wesOrangeLines.cpp)
target_link_libraries(wesOrangeLines camera_funcs)

#FUTURE DEMO!!!!!!!!
#rosbuild_add_executable(futureplanner src/future/planner.cpp)
#rosbuild_add_executable(futuremapper src/future/mapper.cpp)
#target_link_libraries(futureplanner MathyStuff)
#target_link_libraries(futureplanner CSpaceFuncs)
#target_link_libraries(futuremapper CSpaceFuncs)
#rosbuild_add_executable(futurecamera src/future/camera.cpp)
#target_link_libraries(futurecamera Vision)
#target_link_libraries(orangelines src/orangelines.cpp)
#target_link_libraries(example ${PROJECT_NAME})
