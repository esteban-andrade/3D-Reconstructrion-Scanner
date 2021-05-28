cmake_minimum_required(VERSION 2.9 FATAL_ERROR)

project(3D_reconstruction_pipeline)
set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

SET(CMAKE_BUILD_TYPE Release)
FIND_PACKAGE(Matlab REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES("${Matlab_ROOT_DIR}/extern/include")
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(icp_scale icp.cpp rply.c)
add_library(recontruction Reconstruction.cpp)
add_executable (3D_reconstruction_pipeline main.cpp)


if(WIN32 OR MSVC OR MSYS OR MINGW)
 add_compile_options(/bigobj)
 add_definitions(-DPCL_NO_PRECOMPILE)
 add_definitions(-DOS=1)
  message(STATUS "Linking WINDOWS C++")
   
   target_link_libraries (${PROJECT_NAME}  recontruction icp_scale ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
    target_link_libraries(${PROJECT_NAME}  ${Matlab_ROOT_DIR}/extern/lib/win64/microsoft/libMatlabEngine.lib)
    target_link_libraries(${PROJECT_NAME}  ${Matlab_ROOT_DIR}/extern/lib/win64/microsoft/libMatlabDataArray.lib)
endif()

if(UNIX AND NOT APPLE)
 add_definitions(-DOS=0)
  message(STATUS "Linking UNIX C++")
    target_link_libraries (${PROJECT_NAME}  recontruction icp_scale ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
    target_link_libraries(${PROJECT_NAME} ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabEngine.so)
    target_link_libraries(${PROJECT_NAME}  ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabDataArray.so)
    
    find_package(Threads)
   
     target_link_libraries(${PROJECT_NAME}  ${CMAKE_THREAD_LIBS_INIT})
endif()
