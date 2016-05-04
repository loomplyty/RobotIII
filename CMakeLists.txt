#cmake_minimum_required(VERSION 2.8)

## add prefix to every string in a variable
#FUNCTION(PREPEND var prefix)
#   SET(listVar "")
#   FOREACH(f ${ARGN})
#      LIST(APPEND listVar "${prefix}/${f}")
#   ENDFOREACH(f)
#   SET(${var} "${listVar}" PARENT_SCOPE)
#ENDFUNCTION(PREPEND)

#FUNCTION(PREPEND_CPP var prefix)
#   SET(listVar "")
#   FOREACH(f ${ARGN})
#      LIST(APPEND listVar "${prefix}/${f}.cpp")
#   ENDFOREACH(f)
#   SET(${var} "${listVar}" PARENT_SCOPE)
#ENDFUNCTION(PREPEND_CPP)

#FUNCTION(PREPEND_H var prefix)
#   SET(listVar "")
#   FOREACH(f ${ARGN})
#      LIST(APPEND listVar "${prefix}/${f}.h")
#   ENDFOREACH(f)
#   SET(${var} "${listVar}" PARENT_SCOPE)
#ENDFUNCTION(PREPEND_H)

#project(CMakeDemo_by_PY)

#if(UNIX)
#        set(CMAKE_C_COMPILER "/usr/bin/gcc")
#        set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#endif(UNIX)

## set binary and library folders
#set(EXECUTABLE_OUTPUT_PATH bin)
#set(LIBRARY_OUTPUT_PATH lib)

## set debug and release options
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
#set(CMAKE_BUILD_TYPE "Release")
##set(CMAKE_CONFIGURATION_TYPES Debug Release)

## set compile options
#if(UNIX)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#endif(UNIX)

## set include and lib folders
#if(UNIX)

#set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
#set(SYSTEM_LINK_LIB pthread)
#set(XENOMAI_INCL_DIR "/usr/xenomai/include")
#set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
#set(XENOMAI_LINK_LIB native rtdm xenomai)
#set(ETHERLAB_INCL_DIR "/opt/etherlab/include")
#set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
#set(ETHERLAB_LINK_LIB ethercat rt)
#set(XSENS_INCL_DIR "/usr/Aris_Dependent/xsens/include")
#set(XSENS_LINK_LIB xdeviceclass xcommunication xstypes)
#set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")
#set(XNNITE_INCL_DIR "/usr/include/nite")#Driver for kinect
#set(XNNITE_LINK_DIR "/usr/lib")
#set(XNNITE_LINK_LIB )
#set(OPENNI_INCL_DIR "/usr/include/ni")#open NI for kinect
#set(OPENNI_LINK_DIR "/usr/lib")
#set(OPENNI_LINK_LIB OpenNI)
#set(ARIS_INCL_DIR "/usr/Aris/include")
#set(ARIS_LINK_DIR "/usr/Aris/lib")
#set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
#set(ROBOTS_INCL_DIR "/usr/Robots/include")
#set(ROBOTS_LINK_DIR "/usr/Robots/lib")
#set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)

#set(ALL_INCL_DIR ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${OPENNI_INCL_DIR} ${XNNITE_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
#set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${OPENNI_LINK_DIR} ${XNNITE_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
#set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${OPENNI_LINK_LIB} ${XNNITE_LINK_LIB} ${XSENS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})

#set(VISION_INCL_DIR "./Aris_Vision")

#set(ALL_INCL_DIR ${VISION_INCL_DIR} ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${OPENNI_INCL_DIR} ${XNNITE_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
#set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${OPENNI_LINK_DIR} ${XNNITE_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
#set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${OPENNI_LINK_LIB} ${XNNITE_LINK_LIB} ${XSENS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
#endif(UNIX)

#include_directories(${ALL_INCL_DIR})
#link_directories(${ALL_LINK_DIR})


#################################### build server ####################################
#set(SOURCE_FILES main.cpp)
#PREPEND(FULL_SRC Server ${SOURCE_FILES})

#set(VISION_SOURCE_FILES Aris_Vision.cpp Vision_Control0.cpp Vision_Terrain0.cpp Vision_Gait0.cpp )
#PREPEND(VISION_FULL_SRC Aris_Vision ${VISION_SOURCE_FILES})

#add_executable(Server ${FULL_SRC} ${VISION_FULL_SRC})
#target_link_libraries(Server ${ALL_LINK_LIB})

#################################### build client ####################################
#add_executable(Client Client/client.cpp)
#target_link_libraries(Client ${ALL_LINK_LIB})

#################################### install ####################################
#if(UNIX)
#        set(CMAKE_INSTALL_PREFIX "/usr/Robots/CMakeDemo/")
#endif(UNIX)
#if(WIN32)
#        set(CMAKE_INSTALL_PREFIX "C:/Robots/CMakeDemo/")
#endif(WIN32)

#install(TARGETS Server Client RUNTIME DESTINATION bin)

cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(VisionStairs)

if(UNIX)
        set(CMAKE_C_COMPILER "/usr/bin/gcc")
        set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#	set(CMAKE_C_COMPILER "/usr/bin/clang")
#	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
add_definitions(-DWIN32 -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
add_definitions(-DUNIX)
endif(UNIX)

# set include and lib folders
if(UNIX)
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/opt/etherlab/include")
set(ETHERLAB_LINK_DIR "/opt/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat rt)
set(XSENS_INCL_DIR "/usr/Aris_Dependent/xsens/include")
set(XSENS_LINK_LIB xdeviceclass xcommunication xstypes)
set(XSENS_LINK_DIR "/usr/Aris_Dependent/xsens/lib")
set(XNNITE_INCL_DIR "/usr/include/nite")#Driver for kinect
set(XNNITE_LINK_DIR "/usr/lib")
set(XNNITE_LINK_LIB )
set(OPENNI_INCL_DIR "/usr/include/ni")#open NI for kinect
set(OPENNI_LINK_DIR "/usr/lib")
set(OPENNI_LINK_LIB OpenNI)
set(ARIS_INCL_DIR "/usr/aris/include")
set(ARIS_LINK_DIR "/usr/aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
set(ROBOTS_INCL_DIR "/usr/Robots/include")
set(ROBOTS_LINK_DIR "/usr/Robots/lib")
set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)

set(VISION_INCL_DIR "./Aris_Vision")

set(ALL_INCL_DIR ${VISION_INCL_DIR} ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${OPENNI_INCL_DIR} ${XNNITE_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${OPENNI_LINK_DIR} ${XNNITE_LINK_DIR} ${XSENS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${OPENNI_LINK_LIB} ${XNNITE_LINK_LIB} ${XSENS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)
if(WIN32)
set(GENERAL_INCL_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIB ws2_32 setupapi)
set(XSENS_INCL_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIB xdeviceclass)
set(ARIS_INCL_DIR "C:/Aris/include")
set(ARIS_LINK_DIR "C:/Aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_dynamic aris_control aris_core)
set(ROBOTS_INCL_DIR "C:/Robots/include")
set(ROBOTS_LINK_DIR "C:/Robots/lib")
set(ROBOTS_LINK_LIB Robot_Type_I Robot_Type_II Robot_Base)

set(ALL_INCL_DIR ${ROBOTS_INCL_DIR} ${ARIS_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ROBOTS_LINK_DIR} ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} )
set(ALL_LINK_LIB ${ROBOTS_LINK_LIB} ${ARIS_LINK_LIB} ${XSENS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(WIN32)

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

################################### build server ####################################
set(SOURCE_FILES main.cpp continuous_walk_with_force.cpp push_recovery.cpp interpolation.cpp continuous_move.cpp twist_waist.cpp say_hello.cpp peg_in_hole.cpp swing.cpp MovePushDoorSimple.cpp PushDoorSimple.cpp MoveCrowdPassingGait.cpp CrowdPassingPlanner.cpp)
PREPEND(FULL_SRC Server ${SOURCE_FILES})

set(VISION_SOURCE_FILES Aris_Vision.cpp Vision_Control0.cpp Vision_Terrain0.cpp Vision_Gait0.cpp )
PREPEND(VISION_FULL_SRC Aris_Vision ${VISION_SOURCE_FILES})

set(MOTION_SOURCE_FILES Motions.cpp )
PREPEND(MOTION_FULL_SRC  Server ${MOTION_SOURCE_FILES})

set(REFLEX_SOURCE_FILES rofo2.cpp Gait.cpp Line.cpp Communication.cpp log.cpp Compatible.h log.h log_data.h)
PREPEND(REFLEX_SRC  Server ${REFLEX_SOURCE_FILES})

add_executable(Server ${FULL_SRC} ${VISION_FULL_SRC} ${MOTION_FULL_SRC} ${REFLEX_SRC})
target_link_libraries(Server ${ALL_LINK_LIB})

################################### build client ####################################
add_executable(rbt Client/client.cpp)
target_link_libraries(rbt ${ALL_LINK_LIB})









































