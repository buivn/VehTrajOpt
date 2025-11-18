CMAKE_MINIMUM_REQUIRED(VERSION  3.20)

PROJECT(Runner CXX C)
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
# Search path for CMake include files.)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/include/glib-2.0) # for video recording


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

IF(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
ENDIF()


 SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fpermissive" CACHE STRING  
      "Flags used by the C++ compiler during debug builds." FORCE)
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fpermissive " 
      CACHE STRING "Flags used by the C++ compiler during release builds." FORCE) 
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -fpermissive " CACHE STRING
      "Flags used by the C++ compiler during release builds with debug information." FORCE) 

  SET(CMAKE_C_FLAGS_DEBUG "-g -O0" CACHE STRING
      "Flags used by the C compiler during debug builds." FORCE)
  SET(CMAKE_C_FLAGS_RELEASE "-O3 -Wno-unused-result -DNDEBUG" CACHE STRING
      "Flags used by the C compiler during release builds." FORCE)
  SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g" CACHE STRING
      "Flags used by the C compiler during release builds with debug information." FORCE)

ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
 
#############################################################################
#Find OpenGL and GLUT
#
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)
# for video recording
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
FIND_LIBRARY(AVCODEC_LIB avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
FIND_LIBRARY(AVFORMAT_LIB avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
FIND_LIBRARY(AVUTIL_LIB avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
FIND_LIBRARY(AVDEVICE_LIB avdevice)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
FIND_LIBRARY(SWSCALE_LIB swscale)

find_path(NUMA_INCLUDE_DIR numa.h)
find_library(NUMA_LIB numa)

find_path(X264_INCLUDE_DIR x264.h)
FIND_LIBRARY(X264_LIB x264)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0 IMPORTED_TARGET)
pkg_check_modules(RSVG REQUIRED librsvg-2.0 IMPORTED_TARGET)
pkg_check_modules(GLIB2  glib-2.0 gobject-2.0 gmodule-2 gio-2 IMPORTED_TARGET)
pkg_check_modules(CAIRO cairo IMPORTED_TARGET)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${SWSCALE_INCLUDE_DIR} ${AVDEVICE_INCLUDE_DIR} ${NUMA_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${X264_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTK3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${RSVG_INCLUDE_DIRS} ${CAIRO_INCLUDE_DIR})


#############################################################################
#Find Other Libs
#
FIND_LIBRARY(PNG_LIB png /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH}) 
FIND_LIBRARY(DL_LIB dl /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH}) 

#############################################################################
INCLUDE_DIRECTORIES(src)
AUX_SOURCE_DIRECTORY(src/Utils SRC_UTILS)
AUX_SOURCE_DIRECTORY(src/External SRC_EXTERNAL)
AUX_SOURCE_DIRECTORY(src/External/PQP SRC_PQP)
AUX_SOURCE_DIRECTORY(src/GP SRC_GP)
AUX_SOURCE_DIRECTORY(src/Programs   SRC_PROGRAMS)
AUX_SOURCE_DIRECTORY(src/TrajOpt   SRC_OPT)
 

ADD_EXECUTABLE(Runner
		${SRC_UTILS}
		${SRC_EXTERNAL}
		${SRC_GP}
        ${SRC_PQP}
		${SRC_CREATE_SCENES}
		${SRC_PROGRAMS}
        ${SRC_OPT}
	      )



MESSAGE("
*****************************************************************************
* EXTERNAL LIBRARIES
*   GL      : ${OPENGL_LIBRARIES}
*   GLUT   : ${GLUT_glut_LIBRARY}
*   DL_LIB: ${DL_LIB}
*  PNG_LIB: ${PNG_LIB}
*****************************************************************************
")

set_property(TARGET Runner PROPERTY ENABLE_EXPORTS 1)

TARGET_LINK_LIBRARIES(Runner
    ${DL_LIB} ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY} ${PNG_LIB}
    m pthread
    ${AVCODEC_LIB} ${AVFORMAT_LIB} ${AVUTIL_LIB} ${AVDEVICE_LIB} ${SWSCALE_LIB} #for video recording  
    ${X264_LIB} ${NUMA_LIB} ${RSVG_LIBRARY} ${GTK3_LIBRARIES} )


set(ENV 2)
set(INSTANCE 1)
set(AGENT_NUM 25)
set(PLANNER "madl_dist")
set(GOALS_SHAPE "square")
set(GOALDIST 1.0)
set(PENR 3)

add_custom_target(run_compile
COMMAND cmake -DCMAKE_BUILD_TYPE="Release"
    COMMAND make
)


add_custom_target(run_TrajOpt
    COMMAND ./bin/Runner RunKnown data/input/ParamsScene${ENV}.txt
)

add_custom_target(run_Display_TrajOpt
    COMMAND ./bin/Runner GRunUnknown data/ParamsScene${ENV}.txt
)

# add_custom_target(run_TrajOpt_env${ENV}_${AGENT_NUM}agents_instance${INSTANCE}_${GOALS_SHAPE}
#     COMMAND ./bin/Runner RunKnown data/evaluatingEnv/scene${ENV}/${AGENT_NUM}_agents/${GOALS_SHAPE}/instance${INSTANCE}_scene${ENV}_${AGENT_NUM}agents.txt ${GOALDIST} ${PENR}
#     )
