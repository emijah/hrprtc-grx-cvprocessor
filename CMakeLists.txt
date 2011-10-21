cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_modules/ CACHE STRING "set path to cmake_modules")
set(CMAKE_MODULE_PATH /opt/grx/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")

####
## settings for OpenRTM and OpenHRP
##
include(FindPkgConfig)
include(CheckIncludeFiles)
find_package(OpenRTM REQUIRED)
find_package(OpenHRP REQUIRED)
include_directories(${OPENRTM_INCLUDE_DIRS})
link_directories(${OPENRTM_LIBRARY_DIRS})
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})
include_directories(/opt/grx/include)

if(QNXNTO)
  include_directories(/usr/pkg/include)
else()
  if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__LINUX__" )
  endif()
endif(QNXNTO)

include(GenerateStubSkel)

set(comp_name CvProcessor)
set(comp_sources ${comp_name}.cpp)

generate_hrpsys_stub_skel_wrapper(Img)
set(comp_sources ${comp_sources} ImgSK.cc ImgDynSK.cc)

generate_stub_skel(${comp_name}Service)
set(comp_sources ${comp_sources} ${comp_name}Service_impl.cpp ${comp_name}ServiceSk.cpp)

add_library(${comp_name} SHARED ${comp_sources})
target_link_libraries(${comp_name} ${OPENHRP_LIBRARIES})
set_target_properties(${comp_name} PROPERTIES PREFIX "")

if(NOT QNXNTO)
  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  target_link_libraries(${comp_name}Comp ${OPENHRP_LIBRARIES} cv cvaux highgui)
  set(target ${comp_name} ${comp_name}Comp)
endif()

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
