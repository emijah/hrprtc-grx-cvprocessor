cmake_minimum_required(VERSION 2.6)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH /opt/grx/share/hrpsys/cmake_modules/ CACHE STRING "set path to cmake_modules")
endif()

####
## settings for OpenRTM and OpenHRP
##
if(NOT QNXNTO)
  include(FindPkgConfig)
  include(CheckIncludeFiles)
  find_package(OpenRTM REQUIRED)
  find_package(OpenHRP REQUIRED)
  include_directories(${OPENRTM_INCLUDE_DIRS})
  link_directories(${OPENRTM_LIBRARY_DIRS})
  include_directories(${OPENHRP_INCLUDE_DIRS})
  link_directories(${OPENHRP_LIBRARY_DIRS})
  include_directories(/opt/grx/include)

  if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__LINUX__" )
  endif()

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

  add_executable(${comp_name}Comp ${comp_name}Comp.cpp ${comp_sources})
  if (EXISTS /usr/lib/libcv.so)
    target_link_libraries(${comp_name}Comp ${OPENHRP_LIBRARIES} cv cvaux highgui cxcore)
  else()
    target_link_libraries(${comp_name}Comp ${OPENHRP_LIBRARIES} opencv_core opencv_highgui opencv_imgproc opencv_objdetect)
  endif()

  set(target ${comp_name} ${comp_name}Comp)
  
  set(PERMISSIONS_755 OWNER_READ OWNER_WRITE OWNER_EXECUTE  GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  install(FILES cvadjuster.py DESTINATION script PERMISSIONS ${PERMISSIONS_755})
  install(TARGETS ${target}
    RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
    LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
  )
endif()
