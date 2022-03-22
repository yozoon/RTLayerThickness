cmake_minimum_required(VERSION 3.4)

project("RayThickness")

# set default build type
SET(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(ViennaLS REQUIRED)
find_package(ViennaRay REQUIRED)

set(exe "RTExtraction")
add_executable(${exe} ${exe}.cpp)
target_include_directories(${exe} PUBLIC ${VIENNARAY_INCLUDE_DIRS} ${VIENNALS_INCLUDE_DIRS})
target_link_libraries(${exe} ${VIENNARAY_LIBRARIES} ${VIENNALS_LIBRARIES} ${PROJECT_LIBRARIES})

# Copy files
file(COPY first.lvst DESTINATION ${PROJECT_BINARY_DIR})
file(COPY second.lvst DESTINATION ${PROJECT_BINARY_DIR})
