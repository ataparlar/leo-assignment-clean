cmake_minimum_required(VERSION 3.5)
project(point_cloud_registering)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(LibKML REQUIRED)
find_package(GeographicLib REQUIRED)


if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_LIBRARY_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_LIBRARY_DIRS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

endif()








if( LIBKML_INCLUDE_DIR )
    # Already in cache, be silent
    set( LibKML_FIND_QUIETLY TRUE )
endif()

find_path( LIBKML_INCLUDE_DIR       kml/dom.h )
find_path( LIBKML_INCLUDE_DIR       kml/engine.h )
find_path( LIBKML_INCLUDE_DIR       kml/base.h )
find_library( LIBKML_BASE_LIBRARY   NAMES kmlbase )
find_library( LIBKML_DOM_LIBRARY    NAMES kmldom )
find_library( LIBKML_ENGINE_LIBRARY NAMES kmlengine )

mark_as_advanced(
  LIBKML_INCLUDE_DIR
  LIBKML_BASE_LIBRARY
  LIBKML_DOM_LIBRARY
  LIBKML_ENGINE_LIBRARY
  )

# handle the QUIETLY and REQUIRED arguments and set LIBKML_FOUND to TRUE if
# all listed variables are TRUE
include( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  LibKML
  DEFAULT_MSG
  LIBKML_INCLUDE_DIR
  LIBKML_BASE_LIBRARY
  LIBKML_DOM_LIBRARY
  LIBKML_ENGINE_LIBRARY
  )

if(LIBKML_FOUND)
  set(LIBKML_INCLUDE_DIRS ${LIBKML_INCLUDE_DIR})
  set(LIBKML_LIBRARIES
    ${LIBKML_BASE_LIBRARY}
    ${LIBKML_DOM_LIBRARY}
    ${LIBKML_ENGINE_LIBRARY}
    )
endif()






# Point Cloud Registration

add_executable(point_cloud_registration src/point_cloud_registration.cpp)
ament_target_dependencies(point_cloud_registration
tf2
tf2_ros
PCL
Eigen3
geometry_msgs
sensor_msgs
rclcpp)
install(TARGETS
point_cloud_registration
DESTINATION lib/${PROJECT_NAME}
)

# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries( point_cloud_registration ${PCL_LIBRARIES} )


# KML Parser

add_executable(kml_parser src/kml_parser.cpp)
ament_target_dependencies(kml_parser
tf2
LibKML
GeographicLib
geometry_msgs
nav_msgs
sensor_msgs
rclcpp)
install(TARGETS
kml_parser
DESTINATION lib/${PROJECT_NAME}
)

# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries( kml_parser
${LIBKML_BASE_LIBRARY}
${LIBKML_DOM_LIBRARY}
${LIBKML_ENGINE_LIBRARY}
${GeographicLib_LIBRARIES}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
