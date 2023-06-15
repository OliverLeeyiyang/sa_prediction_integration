cmake_minimum_required(VERSION 3.8)
project(cpp_to_python)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT REQUIRED)
# add more dependencies here

include_directories(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

set(MODULE_NAME  dealBinMsg)

add_library(${MODULE_NAME} SHARED src/dealBinMsg.cpp)

if (UNIX)
  set_target_properties(${MODULE_NAME}
    PROPERTIES
    PREFIX ""
  )
elseif (WIN32)
  set_target_properties(${MODULE_NAME}
  PROPERTIES
  SUFFIX ".pyd"
  )
endif()

target_link_libraries(${MODULE_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

ament_export_dependencies(ament_cmake rclcpp)
ament_export_include_directories(include)

install(TARGETS ${MODULE_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


ament_package()
