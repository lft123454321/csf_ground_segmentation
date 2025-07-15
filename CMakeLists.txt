cmake_minimum_required(VERSION 3.0.2)
project(csf_ground_segmentation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# === CSF LIBRARY ===
set(CSF_SOURCES
    src/csf/c2cdist.cpp
    src/csf/Cloth.cpp
    src/csf/Constraint.cpp
    src/csf/CSF.cpp
    src/csf/Particle.cpp
    src/csf/point_cloud.cpp
    src/csf/Rasterization.cpp
    src/csf/XYZReader.cpp
)
set(CSF_HEADERS
    src/csf/c2cdist.h
    src/csf/Cloth.h
    src/csf/Constraint.h
    src/csf/CSF.h
    src/csf/Particle.h
    src/csf/point_cloud.h
    src/csf/Rasterization.h
    src/csf/Vec3.h
    src/csf/XYZReader.h
)
# add_library(CSF STATIC ${CSF_SOURCES} ${CSF_HEADERS})
# target_include_directories(CSF PUBLIC src/csf)

# if (MSVC)
#     add_compile_options(/W4 /WX)
# else()
#     add_compile_options(-Wall -Wextra -pedantic -Werror)
# endif()

# === 主节点可执行文件 ===
add_executable(csf_ground_segmentation_node src/csf_ground_segmentation_node.cpp ${CSF_SOURCES})
target_link_libraries(csf_ground_segmentation_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
