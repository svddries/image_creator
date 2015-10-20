cmake_minimum_required(VERSION 2.8.3)
project(image_creator)

find_package(catkin REQUIRED COMPONENTS
    geolib2
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(image_creator
  src/particle_filter.cpp
  src/lrf.cpp
  src/image_writer.cpp
  src/create_images.cpp
  src/segmentation.cpp
  src/canvas.cpp
  src/relative.cpp
  src/world_model.cpp
  src/bad_localization.cpp
)
target_link_libraries(image_creator ${catkin_LIBRARIES})

add_executable(create-images src/create_images.cpp)
target_link_libraries(create-images image_creator ${catkin_LIBRARIES})


