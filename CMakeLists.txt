cmake_minimum_required(VERSION 2.8)

project(image_view_terminal)

find_package(catkin REQUIRED cv_bridge image_transport)
find_package(PkgConfig)

pkg_check_modules(CACA REQUIRED caca)

catkin_package()

include_directories(${CACA_INCLUDE_DIRS})

add_executable(image_view_terminal src/image_view_terminal.cpp)
target_link_libraries(image_view_terminal ${CACA_LIBRARIES} ${catkin_LIBRARIES})
