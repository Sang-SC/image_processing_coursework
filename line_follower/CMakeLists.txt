cmake_minimum_required(VERSION 3.0.2)
project(line_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(OpenCV 4.2 REQUIRED) # 找OpenCV 4.2 安装路径

catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_opencv src/test_opencv.cpp)
target_link_libraries(test_opencv	${catkin_LIBRARIES})
target_link_libraries(test_opencv	${OpenCV_LIBRARIES})