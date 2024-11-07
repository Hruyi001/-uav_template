# cmake_minimum_required(VERSION 3.1)
# project(newTemplate)

# # 设置OpenCV和GDAL的查找路径
# set(OpenCV_DIR "root/opencv/cmake")
# set(GDAL_DIR "/usr/gdal/cmake")

# # 查找OpenCV和GDAL包
# find_package(OpenCV REQUIRED)
# find_package(GDAL REQUIRED)

# # 将OpenCV和GDAL的头文件目录添加到包含路径
# include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${GDAL_INCLUDE_DIRS})

# # 设置C++标准为C++11
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# # 设置可执行文件的输出目录为项目根目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

# # 添加可执行文件
# add_executable(newTemplate new_template.cpp)

# # 链接OpenCV和GDAL库到可执行文件
# target_link_libraries(newTemplate PRIVATE ${OpenCV_LIBS} ${GDAL_LIBRARIES})



cmake_minimum_required(VERSION 3.1)
project(newTemplate)

# 设置 OpenCV 和 GDAL 的查找路径
set(OpenCV_DIR "root/opencv/cmake")
set(GDAL_DIR "/usr/gdal/cmake")

# 查找 OpenCV 和 GDAL 包
find_package(OpenCV REQUIRED)
find_package(GDAL REQUIRED)

# 将 OpenCV 和 GDAL 的头文件目录添加到包含路径
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GDAL_INCLUDE_DIRS})

# 设置 C++ 标准为 C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 设置可执行文件的输出目录为项目根目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

# 添加 template1 库
add_library(template1 STATIC template1.cpp)
target_link_libraries(template1 PRIVATE ${OpenCV_LIBS} ${GDAL_LIBRARIES} pthread)

# 添加 main 可执行文件
add_executable(main main.cpp)
target_link_libraries(main PRIVATE template1 ${OpenCV_LIBS} ${GDAL_LIBRARIES})