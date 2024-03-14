cmake_minimum_required(VERSION 3.8)
project(chibi24_c_global_path_planner)

# find dependencies
find_package(ament_cmake_auto REQUIRED)


ament_auto_find_build_dependencies()
# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/a_star.cpp
  src/global_path_planner_node.cpp)


ament_auto_package()
