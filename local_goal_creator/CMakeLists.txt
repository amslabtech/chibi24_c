cmake_minimum_required(VERSION 3.8)
project(chibi24_c_local_goal_creator)

# find dependencies
find_package(ament_cmake_auto REQUIRED)


ament_auto_find_build_dependencies()
# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
  src/local_goal_creator.cpp
  src/local_goal_creator_node.cpp)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

  
ament_auto_package()
