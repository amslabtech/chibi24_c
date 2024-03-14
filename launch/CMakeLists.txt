cmake_minimum_required(VERSION 3.8)
project(chibi24_c_launch)
# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# 実行ファイルを作成する
ament_auto_add_executable(${PROJECT_NAME}_node
src/launch.cpp
)

# ディレクトリのインストール
install(DIRECTORY
  launch
  files
  DESTINATION share/${PROJECT_NAME}/
)


ament_auto_package()
