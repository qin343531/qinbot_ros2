cmake_minimum_required(VERSION 3.5)
project(ares_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装scripts目录下的所有文件（包括子目录）
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                   GROUP_READ GROUP_EXECUTE
                   WORLD_READ WORLD_EXECUTE
  PATTERN "*"
)

# 安装其他目录
install(
  DIRECTORY config launch meshes urdf world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
