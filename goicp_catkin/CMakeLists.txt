cmake_minimum_required(VERSION 2.8.3)
project(goicp_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(goicp
  GIT_REPOSITORY https://github.com/yangjiaolong/Go-ICP.git
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cmake
    -DCMAKE_BUILD_TYPE:STRING=Release
    ../goicp

  BUILD_COMMAND make
  INSTALL_COMMAND $(MAKE) && cp -u GoICP ${CMAKE_CURRENT_LIST_DIR}
)
