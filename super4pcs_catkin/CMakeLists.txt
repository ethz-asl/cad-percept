cmake_minimum_required(VERSION 2.8.3)
project(super4pcs_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(opengr
  GIT_REPOSITORY https://github.com/nmellado/Super4PCS.git
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cmake
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DCMAKE_INSTALL_PREFIX=./install
    ../opengr
     
  BUILD_COMMAND make
  INSTALL_COMMAND $(MAKE) install
)
