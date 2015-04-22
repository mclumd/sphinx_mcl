cmake_minimum_required(VERSION 2.8.3)
project(sphinx_mcl)

find_package(PkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
pkg_check_modules(SPHINX REQUIRED sphinxbase)
pkg_check_modules(POCKETSPHINX REQUIRED pocketsphinx)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SPHINX_INCLUDE_DIRS}
  ${POCKETSPHINX_INCLUDE_DIRS}
)

message(status "keyphrase=${sphinx_mcl_SOURCE_DIR}")
set(mdir,${POCKETSPHINX_PREFIX}/share/pocketsphinx/model)
message(status "Modeldir=${POCKETSPHINX_PREFIX}/share/pocketsphinx/model")
add_definitions(-DMODELDIR="${POCKETSPHINX_PREFIX}/share/pocketsphinx/model")
add_definitions(-DKEYDIR="${sphinx_mcl_SOURCE_DIR}/data")
## Declare a cpp executable
 add_executable(${PROJECT_NAME}_node src/sphinx_mcl.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sphinx_md_node sphinx_md_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${SPHINX_LIBRARIES}
   ${POCKETSPHINX_LIBRARIES}
 )

