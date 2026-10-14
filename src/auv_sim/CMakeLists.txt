cmake_minimum_required(VERSION 3.8)
project(auv_sim)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

# Vehicle SDF and the world meshes are generated at build time, so that every
# image which is having the workspace (sim, gui) gets the same model files.
# Vehicle mesh and textures (models/auv/meshes, materials) are copied in by the
# docker build from Gazebo Fuel.
set(GEN_MODELS ${CMAKE_CURRENT_BINARY_DIR}/models)
set(SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)

add_custom_command(
  OUTPUT ${GEN_MODELS}/auv/model.sdf
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_MODELS}/auv
  COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/models/auv/model.sdf.xacro
          -o ${GEN_MODELS}/auv/model.sdf
  DEPENDS models/auv/model.sdf.xacro
  COMMENT "Generating vehicle model.sdf from xacro"
)

add_custom_command(
  OUTPUT ${GEN_MODELS}/seabed/meshes/seabed.obj
  COMMAND ${Python3_EXECUTABLE} ${SCRIPTS}/generate_seabed.py --out-dir ${GEN_MODELS}/seabed
  DEPENDS scripts/generate_seabed.py scripts/terrain.py
  COMMENT "Generating seabed mesh and sand texture"
)

add_custom_command(
  OUTPUT ${GEN_MODELS}/seabed_features/meshes/rocks.obj
  COMMAND ${Python3_EXECUTABLE} ${SCRIPTS}/generate_features.py --out-dir ${GEN_MODELS}/seabed_features
  DEPENDS scripts/generate_features.py scripts/terrain.py
  COMMENT "Generating rock, pipeline and wreck meshes"
)

add_custom_target(generate_models ALL
  DEPENDS ${GEN_MODELS}/auv/model.sdf
          ${GEN_MODELS}/seabed/meshes/seabed.obj
          ${GEN_MODELS}/seabed_features/meshes/rocks.obj
)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN "*.xacro" EXCLUDE
)
install(DIRECTORY ${GEN_MODELS}/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
