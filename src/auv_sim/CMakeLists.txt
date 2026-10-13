cmake_minimum_required(VERSION 3.8)
project(auv_sim)

find_package(ament_cmake REQUIRED)
find_package(auv_description REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

# Vehicle SDF and seabed mesh are generated at build time, so that every image
# which is having the workspace (sim, gui) gets the same model files.
set(GEN_MODELS ${CMAKE_CURRENT_BINARY_DIR}/models)
set(THRUSTERS_FILE ${auv_description_DIR}/../config/thrusters.yaml)

add_custom_command(
  OUTPUT ${GEN_MODELS}/auv/model.sdf
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_MODELS}/auv
  COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/models/auv/model.sdf.xacro
          thrusters_file:=${THRUSTERS_FILE} -o ${GEN_MODELS}/auv/model.sdf
  DEPENDS models/auv/model.sdf.xacro ${THRUSTERS_FILE}
  COMMENT "Generating vehicle model.sdf from xacro"
)

add_custom_command(
  OUTPUT ${GEN_MODELS}/seabed/meshes/seabed.obj
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GEN_MODELS}/seabed/meshes
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_seabed.py
          --out ${GEN_MODELS}/seabed/meshes/seabed.obj
  DEPENDS scripts/generate_seabed.py
  COMMENT "Generating seabed mesh"
)

add_custom_target(generate_models ALL
  DEPENDS ${GEN_MODELS}/auv/model.sdf ${GEN_MODELS}/seabed/meshes/seabed.obj
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
