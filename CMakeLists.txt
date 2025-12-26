cmake_minimum_required(VERSION 3.10)
project(pendulum_pd C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)




set(MUJOCO_DIR ${CMAKE_SOURCE_DIR}/..)

set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARY ${MUJOCO_DIR}/lib/libmujoco.so)

find_package(glfw3 REQUIRED)


add_executable(single_pendulum
    src/main.c
)

target_include_directories(single_pendulum PRIVATE
    ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(single_pendulum
    ${MUJOCO_LIBRARY}
    glfw
    GL
    m
    dl
)


add_executable(double_pendulum
    src/double_main.c
)

target_include_directories(double_pendulum PRIVATE
    ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(double_pendulum
    ${MUJOCO_LIBRARY}
    glfw
    GL
    m
    dl
)

