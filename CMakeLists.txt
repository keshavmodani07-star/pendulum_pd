cmake_minimum_required(VERSION 3.10)
project(pendulum_pd C)

set(CMAKE_C_STANDARD 11)

# Set MuJoCo path (edit if needed)
set(MUJOCO_PATH $ENV{MUJOCO_PATH})

include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

add_executable(pendulum src/main.c)

target_link_libraries(pendulum
    mujoco
    glfw
    GL
    m
)

