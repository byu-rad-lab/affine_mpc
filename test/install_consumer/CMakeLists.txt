# A project that uses an installed affine_mpc, found only through
# affine_mpc_DIR: OSQP is a private dependency, so neither OSQP's headers nor
# an osqp CMake package are needed (see the install-consumer CI job).
cmake_minimum_required(VERSION 3.18)
project(affine_mpc_install_consumer LANGUAGES CXX)

find_package(affine_mpc CONFIG REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE affine_mpc::affine_mpc)
target_compile_features(consumer PRIVATE cxx_std_17)
