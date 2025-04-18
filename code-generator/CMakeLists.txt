cmake_minimum_required(VERSION 3.10)
project(ral_demo)

find_package(robif2b)
find_package(hddc2b)
find_package(Eigen3)
find_package(orocos_kdl)
find_package(urdfdom_headers)
find_package(urdfdom)

add_executable(main
  main.cpp
  chainhdsolver_vereshchagin_fext.cpp
)

target_include_directories(main
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(main
  robif2b::ethercat
  robif2b::kelo
  robif2b::kinova_gen3
  hddc2b::hddc2b
  orocos-kdl
  urdfdom_headers::urdfdom_headers
  urdfdom::urdf_parser
  kdl_parser
)

target_compile_options(main
  PRIVATE
    -Wall
    -Wextra
)