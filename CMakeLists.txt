# cmake version to be used
cmake_minimum_required( VERSION 3.0 )

# project name
project(directAlign)

# external libs
find_package(Ceres REQUIRED)
# Or find_package(Ceres REQUIRED PATHS "/path/to/install/dir") if the install directory is not in your PATH

# external libs
find_package(OpenCV REQUIRED)

# target align_pc2equi
add_executable( align_pc2equi align_pc2equi.cpp )

target_include_directories(align_pc2equi
  PRIVATE
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(align_pc2equi
  PRIVATE
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)



