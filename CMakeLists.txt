cmake_minimum_required( VERSION 2.8 )
project( test-plot )

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( test-plot src/main.cpp  )
target_link_libraries( test-plot ${OpenCV_LIBS} 
                            ${EIGEN3_LIBARIES}
                            ${CERES_LIBRARIES} )
cmake_minimum_required(VERSION 3.0)
project(test-plot)