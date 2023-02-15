cmake_minimum_required(VERSION 3.7)
project(BestPos)
set(CMAKE_CXX_STANDARD 11)

#------------------------------------------------------------------------------
# Create a library
#------------------------------------------------------------------------------
add_library(BestPos SHARED
    src/PosComputator.cpp
    src/PosFilter.cpp
    src/PosNLLS.cpp
    src/PosUKF.cpp
)

include_directories(/usr/local/openjdk-8/include /usr/local/openjdk-8/include/linux)

target_include_directories(BestPos 
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/external_lib
)


#------------------------------------------------------------------------------
# Create an executable
#------------------------------------------------------------------------------
add_executable(demo 
    src/main.cpp
)

target_link_libraries(demo
    PRIVATE
        BestPos
)