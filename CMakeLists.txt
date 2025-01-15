cmake_minimum_required(VERSION 2.6)

project(lns)
set(CMAKE_CXX_STANDARD 20)
set(PYBIND11_PYTHON_VERSION 3.12)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("inc")
include_directories("src")
file(GLOB SOURCES "src/*.cpp")

file(GLOB INC_FILES "inc/*.h")
file(GLOB SRC_FILES "src/*.cpp")

ADD_LIBRARY(LibsModule ${INC_FILES} ${SRC_FILES})

pybind11_add_module(${PROJECT_NAME} src/pywrap.cpp)
target_link_libraries(${PROJECT_NAME} LibsModule)

target_compile_definitions(${PROJECT_NAME} PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_include_directories(${PROJECT_NAME} PRIVATE ${PYBIND11_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(lns ${Boost_LIBRARIES})
