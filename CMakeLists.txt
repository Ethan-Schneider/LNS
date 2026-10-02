cmake_minimum_required(VERSION 3.5)

project(lns)
set(PYBIND11_PYTHON_VERSION 3.11.7)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dlib REQUIRED)

include_directories("inc")
include_directories("src")
file(GLOB SOURCES "src/*.cpp")

file(GLOB INC_FILES "inc/*.h")
file(GLOB SRC_FILES "src/*.cpp")

ADD_LIBRARY(LibsModule 
    inc/AgentsLoader.h
    inc/BasicGraph.h
    inc/common.h
    inc/KivaGraph.h
    inc/LNS.h
    inc/ReservationTable.h
    inc/SingleAgentSolver.h
    inc/States.h
    inc/StateTimeAStar.h
    inc/TasksLoader.h

    src/AgentsLoader.cpp
    src/BasicGraph.cpp
    src/common.cpp
    src/KivaGraph.cpp
    src/LNS.cpp
    src/ReservationTable.cpp
    src/SingleAgentSolver.cpp
    src/StateTimeAStar.cpp
    src/TasksLoader.cpp
    src/States.cpp
)

pybind11_add_module(${PROJECT_NAME} src/pywrap.cpp)
pybind11_add_module(dc src/pywrap2.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE LibsModule)
target_link_libraries(dc PRIVATE LibsModule)
target_link_libraries(LibsModule PUBLIC dlib::dlib)

target_compile_definitions(${PROJECT_NAME} PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_include_directories(${PROJECT_NAME} PRIVATE ${PYBIND11_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

target_compile_definitions(dc PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_include_directories(dc PRIVATE ${PYBIND11_INCLUDE_DIRS})
target_link_libraries(dc PRIVATE Eigen3::Eigen)

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(lns PRIVATE ${Boost_LIBRARIES})
target_link_libraries(dc PRIVATE ${Boost_LIBRARIES})
