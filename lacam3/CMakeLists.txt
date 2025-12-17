cmake_minimum_required(VERSION 3.16)
file(GLOB SRCS "./src/*.cpp")
project(lacam3)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE ./include)

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)

# Gurobi
set(GUROBI_ROOT "/Library/gurobi1203/macos_universal2" CACHE PATH
    "Gurobi installation root")
find_library(GUROBI_LIBRARY
    NAMES gurobi gurobi120
    HINTS "${GUROBI_ROOT}/lib"
    REQUIRED
)
find_library(GUROBI_CXX_LIBRARY
    NAMES gurobi_c++
    HINTS "${GUROBI_ROOT}/lib"
    REQUIRED
)
target_include_directories(${PROJECT_NAME} PUBLIC "${GUROBI_ROOT}/include")
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${GUROBI_LIBRARY}
    ${GUROBI_CXX_LIBRARY}
)
