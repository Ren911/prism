
set(SOURCES
    polygon.cpp
    polygon3d.cpp
    triangle.cpp
    math_funcs.cpp
    prism.cpp)

add_library(${PROJECT_NAME}_lib STATIC ${SOURCES})

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_lib)
