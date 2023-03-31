include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0
)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)


add_executable(${PROJECT_NAME}-test
    test.cpp)
target_include_directories(${PROJECT_NAME}-test PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME}-test
    PRIVATE
        ${PROJECT_NAME}_lib
        GTest::GTest)

add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-test)
