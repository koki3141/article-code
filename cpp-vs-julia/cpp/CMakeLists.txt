cmake_minimum_required(VERSION 3.0.0)
project(Heat VERSION 0.1.0 LANGUAGES C CXX)

add_executable(Heat main.cc)

set_target_properties(Heat PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
