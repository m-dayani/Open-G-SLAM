
project(og_slam_examples)

set(LIB_OG_SLAM_NAME og_slam_core)



set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Test)

add_executable(data_loaders Test/data_loaders.cpp)
target_link_libraries(data_loaders LINK_PUBLIC ${LIB_OG_SLAM_NAME})
list(APPEND INSTALLABLE_EXE data_loaders)

#install(TARGETS ${INSTALLABLE_EXE} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/bin)

