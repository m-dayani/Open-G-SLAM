cmake_minimum_required(VERSION 3.10)
project(OG_SLAM VERSION 1.0)

include(GNUInstallDirs)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PROJ_CORE_PATH core)
set(PROJ_EXAMPLES_PATH examples)
set(PROJ_ROS_PATH ros)
set(PROJ_THIRDPARTY_PATH thirdparty)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/lib)
#set(DESTDIR ${CMAKE_INSTALL_PREFIX})


configure_file(${PROJ_CORE_PATH}/GlobalConfig.h.in ${PROJ_CORE_PATH}/GlobalConfig.h)

# External dependencies
add_subdirectory(${PROJ_THIRDPARTY_PATH}/g2o)
add_subdirectory(${PROJ_THIRDPARTY_PATH}/DBoW2)

add_subdirectory(${PROJ_EXAMPLES_PATH})
add_subdirectory(${PROJ_CORE_PATH})
# WARNING: Always source ros setup before config.
#add_subdirectory(${PROJ_ROS_PATH})


install(FILES ${PROJECT_BINARY_DIR}/${PROJ_CORE_PATH}/GlobalConfig.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT OG_SLAM_CORE
        FILE OG_SLAM_CORE.cmake
        NAMESPACE OG_SLAM::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
)

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/OG_SLAM_COREConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
# generate the version file for the config file
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/OG_SLAM_COREConfigVersion.cmake"
        VERSION "${OG_SLAM_VERSION_MAJOR}.${OG_SLAM_VERSION_MINOR}"
        COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/OG_SLAM_COREConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/OG_SLAM_COREConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
)

export(EXPORT OG_SLAM_CORE
        FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/OG_SLAM_CORE.cmake"
)



