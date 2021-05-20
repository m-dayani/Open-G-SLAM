
project(og_slam_core)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)
MESSAGE("-- OpenCV Version: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

# Note: One can add g2o as an external package:
#find_package(g2o REQUIRED)

#list(APPEND LIST_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

set(UTILS_DIR Utils)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_DIR})
set(UTILS_YAML_PARSER_DIR ${UTILS_DIR}/YamlParser)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_YAML_PARSER_DIR})

set(PARAMS_DIR Parameters)
list(APPEND LIST_INCLUDE_DIRS ${PARAMS_DIR})
set(PARAMS_TYPES_DIR ${PARAMS_DIR}/Types)
list(APPEND LIST_INCLUDE_DIRS ${PARAMS_TYPES_DIR})

set(DATA_TYPES_DIR DataTypes)
list(APPEND LIST_INCLUDE_DIRS ${DATA_TYPES_DIR})

set(DATA_LOADERS_DIR DataLoaders)
list(APPEND LIST_INCLUDE_DIRS ${DATA_LOADERS_DIR})
set(DATA_STORE_DIR ${DATA_LOADERS_DIR}/DataStore)
list(APPEND LIST_INCLUDE_DIRS ${DATA_STORE_DIR})

set(ALL_H_FILES
        ${UTILS_DIR}/DataConversion.h
        ${UTILS_YAML_PARSER_DIR}/YamlParserCV.h
        ${PARAMS_TYPES_DIR}/SensorConfig.h
        ${PARAMS_TYPES_DIR}/CamParams.h
        ${PARAMS_TYPES_DIR}/DS_Params.h
        ${PARAMS_TYPES_DIR}/IMU_Params.h
        ${PARAMS_TYPES_DIR}/Features2D.h
        ${PARAMS_TYPES_DIR}/ViewerParams.h
        ${PARAMS_TYPES_DIR}/EventParams.h
        ${PARAMS_DIR}/ParameterServer.h
        ${DATA_TYPES_DIR}/Image.h
        ${DATA_TYPES_DIR}/ImageFilter.h
        ${DATA_TYPES_DIR}/ImageHook.h
        ${DATA_TYPES_DIR}/IMU.h
        ${DATA_TYPES_DIR}/IMU_Hook.h
        ${DATA_TYPES_DIR}/Pose.h
        ${DATA_TYPES_DIR}/PoseHook.h
        ${DATA_TYPES_DIR}/SharedQueue.h
        ${DATA_STORE_DIR}/TabularTextDS.h
        ${DATA_STORE_DIR}/ImageDS.h
        ${DATA_STORE_DIR}/IMU_DS.h
        ${DATA_STORE_DIR}/PoseDS.h
        ${DATA_LOADERS_DIR}/BaseLoader.h
        ${DATA_LOADERS_DIR}/EurocLoader.h
)
set(ALL_SRC_FILES
        ${UTILS_DIR}/DataConversion.cpp
        ${UTILS_YAML_PARSER_DIR}/YamlParserCV.cpp
        ${PARAMS_TYPES_DIR}/SensorConfig.cpp
        ${PARAMS_TYPES_DIR}/CamParams.cpp
        ${PARAMS_TYPES_DIR}/DS_Params.cpp
        ${PARAMS_TYPES_DIR}/IMU_Params.cpp
        ${PARAMS_TYPES_DIR}/Features2D.cpp
        ${PARAMS_TYPES_DIR}/ViewerParams.cpp
        ${PARAMS_TYPES_DIR}/EventParams.cpp
        ${PARAMS_DIR}/ParameterServer.cpp
        ${DATA_TYPES_DIR}/Image.cpp
        ${DATA_TYPES_DIR}/ImageFilter.cpp
        ${DATA_TYPES_DIR}/ImageHook.cpp
        ${DATA_TYPES_DIR}/IMU.cpp
        ${DATA_TYPES_DIR}/IMU_Hook.cpp
        ${DATA_TYPES_DIR}/Pose.cpp
        ${DATA_TYPES_DIR}/PoseHook.cpp
        ${DATA_TYPES_DIR}/SharedQueue.cpp
        ${DATA_STORE_DIR}/TabularTextDS.cpp
        ${DATA_STORE_DIR}/ImageDS.cpp
        ${DATA_STORE_DIR}/IMU_DS.cpp
        ${DATA_STORE_DIR}/PoseDS.cpp
        ${DATA_LOADERS_DIR}/BaseLoader.cpp
        ${DATA_LOADERS_DIR}/EurocLoader.cpp
)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
#        ${Pangolin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        ${ALL_H_FILES}
        ${ALL_SRC_FILES}
)

list(APPEND ALL_BUILD_INTERFACE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
foreach(INCLUDE_DIR IN LISTS LIST_INCLUDE_DIRS)
    list(APPEND ALL_BUILD_INTERFACE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR}>)
endforeach()
#message(${ALL_BUILD_INTERFACE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${ALL_BUILD_INTERFACE_DIRS}
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_LIBS}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
#        ${Pangolin_LIBRARIES}
#        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${CMAKE_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so
        -lboost_system
        -lboost_filesystem
        -lboost_serialization
#        -lcrypto
)

install(TARGETS ${PROJECT_NAME}
        EXPORT OG_SLAM_CORE
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${ALL_H_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
