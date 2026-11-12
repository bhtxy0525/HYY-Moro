# ===========================
# 第三方库绝对路径导入
# ===========================

# *************************************************
# 不推荐使用 CMAKE_SOURCE_DIR 如果库被 add_subdirectory 到别的工程, CMAKE_SOURCE_DIR 就变成,another_project, 路径会错。
# ${CMAKE_SOURCE_DIR}/thirdparty/urdfdom_model/include
# 正确写法：
# ${PROJECT_SOURCE_DIR}/thirdparty/urdfdom_model/include
# ${robot_sdk_SOURCE_DIR}/thirdparty/urdfdom_model/include
# 使用set(ROBOT_SDK_DIR ${robot_sdk_SOURCE_DIR})同一设置路径，方便所有的库的路径管理
# *************************************************
# ${ROBOT_SDK_DIR}
set(ROBOT_SDK_DIR ${CMAKE_SOURCE_DIR}/depend/robot_sdk)

function(add_thirdparty name)
    set(base_lib ${ROBOT_SDK_DIR}/lib/thirdparty)
    set(base_include ${ROBOT_SDK_DIR}/include/thirdparty)

    add_library(${name}_thirdparty SHARED IMPORTED GLOBAL)

    set_target_properties(${name}_thirdparty PROPERTIES
        IMPORTED_LOCATION ${base_lib}/lib${name}_thirdparty.so
        INTERFACE_INCLUDE_DIRECTORIES ${base_include}
    )
endfunction()

set(THIRDPARTY_LIST
    ompl
    fcl
    ccd
    toppra
    assimp
    yaml-cpp
    ruckig
    tinyxml2
    tinyxml
    octomap
    octomath
    draco
    minizip
    console_bridge
)

foreach(lib ${THIRDPARTY_LIST})
    add_thirdparty(${lib})
endforeach()


add_library(urdfdom_model_thirdparty SHARED IMPORTED GLOBAL)
set_target_properties(urdfdom_model_thirdparty PROPERTIES
    IMPORTED_LOCATION
    ${ROBOT_SDK_DIR}/lib/thirdparty/liburdfdom_model_thirdparty.so.3.0
    INTERFACE_INCLUDE_DIRECTORIES
    ${ROBOT_SDK_DIR}/include/thirdparty
)

# boost
add_library(boost_filesystem_thirdparty SHARED IMPORTED GLOBAL)
set_target_properties(boost_filesystem_thirdparty PROPERTIES
    IMPORTED_LOCATION
    ${ROBOT_SDK_DIR}/lib/thirdparty/libboost_filesystem_thirdparty.so

    INTERFACE_INCLUDE_DIRECTORIES
    ${ROBOT_SDK_DIR}/include/thirdparty
)
add_library(boost_serialization_thirdparty SHARED IMPORTED GLOBAL)
set_target_properties(boost_serialization_thirdparty PROPERTIES
    IMPORTED_LOCATION
    ${ROBOT_SDK_DIR}/lib/thirdparty/libboost_serialization_thirdparty.so

    INTERFACE_INCLUDE_DIRECTORIES
    ${ROBOT_SDK_DIR}/include/thirdparty
)

# pinocchio
add_library(pinocchio_default_thirdparty SHARED IMPORTED GLOBAL)
set_target_properties(pinocchio_default_thirdparty PROPERTIES
    IMPORTED_LOCATION
    ${ROBOT_SDK_DIR}/lib/thirdparty/libpinocchio_default_thirdparty.so

    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
    ${ROBOT_SDK_DIR}/include/thirdparty
)
add_library(pinocchio_parsers_thirdparty SHARED IMPORTED GLOBAL)
set_target_properties(pinocchio_parsers_thirdparty PROPERTIES
    IMPORTED_LOCATION
    ${ROBOT_SDK_DIR}/lib/thirdparty/libpinocchio_parsers_thirdparty.so

    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
    ${ROBOT_SDK_DIR}/include/thirdparty
)



# 封装 robot_base
add_library(robot_base INTERFACE)
target_include_directories(robot_base INTERFACE
    ${ROBOT_SDK_DIR}/../depend/robot_base/include
)
target_link_directories(robot_base INTERFACE
    ${ROBOT_SDK_DIR}/../depend/robot_base/lib
)
target_link_libraries(robot_base INTERFACE
    HYYRobotTechnology
    RobotControl
    modbus
    m pthread rt dl
)
