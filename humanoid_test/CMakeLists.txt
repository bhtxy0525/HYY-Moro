#cmake version
cmake_minimum_required(VERSION 3.16)

#project name
project(robotdemo VERSION 12.0.0)

#编译模式
set(CMAKE_BUILD_TYPE Release)#Debug / Release
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
enable_testing()

include_directories(
    ./depend/robot_base/include
    ./depend/eigen3
    ./examples
	./depend
	./depend/wbc_base/include
	./src/FDCC
	./src/AVP_TELE
)
find_package(pinocchio)
find_package(qpSWIFT REQUIRED)

link_directories(
	./depend/wbc_base/lib
)

aux_source_directory(./examples DIR_SRCS)
aux_source_directory(./src/FDCC FDCC_SRCS)
aux_source_directory(./src/AVP_TELE AVP_TELE_SRCS)

# 可执行文件输出目录
set(EXECUTABLE_OUTPUT_PATH /home/robot/Documents/humanoid_test)
add_executable(${PROJECT_NAME} ${DIR_SRCS} ${FDCC_SRCS} ${AVP_TELE_SRCS})

target_precompile_headers(${PROJECT_NAME} PRIVATE
    <Eigen/Dense>
    <pinocchio/fwd.hpp>
    <pinocchio/algorithm/kinematics.hpp>
    <pinocchio/algorithm/frames.hpp>
    <pinocchio/algorithm/jacobian.hpp>
    <pinocchio/algorithm/aba.hpp>
)

target_link_libraries(${PROJECT_NAME} 
	RobotControl 	
	RobotControlPlus	
	modbus 
	HYYRobotTechnology 
	m 
	pthread 
	rt 
	dl 
	xml2
	zmq
	WBC
	#opengjk_ce
	pinocchio::pinocchio
	#qpSWIFT::qpSWIFT-shared
	)


