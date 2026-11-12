#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <vector>
#include "Eigen/Eigen"
#include "HYYRobotInterface.h"
using namespace HYYRobotBase;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Matrix76d = Eigen::Matrix<double, 7, 6>;

/**
 * @brief 机器人上下电示例
 */
int RobotEnableDemo();
/**
 * @brief 单臂运动示例
 * @return int 0：正常，其他：异常
 */
int MoveDemo();
/**
 * @brief 双臂运动示例
 * @return int 0：正常，其他：异常
 */
int DualArmMoveDemo();
/**
 * @brief 外部轴运动示例
 * @return int 0：正常，其他：异常
 */
int AdditionAxisDemo();
/**
 * @brief 移动到目标位姿实例
 * @return int 0：正常，其他：异常
 */
int MoveToPoseDemo();
/**
 * @brief 步进实例
 * @return int 0：正常，其他：异常
 */
int StepMoveDemo();
/**
 * @brief 关节空间速度控制实例
 * @return int 0：正常，其他：异常
 */
int SpeedAMoveDemo();
/**
 * @brief 笛卡尔空间速度控制实例
 * @return int 0：正常，其他：异常
 */
int SpeedLMoveDemo();
/**
 * @brief 笛卡尔空间速度控制实例
 * @return int 0：正常，其他：异常
 */
int CartesianServoDemo(robpose *rp, tool *to, wobj *wo, int robot_index);
/**
 * @brief 获取机器人运动状态实例
 * @return int 0：正常，其他：异常
 */
int GetRobotStateDemo();
/**
 * @brief 设置零位实例
 * @return int 0：正常，其他：异常
 */
int SetZeroDemo();
/**
 * @brief 获取关节限位信息
 * @return int 0：正常，其他：异常
 */
int GetLimitDemo();
/**
 * @brief 雅可比矩阵使用实例
 * @return int 0：正常，其他：异常
 */
int JacoDemo();
/**
 * @brief 关节跟随控制使用实例
 * @return int 0：正常，其他：异常
 */
int ServoJointDemo();
/**
 * @brief 笛卡尔末端增量跟随控制使用实例，绝对位置
 * @return int 0：正常，其他：异常
 */
int ServoCartesianDemo();
/**
 * @brief 笛卡尔末端跟随控制使用实例,相对基坐标系
 * @return int 0：正常，其他：异常
 */
int ServoCartesianOffBaseDemo();
/**
 * @brief 笛卡尔末端增量跟随控制使用实例，相对末端坐标系
 * @return int 0：正常，其他：异常
 */
int ServoCartesianOffEndDemo();

int TimerDemo();
