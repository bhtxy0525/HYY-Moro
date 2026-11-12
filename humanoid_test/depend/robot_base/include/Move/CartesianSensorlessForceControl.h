/**
 * @file CartesianSensorlessForceControl.h
 * 
 * @brief  无传感笛卡尔阻抗控制
 * @author HanBing
 * @version 12.4.1
 * @date 2026-04-30
 * 
 */

#ifndef CARTESIANSENSORLESSFORCECONTROL_H_
#define CARTESIANSENSORLESSFORCECONTROL_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/RobotStruct.h"

#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 初始化数据
 *
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int InitSensorlessCartesianImpedance(tool* rtool,wobj* rwobj, int robot_index);

/**
 * @brief 设置控制参数
 *
 * @param K 位置增益
 * @param D 速度增益
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int SetSensorlessCartesianImpedanceParameter(double* K, double* D,int robot_index);

/**
 * @brief 设置约束限制
 *
 * @param velocity_limit_per 速度约束限制（0~1）
 * @param torque_limit_per 力矩约束限制（0~1）
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int SetSensorlessCartesianImpedanceLimit(double velocity_limit_per,double torque_limit_per, int robot_index);

/**
 * @brief 设置摩擦补偿系数
 *
 * @param friction_coeff 摩擦补偿系数（0~1）
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int SetSensorlessCartesianImpedanceFrictionCoeff(double friction_coeff, int robot_index);

/**
 * @brief 开启无传感笛卡尔阻抗控制
 *
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int StartSensorlessCartesianImpedance(int robot_index);

/**
 * @brief 关闭无传感笛卡尔阻抗控制
 *
 * @param robot_index 机器人索引
 * @return int 成功返回0，失败返回其他
 */
extern int EndSensorlessCartesianImpedance(int robot_index);

#ifdef __cplusplus
}
}
#endif

#endif /* CARTESIANSENSORLESSFORCECONTROL_H_ */
