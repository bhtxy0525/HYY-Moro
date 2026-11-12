/**
 * @file torqueSensor_interface.h
 *
 * @brief  六维力传感器接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-9
 *
 */
#ifndef TORQUESENSOR_INTERFACE_H_
#define TORQUESENSOR_INTERFACE_H_

/*---------------------------- Includes ------------------------------------*/
#include "Base/RobotStruct.h"



#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 开启六维力传感器数据清零(临时有效)
 *
 * @param sensor_name 传感器的名字
 * @return int 0:成功，其他：失败
 */
extern int ForceSensorOpenBias(const char* sensor_name);

/**
 * @brief 关闭六维力传感器数据清零
 *
 * @param sensor_name 传感器的名字
 * @return int 0:成功，其他：失败
 */
extern int ForceSensorCloseBias(const char* sensor_name);

/**
 * @brief 是否设置六维力传感器数据偏置
 *
 * @param sensor_name 传感器的名字
 * @return unsigned int 0:否,其他:是
 */
extern unsigned int IsSensorOpenBias(const char* sensor_name);

/**
 * @brief 获取六维力传感器力数据(包含零点标定偏移)
 *
 * @param sensor_name 传感器的名字
 * @param force 返回传感器数据(单位：N, Nm)
 * @return int 0:成功，其他：失败
 */
extern int GetSensorForce(const char* sensor_name, double* force);

/**
 * @brief 获取工具/工件坐标系（工件在机器人上的情况）描述下环境对机器人的作用力
 *
 * @param sensor_name 传感器的名字
 * @param angle 机器人当前关节位置（rad or m）
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引
 * @param force 获取工具/工件坐标系（工件在机器人上的情况）描述下环境对机器人的作用力(单位：N, Nm)
 * @return int 0:成功，其他：失败
 */
extern int GetSensorForceTransformToTool(const char* sensor_name, double* angle,tool* rtool, wobj* rwobj, int robot_index,double* force);


#ifdef __cplusplus
}
}
#endif


#endif /* TORQUESENSOR_INTERFACE_H_ */
