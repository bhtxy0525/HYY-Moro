/**
 * @file egm_interface.h
 *
 * @brief  机器人控制系统外部设备引导接口
 * @author hanbing
 * @version 11.0.0
 * @date 2020-04-08
 *
 */

#ifndef EGM_INTERFACE_H_
#define EGM_INTERFACE_H_
#include "Base/RobotStruct.h"


#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 创建外部修正数据区(用于控制器内使用)
 *
 * @param input_type 1:启用外部关节输入 2:启用外部工件坐标系笛卡尔输入 3:启用外部工具坐标系笛卡尔输入
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMCreate(int input_type,tool* rtool, wobj* rwobj,int robot_index);

/**
 * @brief 开始修正(用于控制器内使用)
 *
 * @param guide_type 修正方式,0:相对; 1:绝对
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMStart(int guide_type, int robot_index);

/**
 * @brief 启用外部引导(用于控制器内使用),将禁止执行运动指令
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMGuideMove(int robot_index);

/**
 * @brief 停止修正(用于控制器内使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMStop(int robot_index);

/**
 * @brief 释放外部修正数据区(用于控制器内使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EGMDelete(int robot_index);

/**
 * @brief 设置修正数据(用于控制器内使用),需要按照总线周期进行调用
 *
 * @param position 修正数据
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int SetEGMInput(double* position, int robot_index);

/**
 * @brief 获取引导基准(用于控制器内使用),调用EGMGuideMove()时机器人所处的关节位置或笛卡尔位置
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern double* GetEGMGuideBase(int robot_index);

/**
 * @brief 创建外部修正数据区(用于控制器外通讯使用)
 *
 * @param input_type 1:启用外部关节输入 2:启用外部工件坐标系笛卡尔输入 3:启用外部工具坐标系笛卡尔输入
 * @param cycle_times 修正周期倍数（为总线周期的倍数）
 * @param rtool 工具
 * @param rwobj 工件
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmCreate(int input_type,int cycle_times,tool* rtool, wobj* rwobj,int robot_index);

/**
 * @brief 设置修正数据限制(用于控制器外通讯使用)
 *
 * @param velocity_max_per 最大速度百分比(0~1)
 * @param acceleration_max_per 最大加速度百分比(0~1)
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetLimit(double velocity_max_per, double acceleration_max_per,int robot_index);

/**
 * @brief 设置使用的传感器名称(用于控制器外通讯使用)
 *
 * @param sensor_name 传感器名称(为NULL表示不采集力传感器数据)
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetForceSensor(const char* sensor_name, int robot_index);

/**
 * @brief 设置使用的夹爪名称(用于控制器外通讯使用)
 *
 * @param grip_name 夹爪名称(为NULL表示不使用夹爪)
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetGrip(const char* grip_name, int robot_index);

/**
 * @brief 设置虚拟墙刚度(用于控制器外通讯使用)
 *
 * @param stiffness 刚度系数（单位：N/m）
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetVirtualStiffness(double stiffness,int robot_index);

/**
 * @brief 设置数据数据滤波截止频率(用于控制器外通讯使用)
 *
 * @param filter_freq_hz 0:不滤波; >0:滤波截止频率(hz)
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmSetInputFilterFrequency(int filter_freq_hz,int robot_index);

/**
 * @brief 开始修正(用于控制器外通讯使用)
 *
 * @param ip 引导程序udp服务器ip
 * @param port 引导程序udp服务器端口号
 * @param guide_type 修正方式,0:相对; 1:绝对
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmStart(const char* ip, int port,int guide_type, int robot_index);

/**
 * @brief 启用外部引导(用于控制器外通讯使用),将禁止执行运动指令
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmGuideMove(int robot_index);

/**
 * @brief 停止修正(用于控制器外通讯使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmStop(int robot_index);

/**
 * @brief 释放外部修正数据区(用于控制器外通讯使用)
 *
 * @param robot_index 机器人索引(0,1,2...)
 * @return 0:成功;其他:失败
 */
extern int EgmDelete(int robot_index);



#ifdef __cplusplus
}
}
#endif


#endif /* EGM_INTERFACE_H_ */
