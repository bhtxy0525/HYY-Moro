/**
 * @file robot_teach_interface_plus.h
 *
 * @brief  机器人示教接口
 * @author hanbing
 * @version 13.0.0
 * @date 2026-05-19
 *
 */

#ifndef ROBOT_TEACH_INTERFACE_PLUS_H_
#define ROBOT_TEACH_INTERFACE_PLUS_H_

#include "Base/RobotStruct.h"

#ifdef __cplusplus

#include <string>
#include <vector>
#include <utility>
#include <map>

namespace HYYRobotBase
{

/**
* @brief 运行脚本
*
* @param filename 脚本(含后缀.cpp,.c,.lua,.py)
* @return int 成功返回0，错误返回其他
*/
extern int RunScript(std::string filename);

/**
* @brief 停止运行
*
* @return int 成功返回0，错误返回其他
*/
extern int StopScript();

/**
* @brief 得到脚本状态
*
* @return int 脚本的执行状态(0:未启动;1:正在编译;2:编译失败终止;3:正在运行执行脚本;4:运行正常结束;5:运行异常结束)
*/
extern int GetScriptState();

/**
* @brief 得到脚本实时消息
*
* @return 消息的队列
*/
extern std::vector<std::string> GetScriptMessages();

/**
* @brief 设置脚本文本操作路径
*
* @param base_dir 所在路径
*/
extern void SetScriptPath(const std::string& base_dir);

/**
* @brief 获得脚本文件列表
*
* @return 文件列表(用json存放)
*/
extern std::string GetScriptList();

/**
* @brief 读取指定的脚本内容
* @param filename 指定的脚本名称(含后缀)
* @return 执行结果代码+文件内容
*/
extern std::pair<int, std::string> ReadScript(const std::string& filename);

/**
* @brief 新建/保存脚本(文件不存为新建,否则为保存)
* @param filename 指定的脚本名称(含后缀)
* @param content 文件内容(content为空且为新件时，会为新建文件添加默认内容)
* @return 执行结果代码+操作结果描述
*/
extern std::pair<int, std::string> SaveScript(const std::string& filename, const std::string& content);

/**
* @brief 删除指定的脚本
* @param filename 指定的脚本名称(含后缀)
* @return 执行结果代码+操作结果描述容
*/
extern std::pair<int, std::string> DeleteScript(const std::string& filename);

/**
* @brief 获取选中机器人动力学辨识轨迹参数
*
* @return std::map<std::string,std::vector<double>> 返回选中机器人的轨迹参数
*/
extern std::map<std::string,std::vector<double>>  GetDynTrajectoryParamS();

/**
* @brief 获取选中机器人动力学负载辨识轨迹参数
*
* @return std::map<std::string,std::vector<double>> 返回选中机器人的轨迹参数
*/
extern std::map<std::string,std::vector<double>>  GetPayloadDynTrajectoryParamS();

/**
* @brief 设置选中机器人动力学辨识轨迹参数
* @param subdata 机器人的轨迹参数
* @return bool true:成功，false:失败
*/
extern bool SetDynTrajectoryParamS(const std::map<std::string,std::vector<double>>& subdata);

/**
* @brief 设置选中机器人动力学负载辨识轨迹参数
* @param subdata 机器人的轨迹参数
* @return bool true:成功，false:失败
*/
extern bool SetPayloadDynTrajectoryParamS(const std::map<std::string,std::vector<double>>& subdata);


/**
* @brief 获取配置文件中索六维力传感器名称
* @return std::vector<std::string> 名称向量
*/
extern std::vector<std::string> GetForceSensorList();

/**
* @brief 将六维力传感器绑定到机器人
* @param sensor_name 传感器名称
* @param robot_index 机器人索引
* @return int 0:其他:失败
*/
extern int SetForceSensorBindRobotIndex(std::string& sensor_name,int robot_index);

/**
* @brief 获取六维力传感器绑定的机器人
* @param sensor_name 传感器名称
* @return int 机器人索引
*/
extern int GetForceSensorBindRobotIndex(std::string& sensor_name);

/**
* @brief 获取绑定机器人的六维力传感器名称
* @param index 机器人索引
* @return std::string 传感器名称
*/
extern std::string GetForceSensorBindSensorName(int index);

/**
* @brief 删除六维力传感器绑定的机器人
* @param sensor_name 传感器名称
* @return int 0:其他:失败
*/
extern int DeleteForceSensorBindRobotIndex(std::string& sensor_name);

/**
* @brief 获取配置文件中夹爪名称
* @return std::vector<std::string> 名称向量
*/
extern std::vector<std::string> GetGripList();

/**
* @brief 获取所有设备的最大与最小值
* @return std::tuple<std::vector<std::vector<double>>,std::vector<std::vector<double>>,std::vector<std::vector<double>>,std::vector<std::vector<double>>> 所有机器人的最大值，所有机器人的最小值，所有外部轴组的最大值，所有外部轴组的最小值
*/
extern std::tuple<std::vector<std::vector<double>>,std::vector<std::vector<double>>,std::vector<std::vector<double>>,std::vector<std::vector<double>>> GetDeviceJointLimit();


/**
* @brief 获取所有设备的关节类型(0:转动;1:滑动)
* @return std::tuple<std::vector<std::vector<int>>,std::vector<std::vector<int>>> 所有机器人的关节类型，所有外部轴组的关节类型
*/
extern std::tuple<std::vector<std::vector<int>>,std::vector<std::vector<int>>> GetDeviceJointType();

}
#endif



#endif /* ROBOT_TEACH_INTERFACE_PLUS_H_ */
