/**
 * @file RobotSystemPlus.h
 *
 * @brief  机器人控制系统初始化必要接口
 * @author hanbing
 * @version 13.0.0
 * @date 2026-05-19
 *
 */

#ifndef ROBOTSYSTEMPLUS_H_
#define ROBOTSYSTEMPLUS_H_
/*---------------------------- Includes ------------------------------------*/
#include "Base/RobotStruct.h"
#include <stdarg.h>


#ifdef __cplusplus
namespace HYYRobotBase
{

/**
 * @brief 系统初始化
 * @param arg 命令行参数
 * @param custom_http 自定义http命令回调函数
 * @param custom_websocket 自定义websocket数据发布回调函数
 * @return true:成功，false:失败
 */
extern bool SystemInit(command_arg* arg,void (*custom_http)(void*)=nullptr,void (*custom_websocket)(void*)=nullptr);

}
#endif


#endif /* ROBOTSYSTEMPLUS_H_ */
