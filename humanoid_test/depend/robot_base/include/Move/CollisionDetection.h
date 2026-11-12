/**
 * @file CollisionDetection.h
 * 
 * @brief  碰撞检测
 * @author HanBing
 * @version 13.1.0
 * @date 2026-06-13
 * 
 */

#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_
#include "Base/RobotStruct.h"


#ifdef __cplusplus
namespace HYYRobotBase
{
extern "C" {
#endif

/**
* @brief 设置负载
*
* @param pl 机器人负载
* @param robot_index 机器人索引
* @return int 成功返回0，失败返回其他
*/
extern int SetCollisionDetectionPayLoad(PayLoad* pl,int robot_index);

/**
* @brief 设置碰撞等级
*
* @param robot_index 机器人索引
* @param level 碰撞等级(0,1,2,3,4,5,6)，0：不检测碰撞，正数：增大数值，提高灵敏度
* @return int 成功返回0，失败返回其他
*/
extern int SetCollisionDetectionLevel(int robot_index, int level);

/**
* @brief 获取碰撞等级
*
* @param robot_index 机器人索引
* @return int 碰撞等级
*/
extern int GetCollisionDetectionLevel(int robot_index);

/**
* @brief 设置碰撞回调函数
*
* @param f 回调函数
* @return int 成功返回0，失败返回其他
*/
extern int SetCollisionDetectionCallback(void (*f)(int));

/**
* @brief 开启碰撞检测
*
* @param cycle_times 检测时长(总线周期的倍数)
* @return int 成功返回0，失败返回其他
*/
extern int OpenCollisionDetection(int cycle_times);

/**
* @brief 关闭碰撞检测
*
* @return int 成功返回0，失败返回其他
*/
extern int CloseCollisionDetection();


#ifdef __cplusplus
}
}
#endif

#endif /* COLLISIONDETECTION_H_ */