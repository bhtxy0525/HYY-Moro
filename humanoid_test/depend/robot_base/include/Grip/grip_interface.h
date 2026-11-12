/**
 * @file grip_interface.h
 *
 * @brief  夹爪创建、释放、操作
 * @author hanbing
 * @version 11.0.0
 * @date 2020-8-6
 *
 */
#ifndef GRIP_INTERFACE_H_
#define GRIP_INTERFACE_H_


#ifdef __cplusplus

namespace HYYRobotBase
{
extern "C" {
#endif

/**
 * @brief 夹爪位置控制
 *
 * @param grip_name 操作夹爪名字
 * @param position 夹爪夹紧程度，从0.0~1.0之间取值，0.0完全伸展开，1.0处于夹紧状态；
 * @return int 成功返回0，失败返回其他
 */
extern int SetGripPosition(const char* grip_name, double position);

/**
 * @brief 夹爪力控制
 *
 * @param grip_name 操作夹爪名字
 * @param force 夹爪夹持力，从0.0~1.0之间取值，0.0:零力，1.0:最大力；
 * @return int 成功返回0，失败返回其他
 */
extern int SetGripForce(const char* grip_name,double force);

/**
 * @brief 获取夹爪位置控制
 *
 * @param grip_name 操作夹爪名字
 * @param position 返回夹具夹紧程度，从0.0~1.0之间取值，0.0完全伸展开，1.0处于夹紧状态；
 * @return int 成功返回0，失败返回其他
 */
extern int GetGripPosition(const char* grip_name, double* position);

/**
 * @brief 获取夹爪力控制
 *
 * @param grip_name 操作夹爪名字
 * @param force 返回夹具加持力，从0.0~1.0之间取值，0.0:零力，1.0:最大力；
 * @return int 成功返回0，失败返回其他
 */
extern int GetGripForce(const char* grip_name,double* force);


#ifdef __cplusplus
}
}

#endif

#endif /* GRIP_INTERFACE_H_ */
