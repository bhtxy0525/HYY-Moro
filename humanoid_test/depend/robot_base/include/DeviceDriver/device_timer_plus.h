/**
 * @file device_timer_plus.h
 *
 * @brief 用户定时器接口定义
 * @author hanbing
 * @version 13.1.0
 * @date 2026-06-13
 */

#ifndef DEVICE_TIMER_PLUS_H_
#define DEVICE_TIMER_PLUS_H_


#ifdef __cplusplus

#include <memory>
namespace HYYRobotBase {

/**
 * @class UserTimer
 * @brief 用户定时器抽象接口类
 * @details 定义阻塞式定时器的通用操作规范，用于等待定时器周期触发（Tick）事件。
 *          由定时器管理后台统一触发 Tick 信号，用户线程通过 Wait/WaitTimes 接口阻塞等待。
 *          该类为抽象基类，不可直接实例化，需通过 CreateUserTimer() 工厂函数创建具体实例。
 */
class __attribute__((visibility("default"))) UserTimer {
public:
    /**
     * @brief 虚析构函数
     * @details 保证通过基类指针/智能指针销毁派生类对象时，派生类析构逻辑可被正确执行，
     *          避免资源泄漏与未定义行为，是多态基类的标准写法。
     */
    virtual ~UserTimer() = default;

    /**
     * @brief 阻塞等待下一次 Tick 触发
     * @details 调用线程将进入阻塞休眠状态，直到以下任一条件满足：
     *          1. 定时器触发一次 Tick 事件，函数正常返回；
     *          2. 定时器被主动停止，函数提前返回。
     * @retval true  定时器正常触发，线程被正常唤醒
     * @retval false 定时器已停止，等待提前终止
     */
    virtual bool Wait() = 0;

    /**
     * @brief 阻塞等待指定次数的 Tick 触发
     * @details 调用线程将进入阻塞休眠状态，直到累计触发指定次数的 Tick 事件，
     *          或定时器被主动停止。若等待过程中定时器停止，将立即返回。
     * @param[in] cycle_times 需要等待的 Tick 周期总次数，应为正整数
     * @retval true  已完成全部指定次数的 Tick 等待，正常唤醒
     * @retval false 定时器已停止，等待提前终止
     */
    virtual bool WaitTimes(int cycle_times) = 0;
};

/**
 * @brief 创建用户定时器实例
 * @details 工厂函数，构造并返回 UserTimer 接口的具体实现对象，通过 std::shared_ptr 自动管理生命周期。
 *          创建后的定时器默认处于激活状态，可立即调用 Wait/WaitTimes 等待 Tick 事件。
 * @return std::shared_ptr<UserTimer> 指向新建定时器实例的共享智能指针
 * @note 函数标记 [[nodiscard]]，返回值不可丢弃，必须接收以持有定时器的生命周期与访问权限
 */
[[nodiscard]] extern __attribute__((visibility("default"))) std::shared_ptr<UserTimer> CreateUserTimer();


}

#endif


#endif /*DEVICE_TIMER_PLUS_H_*/