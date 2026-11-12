/**
 * @file GetComputingTimePlus.h
 *
 * @brief  耗时计算接口
 * @author hanbing
 * @version 13.1.0
 * @date 2026-06-13
 *
 */
#ifndef GETCOMPUTINGTIMEPLUS_H_
#define GETCOMPUTINGTIMEPLUS_H_


#ifdef __cplusplus

#include <chrono>
#include <utility>
namespace HYYRobotBase {
    
/**
 * @brief 测量可调用对象的单次执行耗时（微秒级精度）
 * @details 执行传入的可调用对象并统计其完整执行时长，采用系统高精度时钟计时，
 *          返回以微秒为单位的浮点型耗时结果，支持函数、Lambda 表达式、仿函数等任意可调用实体。
 * @tparam Func 可调用对象的类型，支持任意返回值与参数签名的可调用实体
 * @param[in] func 待测量的可调用对象，通过完美转发支持左值与右值传入
 * @return double 可调用对象单次执行的耗时，单位为微秒（μs）
 * @note 计时结果包含函数调用的固有开销，对纳秒级极短函数的测量存在误差，建议多次采样取均值
 * @note 该函数会实际执行一次传入的 func，若 func 包含修改全局状态、IO 等副作用，需谨慎调用
 * @note 若 func 执行过程中抛出异常，异常将按原类型向上抛出，此时无有效计时结果
 */
template<typename Func>
inline double GetComputingTimeUS(Func&& func)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    // 执行传入的代码/函数
    std::forward<Func>(func)();
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::micro>(end - start);
    return duration.count();
}

}

#endif


#endif /*GETCOMPUTINGTIMEPLUS_H_*/