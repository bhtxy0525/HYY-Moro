/*
 * @File name:
 * @Author: tianers
 * @Version: V1.0
 * @Date: 2026-10-27 09:50:02
 * @Description:
 * tianers All rights reserved.
 */
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <iterator>
#include <limits>
#include "VisionProReceiver.h"
#include "HYYRobotInterface.h"
#include "WhrTeachInterfacePublic.h"

namespace
{
    WhrModel robot;
    WhrTeachInterface teach_robot(robot);
}
using namespace HYYRobotBase;
namespace
{
    int CustomMoveA(robjoint *rjoint0, speed *rspeed0, robjoint *rjoint1, speed *rspeed1, robjoint *rjoint2, speed *rspeed2)
    {
        setMoveThread(1);
        MultiMoveA(rjoint0, rspeed0, NULL3, 0);
        MultiMoveA(rjoint1, rspeed1, NULL3, 1);
        MultiMoveA(rjoint2, rspeed2, NULL3, 2);
        MoveSyncStart();
        wait_move_finish(0);
        wait_move_finish(1);
        wait_move_finish(2);
        setMoveThread(0);
        return 0;
    }
}

int TeleTest()
{
    VisionProReceiver receiver("tcp://127.0.0.1:5557");

    if (receiver.Start() != 0)
    {
        std::cerr << "Failed to start Vision Pro receiver." << std::endl;
        return 1;
    }

    std::cout << "Waiting for Vision Pro data..." << std::endl;

    uint64_t last_sequence = 0;
    bool last_connected = false;
    bool ever_connected = false;

    while (true)
    {
        VisionProData data;

        if (receiver.GetLatestData(data))
        {
            if (data.connected)
                ever_connected = true;
            else if (ever_connected)
            {
                std::cout << "AVP disconnected. Ending receiver test."
                          << std::endl;
                break;
            }

            // 新帧或连接状态变化时打印，确保Python停止后也能显示断流
            if (data.sequence != last_sequence ||
                data.connected != last_connected)
            {
                last_connected = data.connected;

                last_sequence =
                    data.sequence;

                std::cout
                    << "\n======================================"
                    << std::endl;

                std::cout
                    << "Sequence       : "
                    << data.sequence
                    << std::endl;

                std::cout
                    << "Connected      : "
                    << std::boolalpha
                    << data.connected
                    << std::endl;

                std::cout
                    << "Left pinch     : "
                    << data.left_pinch
                    << std::endl;

                std::cout
                    << "Right pinch    : "
                    << data.right_pinch
                    << std::endl;

                std::cout
                    << "\nLeft wrist:\n"
                    << data.left_wrist
                    << std::endl;

                std::cout
                    << "\nRight wrist:\n"
                    << data.right_wrist
                    << std::endl;
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));
    }

    receiver.Stop();

    return 0;
}

// AVP平移映射：机器人X=AVP Y，机器人Y=-AVP X，机器人Z=AVP Z。
// 左右手使用同一轴映射；position_scale仅缩放平移。
int HumanoidTeleoperationDemo(double position_scale = 1.0)
{
    Eigen::Matrix3d C;
    C << 0, 1, 0,
        -1, 0, 0,
        0, 0, 1;

    if (!std::isfinite(position_scale) || position_scale <= 0.0)
    {
        std::cerr << "Invalid position scale." << std::endl;
        return -1;
    }

    VisionProReceiver receiver("tcp://127.0.0.1:5557");
    if (receiver.Start() != 0)
        return -1;

    // 等待有效追踪；receiver析构时自动Stop。
    std::cout << "Waiting for connected AVP tracking..." << std::endl;
    while (robot_ok())
    {
        VisionProData data;
        if (receiver.GetLatestData(data) && data.connected)
            break;
        Rsleep(10);
    }
    if (!robot_ok())
        return -1;

    // 沿用HumanoidServoDemo的初始化和上电流程。
    ClearDeviceError();

    IMPORTJOINT(R0_PTEST);
    IMPORTJOINT(R1_PTEST);
    IMPORTJOINT(R2_PTEST);
    IMPORTSPEED(v1);

    double dt = 0.0;

    int ret = 0;

    // ========================================================
    // 上电
    // ========================================================
    while (robot_ok())
    {
        ret = RobotPower(0); // 左臂

        if (!ret)
        {
            break;
        }
        else
        {
            clear_robot_move_error(0);
            clear_robot_driver_error(0);
        }

        Rsleep(1000);
    }

    while (robot_ok())
    {
        ret = RobotPower(1); // 右臂

        if (!ret)
        {
            break;
        }
        else
        {
            clear_robot_move_error(1);
            clear_robot_driver_error(1);
        }

        Rsleep(1000);
    }

    while (robot_ok())
    {
        ret = RobotPower(2); // 躯干

        if (!ret)
        {
            break;
        }
        else
        {
            clear_robot_move_error(2);
            clear_robot_driver_error(2);
        }

        Rsleep(1000);
    }

    if (!robot_ok())
        return -1;

    std::cout << "Robot enable success..." << std::endl;

    // ========================================================
    // 运动到初始位置
    // ========================================================
    ret = CustomMoveA(R0_PTEST, v1, R1_PTEST, v1, R2_PTEST, v1);

    if (ret != 0)
    {
        std::cerr << "CustomMoveA failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    // ========================================================
    // 获取当前实际关节状态
    // ========================================================
    if (teach_robot.GetRobotState() != 0)
    {
        std::cerr << "GetRobotState failed." << std::endl;
        return -1;
    }

    // ========================================================
    // 获取当前左右末端位姿
    // ========================================================
    Eigen::Matrix4d TL_Init = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d TR_Init = Eigen::Matrix4d::Identity();

    robot.fkineTL(teach_robot.robot_state.q_torso_left.data(), TL_Init);
    robot.fkineTR(teach_robot.robot_state.q_torso_right.data(), TR_Init);

    // 目标位姿，每个周期在初始位姿基础上更新
    Eigen::Matrix4d TL_Target = TL_Init;
    Eigen::Matrix4d TR_Target = TR_Init;

    // ========================================================
    // 10 ms 更新周期
    // ========================================================
    auto timer = CreateUserTimer();

    // 用与TeleRecord相同的接口读取最后关节目标，避免用反馈误差重设目标。
    const char *left_arm = get_name_robot_device(get_deviceName(0, nullptr), 0);
    const char *right_arm = get_name_robot_device(get_deviceName(0, nullptr), 1);
    const char *torso_arm = get_name_robot_device(get_deviceName(0, nullptr), 2);
    if (!left_arm || !right_arm || !torso_arm)
    {
        std::cerr << "Get robot group name failed." << std::endl;
        return -2;
    }
    Eigen::Matrix<double, 17, 1> q_hold = teach_robot.robot_state.q_upper;
    if (!q_hold.allFinite())
        return -2;

    // ========================================================
    // 开启三组关节伺服
    // ========================================================
    ret = ServoStart(1e-7, 0.5, 0);
    if (ret != 0)
    {
        std::cerr << "ServoStart left arm failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    ret = ServoStart(1e-7, 0.5, 1);
    if (ret != 0)
    {
        ServoEnd(0);
        std::cerr << "ServoStart right arm failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    ret = ServoStart(1e-7, 0.5, 2);
    if (ret != 0)
    {
        ServoEnd(0);
        ServoEnd(1);
        std::cerr << "ServoStart torso failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    struct HandState
    {
        bool armed = false; // 启动后先观察到松开才允许控制
        bool active = false;
        Eigen::Matrix4d avp_start = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d target_start = Eigen::Matrix4d::Identity();
    };
    HandState left, right;

    // 仅使用现有左右捏合信号；必须完成三次双手捏合和松开。
    struct HomeGesture
    {
        enum class Phase
        {
            NeedOpen,
            WaitPinch,
            WaitRelease,
            ConfirmOpen
        };
        Phase phase = Phase::NeedOpen;
        int count = 0;
        bool counting = false;
        double start = 0.0;
        double closed_since = 0.0;
        double open_since = 0.0;
        double mixed_since = -1.0;

        void reset()
        {
            phase = Phase::NeedOpen;
            count = 0;
            counting = false;
            mixed_since = -1.0;
        }

        bool update(bool lp, bool rp, double now)
        {
            constexpr double window = 2.0;    // 三次动作的总时间
            constexpr double pair_gap = 0.25; // 两手允许的不同步时间
            constexpr double min_hold = 0.08; // 双手捏合/松开的最短确认时间
            const bool closed = lp && rp;
            const bool opened = !lp && !rp;
            if (counting && now - start > window)
                reset();
            if (phase == Phase::NeedOpen)
            {
                if (opened)
                    phase = Phase::WaitPinch;
                return false;
            }
            if (phase == Phase::WaitPinch)
            {
                if (opened)
                    mixed_since = -1.0;
                else if (closed)
                {
                    if (mixed_since >= 0.0 && now - mixed_since > pair_gap)
                    {
                        reset();
                        return false;
                    }
                    if (!counting)
                    {
                        start = mixed_since >= 0.0 ? mixed_since : now;
                        counting = true;
                    }
                    closed_since = now;
                    mixed_since = -1.0;
                    phase = Phase::WaitRelease;
                }
                else
                {
                    if (mixed_since < 0.0)
                        mixed_since = now;
                    if (now - mixed_since > pair_gap)
                        reset();
                }
                return false;
            }
            if (phase == Phase::WaitRelease)
            {
                if (opened)
                {
                    if (now - closed_since < min_hold ||
                        (mixed_since >= 0.0 && now - mixed_since > pair_gap))
                        reset();
                    else
                    {
                        open_since = now;
                        phase = Phase::ConfirmOpen;
                    }
                }
                else if (closed)
                    mixed_since = -1.0;
                else
                {
                    if (mixed_since < 0.0)
                        mixed_since = now;
                    if (now - mixed_since > pair_gap)
                        reset();
                }
                return false;
            }
            if (!opened)
            {
                reset();
                return false;
            }
            if (now - open_since >= min_hold)
            {
                if (++count == 3)
                {
                    reset();
                    return true;
                }
                phase = Phase::WaitPinch;
                mixed_since = -1.0;
            }
            return false;
        }
    };
    HomeGesture home_gesture;

    auto update_target = [&](bool pinch, const Eigen::Matrix4d &wrist,
                             HandState &hand, Eigen::Matrix4d &target)
    {
        if (!pinch)
        {
            hand.armed = true;
            hand.active = false;
            return; // 保持上一次下发的目标
        }
        if (!hand.armed)
            return;
        if (!hand.active)
        {
            hand.avp_start = wrist;
            hand.target_start = target;
            hand.active = true;
            return; // 捏合瞬间无跳变
        }
        target.block<3, 1>(0, 3) = hand.target_start.block<3, 1>(0, 3) +
                                   position_scale * C *
                                       (wrist.block<3, 1>(0, 3) - hand.avp_start.block<3, 1>(0, 3));

        // 按同一轴映射转换旋转增量，再左乘机器人起始姿态。
        const Eigen::Matrix3d delta_R = wrist.block<3, 3>(0, 0) *
                                        hand.avp_start.block<3, 3>(0, 0).transpose();
        target.block<3, 3>(0, 0) = C * delta_R *
                                   C.transpose() * hand.target_start.block<3, 3>(0, 0);
    };

    constexpr double control_period = 0.01;
    const double posture_scale = 1.0;
    bool was_active = false;

    // 松开双手后的指令减速时间（秒），不是ServoStart参数。
    constexpr double release_brake_time = 0.20;
    using UpperJoints = Eigen::Matrix<double, 17, 1>;
    std::array<UpperJoints, 5> command_history;
    int history_count = 1;
    command_history[0] = q_hold;
    bool braking = false;
    double brake_elapsed = 0.0;
    UpperJoints brake_start = q_hold;
    UpperJoints brake_velocity = UpperJoints::Zero();

    const UpperJoints q_home = q_hold; // 本次初始化完成后的17个实际关节角
    UpperJoints home_start = q_hold;
    bool home_requested = false;
    bool returning_home = false;
    bool require_both_release = false;
    double home_elapsed = 0.0;
    double home_duration = 3.0;
    double home_settled_time = 0.0;
    double home_wait_time = 0.0;
    constexpr double home_min_time = 3.0;
    constexpr double home_max_speed = 0.20; // 关节指令速度上限 rad/s
    constexpr double home_max_accel = 0.40; // 关节指令加速度上限 rad/s²

    auto record_command = [&]()
    {
        if (history_count < 5)
            command_history[history_count++] = q_hold;
        else
        {
            for (int i = 0; i < 4; ++i)
                command_history[i] = command_history[i + 1];
            command_history[4] = q_hold;
        }
    };

    auto read_last_joint_target = [&]() -> bool
    {
        Eigen::Matrix<double, 17, 1> next;
        next.setConstant(std::numeric_limits<double>::quiet_NaN());
        GetGroupLastTargetPosition(torso_arm, next.data());
        GetGroupLastTargetPosition(left_arm, next.data() + 3);
        GetGroupLastTargetPosition(right_arm, next.data() + 10);
        if (!next.allFinite())
            return false;
        q_hold = next;
        return true;
    };

    // 从保持关节目标恢复末端基准；重新捏合时不沿用旧笛卡尔误差。
    auto reset_target_to_hold = [&]()
    {
        Eigen::Matrix<double, 10, 1> q_left, q_right;
        q_left.head<3>() = q_hold.head<3>();
        q_left.tail<7>() = q_hold.segment<7>(3);
        q_right.head<3>() = q_hold.head<3>();
        q_right.tail<7>() = q_hold.segment<7>(10);
        robot.fkineTL(q_left.data(), TL_Target);
        robot.fkineTR(q_right.data(), TR_Target);
    };

    auto send_joint_hold = [&](double command_time) -> int
    {
        robjoint R0_hold, R1_hold, R2_hold;
        init_robjoint(&R0_hold, q_hold.data() + 3, 7);
        init_robjoint(&R1_hold, q_hold.data() + 10, 7);
        init_robjoint(&R2_hold, q_hold.data(), 3);
        int result = ServoJoint(&R0_hold, command_time, 0);
        if (result != 0)
            return result;
        result = ServoJoint(&R1_hold, command_time, 1);
        if (result != 0)
            return result;
        return ServoJoint(&R2_hold, command_time, 2);
    };

    std::cout << "Release both pinches before teleoperation." << std::endl;
    while (robot_ok())
    {
        timer->WaitTimes(10);
        dt += control_period;

        VisionProData data;
        if (receiver.GetLatestData(data) && data.connected)
        {
            const double gesture_time = std::chrono::duration<double>(
                                            std::chrono::steady_clock::now().time_since_epoch())
                                            .count();
            if (!home_requested && !returning_home && !require_both_release &&
                home_gesture.update(data.left_pinch, data.right_pinch, gesture_time))
            {
                home_requested = true;
                left.active = right.active = false;
                std::cout << "Home gesture detected. Stopping, then returning home."
                          << std::endl;
            }

            if (require_both_release && !data.left_pinch && !data.right_pinch)
            {
                require_both_release = false;
                left.armed = right.armed = true;
                home_gesture.reset();
            }

            // 回位和减速期间不建立捏合起点；结束后用最新帧重新开始。
            if (!braking && !returning_home && !home_requested && !require_both_release)
            {
                update_target(data.left_pinch, data.left_wrist, left, TL_Target);
                update_target(data.right_pinch, data.right_wrist, right, TR_Target);
            }
        }
        else
        {
            // Python退出、AVP断流或接收端超时：结束本次遥操作。
            std::cerr << "AVP disconnected. Stopping teleoperation."
                      << std::endl;
            break; // 不再下发目标，转到循环后的ServoEnd(0/1/2)
        }

        const bool any_active = left.active || right.active;
        if (was_active && !any_active)
        {
            // 仅“至少一手控制 -> 双手松开”触发，不影响单手继续操作。
            brake_start = q_hold;
            brake_velocity.setZero();
            if (history_count >= 2)
            {
                // 按命令时间轴估计最近40ms内的平均关节目标速度（rad/s）。
                // 这是指令速度估计，不是实际关节速度反馈。
                brake_velocity =
                    (command_history[history_count - 1] - command_history[0]) /
                    ((history_count - 1) * control_period);
            }
            if (!brake_velocity.allFinite())
            {
                std::cerr << "Invalid release velocity." << std::endl;
                ret = -2;
                break;
            }
            brake_elapsed = 0.0;
            braking = true;
        }

        // 必须等原减速轨迹结束，才能开始零起止速度的回位轨迹。
        if (home_requested && !braking && !returning_home)
        {
            home_start = q_hold;
            home_elapsed = 0.0;
            home_settled_time = 0.0;
            home_wait_time = 0.0;
            const double max_distance = (q_home - home_start).cwiseAbs().maxCoeff();
            // 五次时间曲线最大速度系数1.875，最大加速度系数10/sqrt(3)。
            home_duration = std::max(home_min_time,
                                     std::max(1.875 * max_distance / home_max_speed,
                                              std::sqrt((10.0 / std::sqrt(3.0)) * max_distance /
                                                        home_max_accel)));
            home_requested = false;
            returning_home = true;
            left.active = right.active = false;
        }

        if (returning_home)
        {
            home_elapsed = std::min(home_duration, home_elapsed + control_period);
            const double u = home_elapsed / home_duration;
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double blend = u3 * (10.0 - 15.0 * u + 6.0 * u2);
            q_hold = home_start + blend * (q_home - home_start);
            if (home_elapsed >= home_duration)
                q_hold = q_home;
            ret = send_joint_hold(dt);
            if (ret == 0 && home_elapsed >= home_duration)
            {
                // 等实际反馈到位并稳定后才允许重新遥操作，不能只看下发成功。
                home_wait_time += control_period;
                if (teach_robot.GetRobotState() != 0 ||
                    !teach_robot.robot_state.q_upper.allFinite())
                    ret = -2;
                else
                {
                    const double error =
                        (teach_robot.robot_state.q_upper - q_home).cwiseAbs().maxCoeff();
                    home_settled_time = error <= 0.01
                                            ? home_settled_time + control_period
                                            : 0.0;
                    if (home_settled_time >= 0.20)
                    {
                        returning_home = false;
                        require_both_release = true;
                        left = HandState{};
                        right = HandState{};
                        reset_target_to_hold();
                        history_count = 1;
                        command_history[0] = q_hold;
                        home_gesture.reset();
                        std::cout << "Home reached. Release both hands, then pinch to resume."
                                  << std::endl;
                    }
                    else if (home_wait_time >= 5.0)
                    {
                        std::cerr << "Home tracking did not settle within 5 seconds."
                                  << std::endl;
                        ret = -2;
                    }
                }
            }
        }
        else if (braking)
        {
            brake_elapsed = std::min(release_brake_time,
                                     brake_elapsed + control_period);
            const double u = brake_elapsed / release_brake_time;
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double u4 = u3 * u;

            // v(u)=v0*(1-3u²+2u³)，从估计速度平滑降到0。
            // 积分得到位置；起止加速度为0，最终续行量为v0*T/2。
            // 不对位置逐关节硬裁剪，否则会重新引入突变。
            q_hold = brake_start + brake_velocity * release_brake_time *
                                       (u - u3 + 0.5 * u4);
            ret = send_joint_hold(dt);
            if (ret == 0 && brake_elapsed >= release_brake_time)
            {
                braking = false;
                reset_target_to_hold();
                history_count = 1;
                command_history[0] = q_hold;
                left.active = right.active = false;
            }
        }
        else if (any_active)
        {
            // 单手、双手均使用原全身联合逆解，回正系数固定为1。
            ret = teach_robot.UpperServoCartesian(
                TL_Target, TR_Target, 30.0, dt, posture_scale);
            if (ret == 0)
            {
                if (!read_last_joint_target())
                {
                    std::cerr << "Read last joint target failed." << std::endl;
                    ret = -2;
                }
                else
                    record_command();
            }
        }
        else
        {
            // 已停稳：保持同一关节目标，不求逆解。
            ret = send_joint_hold(dt);
            history_count = 1;
            command_history[0] = q_hold;
        }

        if (ret != 0)
        {
            std::cerr << "Teleoperation/hold failed, ret = " << ret << std::endl;
            break;
        }
        was_active = any_active;
    }

    // ========================================================
    // 关闭伺服
    // ========================================================
    ServoEnd(0);
    ServoEnd(1);
    ServoEnd(2);

    receiver.Stop();

    std::cout << "UpperServo teleoperation finish..." << std::endl;

    return ret;
}
int TeleRecord()
{
    const char *left_arm = get_name_robot_device(get_deviceName(0, nullptr), 0);
    const char *right_arm = get_name_robot_device(get_deviceName(0, nullptr), 1);
    const char *torso_arm = get_name_robot_device(get_deviceName(0, nullptr), 2);
    double pos_l[7], pos_r[7], pos_t[3], record[17];
    auto timer = CreateUserTimer();
    while (robot_ok())
    {
        timer->Wait();
        GetGroupLastTargetPosition(left_arm, pos_l);
        GetGroupLastTargetPosition(right_arm, pos_r);
        GetGroupLastTargetPosition(torso_arm, pos_t);
        std::copy(std::begin(pos_l), std::end(pos_l), std::begin(record));
        std::copy(std::begin(pos_r), std::end(pos_r), std::begin(record) + 7);
        std::copy(std::begin(pos_t), std::end(pos_t), std::begin(record) + 14);
        RSaveDataFast1("Tele_test", 1, 100, 17, record);
    }
    return 0;
}