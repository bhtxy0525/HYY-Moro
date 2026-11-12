#include <iostream>
#include <iomanip>
#include "WhrTeachInterfacePublic.h"

namespace
{
    WhrModel robot;
    WhrTeachInterface teach_robot(robot);
}
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
static double gaussrand()
{
    /* 简单高斯噪声 (Box-Muller) */
    static int has = 0;
    static double v2;
    if (has)
    {
        has = 0;
        return v2;
    }
    double u1, u2, s = 0.0;
    do
    {
        u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
        u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
        u1 = 2 * u1 - 1;
        u2 = 2 * u2 - 1;
        s = u1 * u1 + u2 * u2;
    } while (s >= 1.0 || s == 0.0);
    double mult = sqrt(-2.0 * log(s) / s);
    v2 = u2 * mult;
    has = 1;
    return u1 * mult;
}
int kinematics_test()
{
    IMPORTSPEED(v1);
    Eigen::Matrix4d TL = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d TR = Eigen::Matrix4d::Identity();
    double q_left_current[10] = {0.5236, -0.5236, 0, 0, -0.8, 1.5708, 1.5708, 0, 0, 0};
    double q_right_current[10] = {0.5236, -0.5236, 0, 0, -0.8, -1.5708, 1.5708, 0.3, 0, 0};
    double q_torso_left_right[19] = {0, 0, 0.5236, -0.5236, 0, 0, -0.8, 1.5708, 1.5708, 0, 0, 0, 0, -0.8, -1.5708, 1.5708, 0.0, 0, 0};
    robot.fkineTL(q_left_current, TL);
    robot.fkineTR(q_right_current, TR);
    std::cout << "TL relative to torso is : \n"
              << TL << std::endl;
    std::cout << "TR relative to torso is : \n"
              << TR << std::endl;
    robot.fkine(q_torso_left_right, 0, TL);
    robot.fkine(q_torso_left_right, 1, TR);
    std::cout << "TL relative to base is : \n"
              << TL << std::endl;
    std::cout << "TR relative to base is : \n"
              << TR << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    TL(0, 3) = TL(0, 3) + 0.2;
    int ik_iter = 0;
    Eigen::Matrix<double, 17, 1> q_res = robot.inverseUpperlimb(q_torso_left_right, TL, TR, 1, ik_iter);
    std::cout << "ik_iter is : " << ik_iter << " q_res is : " << q_res.transpose() << std::endl;

    double q_check_left[10];
    memcpy(q_check_left, q_res.data(), 10 * sizeof(double));
    robot.fkineTL(q_check_left, TL);
    std::cout << "TL is : " << TL << std::endl;
    // teach_robot.UpperMoveJ(TL, TR, v1, 1);
    return 0;
}
int HumanoidMoveDemo()
{
    ClearDeviceError();

    IMPORTJOINT(R0_PTEST);
    IMPORTJOINT(R1_PTEST);
    IMPORTJOINT(R2_PTEST);
    IMPORTSPEED(v0);
    IMPORTSPEED(v1);

    RobotPower(0);
    RobotPower(1);
    RobotPower(2);

    CustomMoveA(R0_PTEST, v1, R1_PTEST, v1, R2_PTEST, v1);

    std::cout << "RobotPower sucess, ready to run WBC demo..." << std::endl;

    Eigen::Matrix4d TL_Desired = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d TR_Desired = Eigen::Matrix4d::Identity();

    int ret = 0;
    int count = 0;

    // ========================================================
    // UpperMoveL测试
    // ========================================================
    Rsleep(2000);
    // MoveJ结束后重新读取当前实际状态
    if (teach_robot.GetRobotState() != 0)
    {
        std::cerr << "GetRobotState failed." << std::endl;
        return -1;
    }

    robot.fkineTL(teach_robot.robot_state.q_torso_left.data(), TL_Desired);
    robot.fkineTR(teach_robot.robot_state.q_torso_right.data(), TR_Desired);

    // 左末端沿自身X轴移动0.3 m
    TL_Desired.block<3, 1>(0, 3) += TL_Desired.block<3, 3>(0, 0) * Eigen::Vector3d(0.3, 0.0, 0.0);

    // 左末端绕自身X轴旋转45°
    const double angle_x = M_PI / 4.0;
    const Eigen::Matrix3d Rx = Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX()).toRotationMatrix();

    TL_Desired.block<3, 3>(0, 0) = TL_Desired.block<3, 3>(0, 0) * Rx;

    ret = teach_robot.UpperMoveL(TL_Desired, TR_Desired, 5.0, 50.0);

    if (ret != 0)
    {
        std::cerr << "UpperMoveL failed, ret = " << ret << std::endl;
        return ret;
    }
    CustomMoveA(R0_PTEST, v1, R1_PTEST, v1, R2_PTEST, v1);

    // ========================================================
    // UpperMoveJ测试
    // ========================================================
    while (robot_ok())
    {
        if (count == 50)
            break;

        // 获取当前机器人状态
        if (teach_robot.GetRobotState() != 0)
        {
            std::cerr << "GetRobotState failed." << std::endl;
            return -1;
        }

        // 获取当前左右末端位姿
        robot.fkineTL(teach_robot.robot_state.q_torso_left.data(), TL_Desired);
        robot.fkineTR(teach_robot.robot_state.q_torso_right.data(), TR_Desired);

        // 左臂前后
        TL_Desired(0, 3) += 0.15;
        ret = teach_robot.UpperMoveJ(TL_Desired, TR_Desired, v1, 10);
        if (ret != 0)
            break;

        TL_Desired(0, 3) -= 0.15;
        ret = teach_robot.UpperMoveJ(TL_Desired, TR_Desired, v1, 10);
        if (ret != 0)
            break;

        // 右臂前后
        TR_Desired(0, 3) += 0.15;
        ret = teach_robot.UpperMoveJ(TL_Desired, TR_Desired, v1, 10);
        if (ret != 0)
            break;

        TR_Desired(0, 3) -= 0.15;
        ret = teach_robot.UpperMoveJ(TL_Desired, TR_Desired, v1, 10);
        if (ret != 0)
            break;

        // 双臂同时前后
        TL_Desired(0, 3) += 0.15;
        TR_Desired(0, 3) += 0.15;
        ret = teach_robot.UpperMoveJ(TL_Desired, TR_Desired, v1, 10);
        if (ret != 0)
            break;

        TL_Desired(0, 3) -= 0.15;
        TR_Desired(0, 3) -= 0.15;
        ret = teach_robot.UpperMoveJ(TL_Desired, TR_Desired, v1, 10);
        if (ret != 0)
            break;

        ++count;
    }

    if (ret != 0)
    {
        std::cerr << "UpperMoveJ failed, ret = " << ret << std::endl;
        return ret;
    }
    return 0;
}

int HumanoidServoDemo()
{
    // 测试上肢笛卡尔伺服跟随功能
    ClearDeviceError();

    IMPORTJOINT(R0_PTEST);
    IMPORTJOINT(R1_PTEST);
    IMPORTJOINT(R2_PTEST);
    IMPORTSPEED(v1);

    double t = 0.0;
    double A = 0.05; // 运动幅值 0.05 m
    double f = 0.1;  // 运动频率 0.1 Hz

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
    // 50 ms 更新周期
    // ========================================================
    RTimer timer;
    initUserTimer(&timer, 0, 50);

    // ========================================================
    // 开启三组关节伺服
    // ========================================================
    ret = ServoStart(1e-6, 0.2, 0);
    if (ret != 0)
    {
        std::cerr << "ServoStart left arm failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    ret = ServoStart(1e-6, 0.2, 1);
    if (ret != 0)
    {
        ServoEnd(0);
        std::cerr << "ServoStart right arm failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    ret = ServoStart(1e-6, 0.2, 2);
    if (ret != 0)
    {
        ServoEnd(0);
        ServoEnd(1);
        std::cerr << "ServoStart torso failed, ret = "
                  << ret << std::endl;
        return ret;
    }

    // ========================================================
    // Servo测试
    // 前10 s左臂运动，后10 s右臂运动
    // ========================================================
    while (robot_ok())
    {
        userTimer(&timer);

        t += 0.05;

        if (t > 20.0)
        {
            break;
        }

        // 每周期都从固定初始位姿重新生成目标
        TL_Target = TL_Init;
        TR_Target = TR_Init;

        if (t <= 10.0)
        {
            // 左末端沿基坐标系X方向周期运动
            TL_Target(0, 3) = TL_Init(0, 3) + A * cos(2.0 * M_PI * f * t) - A + gaussrand() * 0.003;
        }
        else
        {
            // 右末端沿基坐标系X方向周期运动
            TR_Target(0, 3) = TR_Init(0, 3) + A * cos(2.0 * M_PI * f * (t - 10.0)) - A + gaussrand() * 0.003;
        }

        ret = teach_robot.UpperServoCartesian(TL_Target, TR_Target, 10.0, t);

        if (ret != 0)
        {
            std::cerr << "UpperServoCartesian failed, ret = "
                      << ret << std::endl;
            break;
        }
    }

    // ========================================================
    // 关闭伺服
    // ========================================================
    ServoEnd(0);
    ServoEnd(1);
    ServoEnd(2);

    sleep(1);

    std::cout << "UpperServo test finish..." << std::endl;

    return ret;
}