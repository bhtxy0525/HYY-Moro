#include "WhrTeachInterfacePublic.h"
#include "HYYRobotInterface.h"
#include <thread>
#include <iostream>

namespace
{
    WhrModel robot;
    WhrTeachInterface teach_robot(robot);
}

using namespace HYYRobotBase;
int JointImpedanceControlDemoStart(int robot_index)
{
    double K[7] = {50, 50, 50, 30, 20, 20, 20};
    double D[7] = {2, 2, 2, 2, 2, 2, 2};
    int ret = teach_robot.ImpedanceControl(robot_index, K, D);
    if (ret != 0)
    {
        std::cerr
            << "Start Impedance Control failed: "
            << ret
            << std::endl;
    }

    return 0;
}

int JointImpedanceControlDemoStatus(int robot_index)
{
    int error_code = 0;
    int running = teach_robot.GetImpedanceControlState(robot_index, error_code);
    std::cout
        << "running = " << running
        << ", robot_index = " << robot_index
        << ", error_code = " << error_code
        << std::endl;
    return running;
}

int JointImpedanceControlDemoEnd()
{
    int stop_ret = teach_robot.StopImpedanceControl();
    return 0;
}

int ImpedanceControlMoveCDemo()
{
    ClearDeviceError();
    IMPORTJOINT(R0_PTEST);
    IMPORTJOINT(R0_IMPETEST);
    IMPORTPOSE(R0_p0); // init
    IMPORTPOSE(R0_p1); // high
    IMPORTPOSE(R0_p2); // final
    IMPORTPOSE(R0_p3); // low
    IMPORTSPEED(v0);
    IMPORTZONE(z1);
    RobotPower(0);
    MultiMoveA(R0_PTEST, v0, NULL3, 0);
    MultiMoveL(R0_p0, v0, NULL3, 0);
    // JointImpedanceControlDemoStart(0);
    int cnt = 0;
    while (robot_ok())
    {
        if (cnt > 20)
        {
            break;
        }

        MultiMoveC(R0_p2, R0_p1, v0, z1, NULL2, 0);
        MultiMoveC(R0_p0, R0_p3, v0, z1, NULL2, 0);
        cnt++;
    }
    // JointImpedanceControlDemoEnd();
    return 0;
}
int ImpedanceControlMoveADemo()
{
    ClearDeviceError();
    IMPORTJOINT(R0_IMPETEST1);
    IMPORTJOINT(R0_IMPETEST2);
    IMPORTJOINT(R0_IMPETEST3);
    IMPORTSPEED(v0);
    RobotPower(0);
    MultiMoveA(R0_IMPETEST1, v0, NULL3, 0);
    JointImpedanceControlDemoStart(0);
    Rsleep(3000);
    int cnt = 0;
    // MultiMoveA(R0_IMPETEST3, v0, NULL3, 0);
    while (robot_ok())
    {
        if (cnt > 10)
        {
            break;
        }

        MultiMoveA(R0_IMPETEST2, v0, NULL3, 0);
        MultiMoveA(R0_IMPETEST1, v0, NULL3, 0);
        cnt++;
    }
    JointImpedanceControlDemoEnd();
    return 0;
}
