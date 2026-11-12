#include "HYYRobotInterface.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "nlohmann/json.hpp" //JSON所需
#include <vector>
#include <atomic>
#include <chrono>

#ifdef __cplusplus
extern "C"
{
#endif
    extern void PluginMain();
#ifdef __cplusplus
}
#endif

#define CYCLIE 10 // 数据发送周期, 单位ms
using namespace HYYRobotBase;

static zmq::context_t context(1);                                // zmq的根对象
static zmq::socket_t publisher(context, zmq::socket_type::pub);  // 发布消息，单独线程
static zmq::socket_t subscriber(context, zmq::socket_type::sub); // 接收消息，单独线程
static std::thread *pub_th = nullptr;
static std::thread *sub_th = nullptr;
static std::atomic<bool> is_stop(true);
static std::thread *stop_th = nullptr;

static std::vector<double> cartesian_tmp[2];
static double time_tmp[2] = {0, 0};
/**
 * @brief 数据保存
 * @return
 */
static void save_data()
{
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer, 1, 1);

    double feedback_data[52];
    double feedback_data_test[52];
    robpose p0, p1, p0_last, p1_last;
    while (!is_stop.load())
    {
        HYYRobotBase::userTimer(&timer);
        memset(feedback_data, 0, sizeof(feedback_data));
        memset(feedback_data_test, 0, sizeof(feedback_data_test));

        // 获取关节状态反馈
        HYYRobotBase::GetCurrentJoint(feedback_data, 0);
        HYYRobotBase::GetCurrentLastTargetJoint(&(feedback_data[7]), 0);
        HYYRobotBase::GetCurrentJoint(&feedback_data[14], 1);
        HYYRobotBase::GetCurrentLastTargetJoint(&(feedback_data[21]), 1);

        // 获取笛卡尔状态反馈
        HYYRobotBase::GetCurrentCartesian(NULL, NULL, &p0, 0);
        HYYRobotBase::GetCurrentLastTargetCartesian(NULL, NULL, &p0_last, 0);
        HYYRobotBase::GetCurrentCartesian(NULL, NULL, &p1, 1);
        HYYRobotBase::GetCurrentLastTargetCartesian(NULL, NULL, &p1_last, 1);
        memcpy(&(feedback_data[28]), p0.xyz, 3 * sizeof(double));
        memcpy(&(feedback_data[31]), p0.kps, 3 * sizeof(double));
        memcpy(&(feedback_data[34]), p0_last.xyz, 3 * sizeof(double));
        memcpy(&(feedback_data[37]), p0_last.kps, 3 * sizeof(double));
        memcpy(&(feedback_data[40]), p1.xyz, 3 * sizeof(double));
        memcpy(&(feedback_data[43]), p1.kps, 3 * sizeof(double));
        memcpy(&(feedback_data[46]), p1_last.xyz, 3 * sizeof(double));
        memcpy(&(feedback_data[49]), p1_last.kps, 3 * sizeof(double));

        HYYRobotBase::RSaveDataFast1("TeleFeedbackData", 1, 100, 52, feedback_data);
        if (cartesian_tmp[0].size() > 0 && cartesian_tmp[1].size() > 0)
        {
            for (int i = 0; i < 6; i++)
            {
                feedback_data_test[i] = cartesian_tmp[0][i];
            }
            for (int i = 0; i < 6; i++)
            {
                feedback_data_test[i + 6] = ((double *)(&p0))[i];
            }
            for (int i = 0; i < 6; i++)
            {
                feedback_data_test[i + 12] = cartesian_tmp[1][i];
            }
            for (int i = 0; i < 6; i++)
            {
                feedback_data_test[i + 18] = ((double *)(&p1))[i];
            }
        }
        feedback_data_test[24] = time_tmp[0];
        feedback_data_test[25] = time_tmp[1];
        HYYRobotBase::RSaveDataFast1("TeleFeedbackDataTest", 1, 100, 26, feedback_data_test);
    }
}
/**
 * @brief 循环发送消息
 * @return
 */
static void publisher_loop()
{
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer, 0, CYCLIE); // 10ms
    nlohmann::ordered_json data;                    // 初始化JSON容器，按插入顺序构造 JSON 数据
    printf("start publisher_loop\n");
    while (true)
    {
        HYYRobotBase::userTimer(&timer);
        int robot_num = HYYRobotBase::robot_getNUM(); // 获取机器人数量
        for (int i = 0; i < 3; i++)
        {
            data[std::string("Robot") + std::to_string(i)]["MoveState"] = HYYRobotBase::get_robot_move_state(i); // 运动状态
            data[std::string("Robot") + std::to_string(i)]["PowerState"] = HYYRobotBase::GetRobotPowerState(i);  // 使能状态
            int dof = HYYRobotBase::robot_getDOF(i);
            std::vector<double> joint(dof);
            HYYRobotBase::GetCurrentJoint(joint.data(), i);
            data[std::string("Robot") + std::to_string(i)]["Joint"] = joint; // 当前角度
            // HYYRobotBase::GetCurrentLastTargetJoint(joint.data(), 0);
            // data[std::string("Robot") + std::to_string(i)]["TargetJoint"] = joint; // 当前目标角度
            std::vector<double> Cartesian(6);
            HYYRobotBase::GetCurrentCartesian(NULL, NULL, (HYYRobotBase::robpose *)Cartesian.data(), i);
            data[std::string("Robot") + std::to_string(i)]["Cartesian"] = Cartesian; // 当前笛卡尔位姿
            // HYYRobotBase::GetCurrentLastTargetCartesian(NULL, NULL, (HYYRobotBase::robpose *)Cartesian.data(), 0);
            // data[std::string("Robot") + std::to_string(i)]["TargetCartesian"] = Cartesian; // 当前目标笛卡尔位姿
        }
        publisher.send(zmq::buffer("STAT " + data.dump())); // 前缀STAT+转字符串，发布出去，不关心是否有人接收
    }
}
/**
 * @brief 循环接收消息，控制机器人运动
 * @return
 */
static void subscriber_loop()
{
    printf("start subscriber_loop\n");
    while (true)
    {
        zmq::message_t msg;
        subscriber.recv(msg);
        std::string cmd(static_cast<char *>(msg.data()), msg.size());
        auto pos = cmd.find(' ');
        std::string topic = cmd.substr(0, pos);
        nlohmann::ordered_json cmd_json = nlohmann::json::parse(cmd.substr(pos + 1));
        // std::cout<<cmd_json.dump(4)<<std::endl;
        // std::cout<<topic<<std::endl;
        // int rn = HYYRobotBase::robot_getNUM();
        int robot_num = 2;
        if ("Switch" == topic)
        {
            if (cmd_json.contains("Switch"))
            {
                HYYRobotBase::ClearRobotError();
                if (cmd_json["Switch"].get<bool>())
                {

                    for (int i = 0; i < robot_num; i++)
                    {
                        is_stop.store(true);
                        int ret = HYYRobotBase::ServoStart(0.00001, 0.1, i);
                        printf("servostart %d ...\n", ret);
                        is_stop.store(false);
                        stop_th = new std::thread(save_data); // 开启数据记录
                        stop_th->detach();
                    }
                }
                else
                {
                    for (int i = 0; i < robot_num; i++)
                    {
                        is_stop.store(true);
                        HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                    }
                }
            }
        }
        else if ("Joint" == topic)
        {
            for (int i = 0; i < robot_num; i++)
            {
                if (cmd_json.contains(std::string("Robot") + std::to_string(i)))
                {
                    double time = cmd_json[std::string("Robot") + std::to_string(i)]["time"].get<double>();                             // 解析时间
                    std::vector<double> joint = cmd_json[std::string("Robot") + std::to_string(i)]["joint"].get<std::vector<double>>(); // 解析关节变量
                    HYYRobotBase::robjoint jt;
                    HYYRobotBase::init_robjoint(&jt, joint.data(), HYYRobotBase::robot_getDOF(i));
                    HYYRobotBase::ServoJoint(&jt, time, i);
                }
            }
        }
        else if ("Cartesian" == topic)
        {
            for (int i = 0; i < robot_num; i++)
            {
                if (cmd_json.contains(std::string("Robot") + std::to_string(i)))
                {
                    double time = cmd_json[std::string("Robot") + std::to_string(i)]["time"].get<double>();
                    std::vector<double> cartesian = cmd_json[std::string("Robot") + std::to_string(i)]["cartesian"].get<std::vector<double>>();
                    cartesian_tmp[i] = cartesian;
                    time_tmp[i] = time;
                    HYYRobotBase::robpose pt;
                    HYYRobotBase::init_robpose(&pt, cartesian.data(), cartesian.data() + 3);
                    HYYRobotBase::ServoCartesian(&pt, time, NULL, NULL, i);
                }
            }
        }
        else if ("Init" == topic)
        {
            ClearRobotError();
            IMPORTJOINT(R2_MOVE);
            SETJOINT2(INIT_HEAD, 0, M_PI / 3);
            IMPORTSPEED(v0);
            RobotPower(0);
            printf("robot %d power sucess...\n", 0);
            RobotPower(1);
            printf("robot %d power sucess...\n", 1);
            RobotPower(2);
            printf("robot %d power sucess...\n", 2);
            AdditionPower(0);
            printf("additional robot %d power sucess...\n", 0);
            printf("Robot init finish....\n");
            publisher.send(zmq::buffer("ENABLE"));
        }
    }
}
/**
 * @brief 启动遥操作后台服务
 * @return
 */
void PluginMain()
{
    publisher.set(zmq::sockopt::sndhwm, 0);    // 0 表示无限小队列，但行为是：不能缓存
    publisher.set(zmq::sockopt::immediate, 1); // SUB 未连接时直接丢弃，不会因队列满而阻塞
    publisher.bind("tcp://*:8000");            // PUB服务端，等SUB端连接
    pub_th = new std::thread(publisher_loop);
    pub_th->detach();                                // 非阻塞
    subscriber.connect("tcp://192.168.10.200:8001"); // 客户端静态ip
    subscriber.set(zmq::sockopt::subscribe, "");
    sub_th = new std::thread(subscriber_loop);
    sub_th->detach(); // 非阻塞
}
