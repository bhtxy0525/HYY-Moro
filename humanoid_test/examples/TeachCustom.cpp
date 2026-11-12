#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <iostream>
#include <atomic>
#include <mutex>
#include <sstream>
#include <vector>
#include <time.h>
#include <sys/time.h>
#include "nlohmann/json.hpp"
#include "http/httplib.h"
#include "HYYRobotInterface.h"
#include "WhrModelPublic.h"
#include "WhrTeachInterfacePublic.h"

WhrModel robot;
WhrTeachInterface teach_robot(robot);

using Vector6d = Eigen::Matrix<double, 6, 1>;
using namespace HYYRobotBase;

Vector6d MatrixToPose(const Eigen::Matrix4d &T)
{
    Vector6d pose;

    pose(0) = T(0, 3);
    pose(1) = T(1, 3);
    pose(2) = T(2, 3);

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    double r[9] =
        {
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2)};

    Rotation2RPY(r, &pose(3), &pose(4), &pose(5));

    return pose;
}

void custom_http(void *arg)
{
    httplib::Server *svr = (httplib::Server *)arg;
    // ==================== 笛卡尔 JOG ====================
    svr->Post("/api/cartesian_jog_interface", [&](const httplib::Request &req, httplib::Response &res)
              {
        int error_code = 0;

        ROBOT_INFO("/api/cartesian_jog_interface\n");

        try
        {
            nlohmann::json j = nlohmann::json::parse(req.body);

            if (j.contains("robot_index") &&
                j.contains("axis_index") &&
                j.contains("speed_scale") &&
                j.contains("dir"))
            {
                int robot_index = j["robot_index"];
                int axis_index = j["axis_index"];
                int dir = j["dir"];
                double speed_scale = j["speed_scale"];
                if (speed_scale<0)
                {
                    speed_scale = 0;
                }
                if (speed_scale>1)
                {
                    speed_scale = 1;
                }
                if (dir == 0)
                {
                    error_code = teach_robot.stop_cartesian_jog_interface();
                }
                else
                {
                    error_code = teach_robot.cartesian_jog_interface(robot_index, axis_index, dir,speed_scale);
                }
            }
            else
            {
                error_code = -134;
                ROBOT_ERROR("/api/cartesian_jog_interface param error\n");
            }

            res.set_content(
                nlohmann::json{
                    {"status", error_code}
                }.dump(),
                "application/json; charset=utf-8");
        }
        catch (...)
        {
            error_code = -1;
            ROBOT_ERROR("/api/cartesian_jog_interface json error\n");

            res.set_content(
                nlohmann::json{
                    {"status", error_code}
                }.dump(),
                "application/json; charset=utf-8");
        } });
    // ==================== 开启双臂拖动示教 ====================
    svr->Post("/api/start_drag_teach_interface",
              [](const httplib::Request &req, httplib::Response &res)
              {
                  int error_code = 0;

                  ROBOT_INFO("/api/start_drag_teach_interface\n");

                  try
                  {
                      const nlohmann::json j = nlohmann::json::parse(req.body);

                      if (!j.contains("arm_mode") ||
                          !j.contains("feel_mode") ||
                          !j.contains("drag_mode") ||
                          !j["arm_mode"].is_number_integer() ||
                          !j["feel_mode"].is_number_integer() ||
                          !j["drag_mode"].is_number_integer())
                      {
                          error_code = -134;
                          ROBOT_ERROR("/api/start_drag_teach_interface param error\n");
                      }
                      else
                      {
                          const int arm_mode = j["arm_mode"].get<int>();
                          const int feel_mode = j["feel_mode"].get<int>();
                          const int drag_mode = j["drag_mode"].get<int>();

                          error_code = teach_robot.start_drag_teach_interface(
                              arm_mode,
                              feel_mode,
                              drag_mode);
                      }
                  }
                  catch (...)
                  {
                      error_code = -1;
                      ROBOT_ERROR("/api/start_drag_teach_interface json error\n");
                  }

                  res.set_content(
                      nlohmann::json{{"status", error_code}}.dump(),
                      "application/json; charset=utf-8");
              });

    // ==================== 关闭双臂拖动示教 ====================
    svr->Post("/api/stop_drag_teach_interface",
              [](const httplib::Request &, httplib::Response &res)
              {
                  ROBOT_INFO("/api/stop_drag_teach_interface\n");

                  const int error_code = teach_robot.stop_drag_teach_interface();

                  res.set_content(
                      nlohmann::json{{"status", error_code}}.dump(),
                      "application/json; charset=utf-8");
              });
}

void custom_websocket(void *arg)
{
    try
    {
        nlohmann::json *data = (nlohmann::json *)arg;
        Eigen::Matrix4d TL, TR, TT;

        const char *left_arm = get_name_robot_device(HYYRobotBase::get_deviceName(0, nullptr), 0);
        const char *right_arm = get_name_robot_device(HYYRobotBase::get_deviceName(0, nullptr), 1);
        const char *torso_arm = get_name_robot_device(HYYRobotBase::get_deviceName(0, nullptr), 2);
        double q_array_torso_left[10], q_array_torso_right[10], q_array_left[7], q_array_right[7], q_array_torso[3];
        GetGroupPosition(left_arm, q_array_left);
        GetGroupPosition(right_arm, q_array_right);
        GetGroupPosition(torso_arm, q_array_torso);

        memcpy(q_array_torso_left, q_array_torso, 3 * sizeof(double));
        memcpy(q_array_torso_left + 3, q_array_left, 7 * sizeof(double));
        memcpy(q_array_torso_right, q_array_torso, 3 * sizeof(double));
        memcpy(q_array_torso_right + 3, q_array_right, 7 * sizeof(double));
        robot.fkineTL(q_array_torso_left, TL);
        robot.fkineTR(q_array_torso_right, TR);
        robot.fkineT(q_array_torso, TT);
        std::vector<double> q_torso_left_vec(q_array_torso_left, q_array_torso_left + 10);
        std::vector<double> q_torso_right_vec(q_array_torso_right, q_array_torso_right + 10);
        std::vector<double> q_torso_vec(q_array_torso, q_array_torso + 3);

        // 读取左右臂外力观测值
        double left_observer_force[7] = {0.0};
        double right_observer_force[7] = {0.0};

        teach_robot.get_observer_force(left_observer_force, right_observer_force);
        std::vector<double> left_observer_force_vec(left_observer_force, left_observer_force + 7);
        std::vector<double> right_observer_force_vec(right_observer_force, right_observer_force + 7);

        int drag_arm_mode = 0;
        int drag_feel_mode = 0;
        int drag_mode = 0;
        int drag_error_code = 0;
        const int drag_running = teach_robot.get_drag_teach_state(drag_arm_mode, drag_feel_mode, drag_mode, drag_error_code);

        (*data)["left_chain"]["joint"] = q_torso_left_vec;
        (*data)["left_chain"]["cartesian"] = MatrixToPose(TL);
        (*data)["left_arm"]["observer_force"] = left_observer_force_vec;
        (*data)["right_chain"]["joint"] = q_torso_right_vec;
        (*data)["right_chain"]["cartesian"] = MatrixToPose(TR);
        (*data)["right_arm"]["observer_force"] = right_observer_force_vec;
        (*data)["torso_chain"]["joint"] = q_torso_vec;
        (*data)["torso_chain"]["cartesian"] = MatrixToPose(TT);
        (*data)["drag_teach"]["running"] = drag_running;
        (*data)["drag_teach"]["arm_mode"] = drag_arm_mode;
        (*data)["drag_teach"]["feel_mode"] = drag_feel_mode;
        (*data)["drag_teach"]["drag_mode"] = drag_mode;
        (*data)["drag_teach"]["error_code"] = drag_error_code;
    }
    catch (...)
    { // 必须使用，防止json异常导致系统崩溃
        ROBOT_ERROR("/api/custom_cmd error\n");
    }
}