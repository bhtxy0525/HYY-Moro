#include <iostream>
#include <string>
#include <array>

#include <zmq.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using Pose6D = std::array<double, 6>;

/**
 * @brief 从JSON对象中读取指定key的6D位姿数据
 * @param j JSON对象
 * @param key 要读取的键名
 * @param pose 输出的6D位姿数组
 * @return 成功返回true，失败返回false
 */
static bool readPose(const json &j, const std::string &key, Pose6D &pose)
{
    if (!j.contains(key))
        return false;
    if (!j[key].is_array())
        return false;
    if (j[key].size() != 6)
        return false;

    for (int i = 0; i < 6; ++i)
    {
        pose[i] = j[key][i].get<double>();
    }

    return true;
}

/**
 * @brief 打印6D位姿信息到控制台
 * @param name 位姿名称前缀
 * @param p 6D位姿数组
 */
static void printPose(const std::string &name, const Pose6D &p)
{
    std::cout << name
              << ": x=" << p[0]
              << ", y=" << p[1]
              << ", z=" << p[2]
              << ", k=" << p[3]
              << ", p=" << p[4]
              << ", s=" << p[5]
              << std::endl;
}

/**
 * @brief 使用示例：通过ZMQ请求视觉系统获取机械臂末端位姿
 * @return 成功返回0，失败返回1
 */
int ZMQ6DPoseDemo()
{
    // ==================== 1. 网络配置 ====================
    std::string server_ip = "192.168.10.45"; // 视觉主机（Jetson）的IP地址
    int port = 5555;                         // ZMQ服务端口号

    // 构建ZMQ连接地址（TCP协议）
    std::string addr = "tcp://" + server_ip + ":" + std::to_string(port);

    // ==================== 2. ZMQ初始化 ====================
    zmq::context_t ctx(1);                          // 创建ZMQ上下文（IO线程数1）
    zmq::socket_t sock(ctx, zmq::socket_type::req); // 创建REQ类型的socket（请求-响应模式客户端）

    // 设置ZMQ选项
    sock.set(zmq::sockopt::linger, 0);      // 关闭时立即丢弃未发送数据，不等待
    sock.set(zmq::sockopt::rcvtimeo, 3000); // 接收超时：3000毫秒
    sock.set(zmq::sockopt::sndtimeo, 3000); // 发送超时：3000毫秒

    // ==================== 3. 连接服务器 ====================
    std::cout << "[ROBOT] connect: " << addr << std::endl;
    sock.connect(addr);

    // ==================== 4. 构建请求消息 ====================
    json req;
    req["cmd"] = "get_pose"; // 请求命令：获取位姿

    std::string req_str = req.dump(); // 将JSON序列化为字符串

    // ==================== 5. 发送请求 ====================
    std::cout << "[ROBOT] send: " << req_str << std::endl;

    auto send_ret = sock.send(zmq::buffer(req_str), zmq::send_flags::none);
    if (!send_ret)
    {
        std::cerr << "[ROBOT] send failed or timeout" << std::endl;
        return 1;
    }

    // ==================== 6. 接收响应 ====================
    zmq::message_t reply;
    auto recv_ret = sock.recv(reply, zmq::recv_flags::none);
    if (!recv_ret)
    {
        std::cerr << "[ROBOT] recv timeout" << std::endl;
        return 1;
    }

    // ==================== 7. 解析响应消息 ====================
    std::string resp_str(static_cast<char *>(reply.data()), reply.size());
    std::cout << "[ROBOT] recv: " << resp_str << std::endl;

    json resp;
    try
    {
        resp = json::parse(resp_str); // 解析JSON响应
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ROBOT] json parse failed: " << e.what() << std::endl;
        return 1;
    }

    // ==================== 8. 检查响应状态 ====================
    bool ok = resp.value("ok", false); // 获取操作是否成功
    if (!ok)
    {
        std::cerr << "[ROBOT] vision failed: "
                  << resp.value("msg", "") // 错误消息
                  << ", error="
                  << resp.value("error", "") // 错误码
                  << std::endl;
        return 1;
    }

    // ==================== 9. 验证机械臂标识 ====================
    std::string arm = resp.value("arm", ""); // 获取机械臂标识（"right"或"left"）
    if (arm != "right" && arm != "left")
    {
        std::cerr << "[ROBOT] invalid arm: " << arm << std::endl;
        return 1;
    }

    // ==================== 10. 读取位姿数据 ====================
    Pose6D pose;
    if (!readPose(resp, "pose", pose)) // 从响应中提取6D位姿
    {
        std::cerr << "[ROBOT] pose invalid" << std::endl;
        return 1;
    }

    // ==================== 11. 输出结果 ====================
    std::cout << "[ROBOT] arm = " << arm << std::endl;
    printPose("[ROBOT] pose", pose); // 打印位姿信息

    std::cout << "[ROBOT] communication success" << std::endl;

    return 0;
}