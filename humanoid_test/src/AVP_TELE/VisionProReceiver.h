#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>

// ============================================================
// Apple Vision Pro实时数据
// ============================================================
struct VisionProData
{
    // 左手腕4×4齐次位姿矩阵
    Eigen::Matrix4d left_wrist =
        Eigen::Matrix4d::Identity();

    // 右手腕4×4齐次位姿矩阵
    Eigen::Matrix4d right_wrist =
        Eigen::Matrix4d::Identity();

    // 左手捏合状态
    // true：捏合
    // false：松开
    bool left_pinch = false;

    // 右手捏合状态
    // true：捏合
    // false：松开
    bool right_pinch = false;

    // Vision Pro数据连接状态
    // true：数据流正常
    // false：数据已经断流
    bool connected = false;

    // C++成功读取的数据帧编号
    uint64_t sequence = 0;
};

// ============================================================
// Apple Vision ProZMQ接收类
// ============================================================
class VisionProReceiver
{
public:
    // 构造函数
    explicit VisionProReceiver(
        const std::string &endpoint =
            "tcp://127.0.0.1:5557");

    // 析构函数
    ~VisionProReceiver();

    // 禁止拷贝
    VisionProReceiver(
        const VisionProReceiver &) = delete;

    VisionProReceiver &
    operator=(
        const VisionProReceiver &) = delete;

    // 启动ZMQ接收线程
    int Start();

    // 停止ZMQ接收线程
    void Stop();

    // 判断是否已经收到至少一帧数据
    bool HasData() const;

    // 获取最新一帧Vision Pro数据
    bool GetLatestData(
        VisionProData &data) const;

private:
    // ZMQ读取线程
    void ReceiveThread();

private:
    // ZMQ名称
    std::string endpoint_;

    // 接收线程运行标志
    std::atomic<bool> running_{false};

    // 是否已经收到有效数据
    std::atomic<bool> has_data_{false};

    // 接收线程
    std::thread receive_thread_;

    // 数据互斥锁
    mutable std::mutex data_mutex_;

    // 最新Vision Pro数据
    VisionProData latest_data_;
    std::chrono::steady_clock::time_point last_receive_time_{};
};