#include "VisionProReceiver.h"
#include <zmq.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace
{
    constexpr double kTimeoutSeconds = 0.2;
    // 协议：单帧280字节，35个小端IEEE754 float64；矩阵按行存储。
    double ReadDoubleLE(const unsigned char *p)
    {
        std::uint64_t bits = 0;
        for (int i = 0; i < 8; ++i)
            bits |= std::uint64_t(p[i]) << (8 * i);
        double value;
        static_assert(sizeof(value) == 8, "Requires 64-bit double");
        std::memcpy(&value, &bits, 8);
        return value;
    }
}

VisionProReceiver::VisionProReceiver(const std::string &endpoint)
    : endpoint_(endpoint) {}
VisionProReceiver::~VisionProReceiver() { Stop(); }

// Start/Stop由同一管理线程调用；ZMQ socket仅由接收线程使用。
int VisionProReceiver::Start()
{
    if (running_)
        return 0;
    if (receive_thread_.joinable())
        receive_thread_.join();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_data_ = VisionProData{};
        last_receive_time_ = {};
        has_data_ = false;
    }
    running_ = true;
    try
    {
        receive_thread_ = std::thread(&VisionProReceiver::ReceiveThread, this);
    }
    catch (...)
    {
        running_ = false;
        return -1;
    }
    return 0;
}
void VisionProReceiver::Stop()
{
    running_ = false;
    if (receive_thread_.joinable())
        receive_thread_.join();
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_.connected = false;
    latest_data_.left_pinch = latest_data_.right_pinch = false;
}
bool VisionProReceiver::HasData() const { return has_data_; }
bool VisionProReceiver::GetLatestData(VisionProData &data) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_data_)
        return false;
    data = latest_data_;
    if (!running_ || std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_receive_time_)
                             .count() > kTimeoutSeconds)
        data.connected = false;
    if (!data.connected)
        data.left_pinch = data.right_pinch = false;
    return true;
}
void VisionProReceiver::ReceiveThread()
{
    void *context = zmq_ctx_new();
    void *socket = context ? zmq_socket(context, ZMQ_SUB) : nullptr;
    try
    {
        if (!socket)
            throw std::runtime_error(zmq_strerror(zmq_errno()));
        const int zero = 0, one = 1, timeout_ms = 20;
        auto set = [&](int option, const void *value, size_t size)
        {
            if (zmq_setsockopt(socket, option, value, size) != 0)
                throw std::runtime_error(zmq_strerror(zmq_errno()));
        };
        set(ZMQ_LINGER, &zero, sizeof(zero));
        set(ZMQ_CONFLATE, &one, sizeof(one)); // 只保留最新完整消息
        set(ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
        set(ZMQ_SUBSCRIBE, "", 0);
        if (zmq_connect(socket, endpoint_.c_str()) != 0)
            throw std::runtime_error(zmq_strerror(zmq_errno()));
        std::cout << "[VisionProReceiver] SUB " << endpoint_ << std::endl;
        while (running_)
        {
            std::array<unsigned char, 280> buffer{};
            const int count = zmq_recv(socket, buffer.data(), buffer.size(), 0);
            if (count < 0)
            {
                if (zmq_errno() == EAGAIN || zmq_errno() == EINTR)
                    continue;
                throw std::runtime_error(zmq_strerror(zmq_errno()));
            }
            if (count != static_cast<int>(buffer.size()))
                continue;
            std::array<double, 35> values{};
            bool valid = true;
            for (size_t i = 0; i < values.size(); ++i)
            {
                values[i] = ReadDoubleLE(buffer.data() + 8 * i);
                valid = valid && std::isfinite(values[i]);
            }
            if (!valid)
                continue;
            using RowMatrix = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_data_.left_wrist = Eigen::Map<const RowMatrix>(values.data());
            latest_data_.right_wrist = Eigen::Map<const RowMatrix>(values.data() + 16);
            latest_data_.connected = values[34] > 0.5;
            latest_data_.left_pinch = latest_data_.connected && values[32] > 0.5;
            latest_data_.right_pinch = latest_data_.connected && values[33] > 0.5;
            ++latest_data_.sequence;
            last_receive_time_ = std::chrono::steady_clock::now();
            has_data_ = true;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[VisionProReceiver] " << e.what() << std::endl;
    }
    if (socket)
        zmq_close(socket);
    if (context)
        zmq_ctx_term(context);
    running_ = false;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_.connected = false;
    latest_data_.left_pinch = latest_data_.right_pinch = false;
}
