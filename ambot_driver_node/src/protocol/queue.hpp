#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include "protocolStruct.hpp"
using namespace std::chrono_literals;

namespace ambot_driver_ns {

enum class DeviceType {
    MEC_ARM    = ID_MEC_ARM_UPLOAD,
    LIFTS      = ID_LIFTS_UPLOAD,
    JAW_MOTOR  = ID_JAW_MOTOR_UPLOAD
};

class UnifiedDeviceQueue {
public:
    // 强制深拷贝的存储结构
    struct SafePacket {
        DeviceType type;
        std::unique_ptr<uint8_t[]> data; // 智能指针管理内存
        size_t size;
        
        SafePacket(DeviceType t, const void* src, size_t s) 
            : type(t), data(new uint8_t[s]), size(s) {
            memcpy(data.get(), src, s);
        }
    };

    // 入队（强制深拷贝）
    bool push(DeviceType type, const void* data, size_t size, 
             std::chrono::milliseconds timeout = 50ms) 
    {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // 等待队列有空位
        if (!cv_push_.wait_for(lock, timeout, [this] { 
            return queue_.size() < max_capacity_; 
        })) {
            return false; // 入队超时
        }

        queue_.push(std::make_shared<SafePacket>(type, data, size));
        cv_pop_.notify_one();
        return true;
    }

    // 非阻塞出队
    bool try_pop(DeviceType& type, std::vector<uint8_t>& output) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;

        auto packet = queue_.front();
        type = packet->type;
        output.assign(packet->data.get(), packet->data.get() + packet->size);
        queue_.pop();
        cv_push_.notify_one();
        return true;
    }

    // 阻塞出队
    bool pop(DeviceType& type, std::vector<uint8_t>& output,
            std::chrono::milliseconds timeout = 100ms) 
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_pop_.wait_for(lock, timeout, [this] { 
            return !queue_.empty(); 
        })) {
            return false; // 超时
        }

        auto packet = queue_.front();
        type = packet->type;
        output.assign(packet->data.get(), packet->data.get() + packet->size);
        queue_.pop();
        cv_push_.notify_one();
        return true;
    }

    void set_capacity(size_t cap) {
        std::lock_guard<std::mutex> lock(mutex_);
        max_capacity_ = cap;
    }

private:
    std::queue<std::shared_ptr<SafePacket>> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_push_; // 生产者条件变量
    std::condition_variable cv_pop_;  // 消费者条件变量
    size_t max_capacity_ = 1000;      // 默认队列容量
};

} // namespace ambot_driver_ns