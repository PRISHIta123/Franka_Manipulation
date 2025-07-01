#pragma once

#include <array>
#include <atomic>

namespace franka_bindings {

template <size_t N>
struct RealTimeBuffer {
    alignas(64) std::array<std::array<double,7>, N> positions;
    std::atomic<uint64_t> head{0};
    std::atomic<uint64_t> tail{0};
    
    bool try_write(const std::array<double,7>& pos) noexcept {
        const uint64_t current_head = head.load(std::memory_order_relaxed);
        const uint64_t current_tail = tail.load(std::memory_order_acquire);
        if(current_head - current_tail < N) {
            positions[current_head % N] = pos;
            head.store(current_head + 1, std::memory_order_release);
            return true;
        }
        return false;
    }
    
    bool try_read(std::array<double,7>& pos) noexcept {
        const uint64_t current_tail = tail.load(std::memory_order_relaxed);
        const uint64_t current_head = head.load(std::memory_order_acquire);
        if(current_head > current_tail) {
            pos = positions[current_tail % N];
            tail.store(current_tail + 1, std::memory_order_release);
            return true;
        }
        return false;
    }
};

struct SharedMemory {
    // From Python
    std::atomic<uint64_t> command_sequence{0};
    std::array<double, 7> target_joint_position;
    
    // To Python (read-only)
    std::atomic<uint64_t> state_sequence{0};
    std::array<double, 7> current_joint_position;
    std::array<double, 7> current_joint_velocity;
    std::array<double, 16> current_ee_pose;

    RealTimeBuffer<8> command_buffer;
    RealTimeBuffer<8> state_buffer;
    std::atomic<bool> emergency_stop{false};
    std::atomic<uint32_t> sequence{0};
};

}  // namespace franka_bindings 