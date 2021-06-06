#ifndef SONICLINK_RECV_H
#define SONICLINK_RECV_H

#include <thread>
#include <mutex>
#include <functional>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include "core.h"





class Receiver {
public:
    Receiver(const CarrierSpecs& specs, const PacketMetadata& meta,
             SignalMapFunc map_func, SignalReduceFunc reduce_func,
             SignalCostFunc cost_func);

    auto& msg_mutex()
    {
        return msg_mutex_;
    }

    auto& msg_buffer()
    {
        return msg_buffer_;
    }

private:
    static const auto lockfree_raw_buffer_size = 1ul << 20;
    static const auto raw_buffer_size = 1ul << 20;
    static const auto bit_buffer_size = 1ul << 20;
    static const auto msg_buffer_size = 1ul << 10;

private:
    std::mutex raw_mutex_, bit_mutex_, msg_mutex_;
    std::thread msg_loop_thread_, decode_loop_thread_, pour_thread_;
    boost::lockfree::spsc_queue<float> lockfree_raw_buffer_;
    boost::circular_buffer<float> raw_buffer_;
    boost::circular_buffer<BitType> bit_buffer_;
    boost::circular_buffer<std::vector<BitType>> msg_buffer_;
    CarrierSpecs specs_;
    PacketMetadata meta_;
    SignalMapFunc map_func_;
    SignalReduceFunc reduce_func_;
    SignalCostFunc cost_func_;
};


#endif
