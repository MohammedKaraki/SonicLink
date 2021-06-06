#ifndef SONICLINK_SEND_H
#define SONICLINK_SEND_H

#include <mutex>
#include <boost/circular_buffer.hpp>
#include "core.h"

void start_broadcasting(boost::circular_buffer<float>& raw_buffer,
                        const CarrierSpecs& specs,
                        std::mutex& raw_mutex);


class Sender {
public:
    Sender(const CarrierSpecs& specs, BitsEncodeFunc encode_func);
    void encode_and_queue(const std::vector<BitType>& bits);

private:
    static const auto raw_buffer_size = 1ul << 20;

private:
    std::mutex raw_mutex_;
    boost::circular_buffer<float> raw_buffer_;
    CarrierSpecs specs_;
    BitsEncodeFunc encode_func_;
};



#endif
