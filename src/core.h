#ifndef SONICLINK_CORE_H
#define SONICLINK_CORE_H

constexpr auto sample_rate = 44100u;
constexpr auto frames_per_buffer = 1024u;

constexpr auto bytes_per_msg_data = 10;
constexpr auto bytes_per_ack_data = 16;

constexpr auto bytes_per_msg_body = bytes_per_msg_data + 4;
constexpr auto bytes_per_msg_ack_body = bytes_per_ack_data + 4;



#include <cstddef>
#include <vector>
#include <functional>
#include <string_view>
#include <stdexcept>
#include <boost/circular_buffer.hpp>

using BitType = unsigned char;

struct PacketMetadata {
    std::vector<BitType> head_signature;
    std::vector<BitType> tail_signature;
    std::size_t bits_per_body;

    auto bits_per_packet() const
    {
        return bits_per_body + head_signature.size() + tail_signature.size();
    }
};

struct CarrierSpecs {
    std::size_t samples_per_bit;
    std::size_t bits_per_prefix;
    std::size_t bits_per_suffix;
    float frequency;
};

using CircularIter = boost::circular_buffer<float>::const_iterator;
using SignalMapFunc = std::function<std::vector<float>(CircularIter first,
                                                       CircularIter last,
                                                       const CarrierSpecs&)>;
using SignalReduceFunc = std::function<
                        std::vector<BitType>(const std::vector<float>&)>;
using SignalCostFunc = std::function<float(const std::vector<float>&)>;
using BitsEncodeFunc = std::function<std::vector<float>(
                const std::vector<BitType>& bits, const CarrierSpecs& specs)>;

#define PACKET_PREFIX \
    "1000010000100001000010000100001000010000"
    // "0111011101110111011101110"

#define HEAD_SIGNATURE \
    "0111000011001010"


#define TAIL_SIGNATURE \
    "11101011"

#define PACKET_SUFFIX \
    "00000000000000000000000000000000"

#endif
