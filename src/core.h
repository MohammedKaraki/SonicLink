#include <cstddef>
#include <vector>
#include <string_view>
#include <stdexcept>

using BitType = unsigned char;

constexpr auto sample_rate = 44100u;
constexpr auto time_unit = 1.0f / sample_rate;
constexpr auto frames_per_buffer = 1024u >> 2;

constexpr auto freq_factor = 1;

constexpr auto samples_per_cycle = 12 * freq_factor;
constexpr auto frequency = 1.0f / (samples_per_cycle);
constexpr auto samples_per_bit = 4 * samples_per_cycle / freq_factor;


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

#define PACKET_PREFIX \
    "01110111011101110111011101110111011101110"

#define HEAD_SIGNATURE \
    "1111000011001010"

// Fixed msg for testing purposes
#define PACKET_BODY \
    "0001010101010101010111000000000011111111110000011111000110001010101010100"\
    "1011100000000001111111111000001111100011101110000000000111111111100000111"\
    "1000111100010101010101010101110000000000111111111100000111110001100010100"\
    "1010101010111000000000011111111110000011111000110001010101010101010111000"\
    "0000001111111111000001111100011101110000000000111111111100000111110001110"\
    "0010101010101010101110000000000111111111100000111110001111000101010101011"\
    "0101110000000000111111111100000111110001100010101010101010101110000000001"\
    "1111111110000011111000111011100000000001111111111000001111100011110001011"\
    "01010101010111000000000011111111110000011111000111"

#define TAIL_SIGNATURE \
    "111010111"

#define PACKET_SUFFIX \
    "00000000000000000000000000000000000000000"


// TODO: Move the definition out of the header file.
auto bits_from_str(std::string_view str)
{
    auto result = std::vector<BitType>();
    result.reserve(str.size());

    for (auto c : str) {
        if (c == '1') {
            result.push_back(1);
        }
        else if (c == '0') {
            result.push_back(0);
        }
        else {
            throw std::runtime_error("Invalid input string");
        }
    }

    return result;
}
