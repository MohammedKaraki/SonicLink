#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <chrono>
#include <thread>
#include <mutex>
#include <algorithm>
#include <vector>
#include <cstddef>
#include <boost/circular_buffer.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <portaudio.h>
#include "core.h"

#define FMT_HEADER_ONLY
#include <fmt/format.h>
#include <fmt/color.h>

std::mutex raw_mutex;
std::mutex bit_mutex;
std::mutex msg_mutex;

namespace thread_safe {
    static inline auto size(const auto& buffer, auto& mutex)
    {
        auto lock = std::lock_guard{mutex};
        return buffer.size();
    }
}

int record_callback(const void *input_buffer,
                    void * /* output_buffer */,
                    unsigned long /* frame_count */,
                    const PaStreamCallbackTimeInfo * /* time_info */,
                    PaStreamCallbackFlags /* status_flags */,
                    void *lockfree_raw_buffer_ptr/* user_data */)
{
    auto& lockfree_raw_buffer = *static_cast<
        boost::lockfree::spsc_queue<float> *>(lockfree_raw_buffer_ptr);

    auto rptr = static_cast<const float *>(input_buffer);

    if (input_buffer != nullptr)
    {
        for (auto i = 0u; i < frames_per_buffer; i++)
        {
            float r = *rptr++;
            assert(lockfree_raw_buffer.push(r));
        }
    }

    return paContinue; // Never stop recording
}


void launch_mic_loop(boost::lockfree::spsc_queue<float>& lockfree_raw_buffer)
{
        PaError error = paNoError;
        PaStream *input_stream;
        PaStreamParameters input_params;

        error = Pa_Initialize();
        if (error != paNoError) {
                Pa_Terminate();
                fmt::print(stderr, "{}: {}\n",
                           error, Pa_GetErrorText(error));
                exit(1);
        }

        input_params.device = Pa_GetDefaultInputDevice();
        if (input_params.device == paNoDevice) {
                fmt::print(stderr, "Error: No default input device.\n");
                exit(1);
        }

        input_params.channelCount = 1;
        input_params.sampleFormat = paFloat32;
        input_params.suggestedLatency =
                Pa_GetDeviceInfo(input_params.device)->defaultLowInputLatency;
        input_params.hostApiSpecificStreamInfo = nullptr;

        error = Pa_OpenStream(&input_stream,
                              &input_params,
                              nullptr /* output params */,
                              sample_rate,
                              frames_per_buffer,
                              paClipOff /* stream flags */,
                              record_callback,
                              &lockfree_raw_buffer /* user data */);
        if (error != paNoError) {
                fmt::print(stderr, "{}: {}\n",
                           error, Pa_GetErrorText(error));
                exit(1);
        }

        fmt::print(stderr, "starting mic loop\n");
        error = Pa_StartStream(input_stream);
        if (error != paNoError) {
                fmt::print(stderr, "{}: {}\n",
                           error,
                           Pa_GetErrorText(error));
                exit(1);
        }
}

// Is the front of the bit buffer a valid packet in the sense of having
// correct head and tail signatures and having a correct packet size?
bool check_match(const boost::circular_buffer<BitType>& bit_buffer,
                 const PacketMetadata& meta)
{
    for (auto i = 0u; i < meta.head_signature.size(); ++i) {
        if (meta.head_signature[i] != bit_buffer[i]) {
            return false;
        }
    }

    for (auto i = 0u; i < meta.tail_signature.size(); ++i) {
        auto tail_offset = meta.head_signature.size()
            + meta.bits_per_body;
        if (meta.tail_signature[i] != bit_buffer[i + tail_offset]) {
            return false;
        }
    }

    return true;
}



void extract_msg_loop(boost::circular_buffer<BitType>& bit_buffer,
                      boost::circular_buffer<std::vector<BitType>>& msgs,
                      const PacketMetadata& meta)
{
    const auto packet_size = meta.bits_per_packet();

    fmt::print(stderr, "starting extract msg loop\n");
    while (true) {
        while (thread_safe::size(bit_buffer, bit_mutex) < packet_size) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        auto bit_lock = std::lock_guard{bit_mutex};
        if (bit_buffer.size() < packet_size) {
            continue;
        }

        auto normal_match = check_match(bit_buffer, meta);

        if (normal_match) {

            std::vector<BitType> msg(meta.bits_per_body);
            std::copy_n(bit_buffer.begin() + meta.head_signature.size(),
                        meta.bits_per_body,
                        msg.begin());

            {
                auto msg_lock = std::lock_guard{msg_mutex};
                msgs.push_back(msg);
            }

            assert(bit_buffer.size() >= packet_size);
            bit_buffer.erase_begin(packet_size);

        }
        else {
            bit_buffer.pop_front();
        }
    }
}


template<class Vec>
auto inner_product(const Vec& first, const Vec& second)
{
    assert(!first.empty());
    assert(first.size() == second.size());

    return std::inner_product(first.begin(), first.end(), second.begin(),
                              decltype(first[0]){ })
        / first.size();
}

template<class Iter>
std::vector<float> decode(Iter first_it,
                          std::size_t num_bits,
                          const CarrierSpecs& carrier_specs)
{
    std::vector<float> result(num_bits);

    const auto N = carrier_specs.samples_per_bit;

    auto first = std::vector<float>(N);
    auto second = std::vector<float>(N);

    std::copy_n(first_it, N, first.begin());

    auto sign = 1;
    for (auto i = 0u; i < num_bits; ++i) {
        std::copy_n(first_it + (i+1)*N, N, second.begin());

        result[i] = sign * inner_product(first, second);
        sign = result[i]>0 ? 1 : -1;

        first = second;
    }

    return result;
}

float calc_cost(const std::vector<float>& decoded)
{
    auto result = 0.0f;

    auto cost_func = [](float x) {
        return 1.0 / (x*x + 1e-100);
    };

    for (auto d : decoded) {
        result += cost_func(d);
    }

    return result;
}

BitType digitize(float decoded)
{
    return decoded > 0.0 ? 1 : 0;
}

std::vector<BitType> digitize(const std::vector<float>& decoded, bool invert)
{
    std::vector<BitType> result;
    result.reserve(decoded.size());

    if (!invert) {
        for (auto d : decoded) {
            result.push_back(digitize(d));
        }
    }
    else {
        for (auto d : decoded) {
            result.push_back(1 - digitize(d));
        }
    }

    return result;
}

// Valid packet prefix looks like "0111011101110111..."
bool valid_prefix(const std::vector<BitType>& bits)
{
    // Just to make sure that a packet prefix passing this test isn't too small
    if (bits.size() < 5) {
        return false;
    }

    if (bits[0] + bits[1] + bits[2] + bits[3] != 3) {
        return false;
    }

    auto start = 0u;
    for (auto i = 1; i < 4; ++i) {
        if (bits[i] == 0) {
            start = i;
            break;
        }
    }

    for (auto i = start; i < bits.size()-3; i += 4) {
        if (bits[i] != 0 ||
            bits[i+1]*bits[i+2]*bits[i+3] != 1)
        {
            return false;
        }
    }

    return true;
}

void decode_loop(boost::circular_buffer<float>& raw_buffer,
                 boost::circular_buffer<BitType>& bit_buffer,
                 const CarrierSpecs& carrier_specs,
                 const PacketMetadata& packet_meta)
{
    const auto N = carrier_specs.samples_per_bit;

    const auto min_raw_buffer_size = (packet_meta.bits_per_packet()
                                      + carrier_specs.bits_per_prefix/2
                                      + carrier_specs.bits_per_suffix/2) * N;

    fmt::print(stderr, "start decode loop\n");
    while (true) {
        while (thread_safe::size(raw_buffer, raw_mutex) < min_raw_buffer_size) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        auto lock = std::lock_guard{raw_mutex};

        if (raw_buffer.size() < min_raw_buffer_size) {
            continue;
        }
        const auto bits_per_half_prefix = carrier_specs.bits_per_prefix / 2;

        auto best_shift = 0;
        auto best_cost = std::numeric_limits<float>::max();
        auto best_decoded = std::vector<float>{ };


        for (auto shift = 0u; shift < N; shift += samples_per_cycle/2) {
            auto curr_decoded = decode(raw_buffer.begin() + shift,
                                       bits_per_half_prefix,
                                       carrier_specs);
            auto curr_cost = calc_cost(curr_decoded);
            if (curr_cost < best_cost) {
                best_cost = curr_cost;
                best_decoded = curr_decoded;
                best_shift = shift;
            }
        }

        auto digitized = digitize(best_decoded, false);
        auto digitized_inverted = digitize(best_decoded, true);

        bool invert = false;
        if (valid_prefix(digitized)) {
        }
        else if (valid_prefix(digitized_inverted)) {
            invert = true;
        }
        else {
            raw_buffer.erase_begin(N * bits_per_half_prefix / 2);
            continue;
        }

        static int k = 0;
        fmt::print(stderr, "\n-----{}-----{}-----\n", k++, raw_buffer.size());
        std::vector<BitType> data = digitize(
            decode(raw_buffer.begin() + best_shift,
                   min_raw_buffer_size/N - 3,
                   carrier_specs), invert);

        {
            auto bit_lock = std::lock_guard{bit_mutex};
            for (auto bit : data) {
                // fmt::print(stderr, "{}", bit);
                bit_buffer.push_back(bit);
            }
        }

        // TODO: maybe should erase a different amount (based on how much was
        // consumed in decode(), (or maybe it doesn't matter since the edge will
        // reside in a `telomere'
        raw_buffer.erase_begin(min_raw_buffer_size);
    }
}


// Move raw data from the thread-safe buffer to the thread-unsafe buffer
void pour_to_raw_buffer(boost::lockfree::spsc_queue<float>& raw_queue,
                        boost::circular_buffer<float>& raw_buffer)
{
    constexpr auto samples_per_pour = sample_rate / 10;

    while (true) {
        while (raw_queue.read_available() < samples_per_pour) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }

        auto lock = std::lock_guard{raw_mutex};
        for (auto i = 0u; i < samples_per_pour; ++i) {
            auto sample = float{};
            raw_queue.pop(sample);
            raw_buffer.push_back(sample);
        }
    }
}

int main()
{
    auto lockfree_raw_buffer = boost::lockfree::spsc_queue<float>(1ul << 20);
    auto raw_buffer = boost::circular_buffer<float>(1ul << 20);
    auto bit_buffer = boost::circular_buffer<BitType>(1ul << 20);
    auto msgs = boost::circular_buffer<std::vector<BitType>>(50);

    auto meta = PacketMetadata {
        .head_signature = bits_from_str(HEAD_SIGNATURE),
        .tail_signature = bits_from_str(TAIL_SIGNATURE),
        .bits_per_body = strlen(PACKET_BODY)
    };

    auto carrier_specs = CarrierSpecs {
        .samples_per_bit = samples_per_bit,
        .bits_per_prefix = strlen(PACKET_PREFIX),
        .bits_per_suffix = strlen(PACKET_SUFFIX),
        .frequency = frequency
    };



    auto l1 = std::thread(extract_msg_loop, std::ref(bit_buffer),
                          std::ref(msgs), std::cref(meta));
    auto l2 = std::thread(decode_loop, std::ref(raw_buffer),
                          std::ref(bit_buffer), std::ref(carrier_specs),
                          std::ref(meta));
    auto l3 = std::thread(pour_to_raw_buffer, std::ref(lockfree_raw_buffer),
                          std::ref(raw_buffer));
    launch_mic_loop(lockfree_raw_buffer);

    const auto body = std::string{PACKET_BODY};

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds{1});

        while (thread_safe::size(msgs, msg_mutex) > 0) {
            auto msg_lock = std::lock_guard{msg_mutex};

            if (msgs.empty()) {
                break;
            }

            auto msg = *msgs.begin();
            msgs.pop_front();
            fmt::print("\nmsg: ");
            for (auto i = 0u; i < msg.size(); ++i) {
                fmt::print(msg[i]+'0'==body[i]
                           ? fmt::bg(fmt::color::green)
                           : fmt::bg(fmt::color::red),
                           "{}", msg[i]);
            }
            fmt::print("\n");
        }
    }
}
