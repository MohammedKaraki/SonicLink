#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <chrono>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include <vector>
#include <cstddef>
#include <cstdio>
#include <boost/circular_buffer.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <portaudio.h>
#include "core.h"
#include "recv.h"

#include <fmt/format.h>
#include <fmt/color.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
auto logger = spdlog::rotating_logger_mt("receiver",
                                         "logs/log.txt",
                                         1ul << 20,
                                         2);


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
                logger->error(fmt::format("{}: {}\n",
                           error, Pa_GetErrorText(error)));
                exit(1);
        }

        input_params.device = Pa_GetDefaultInputDevice();
        if (input_params.device == paNoDevice) {
                logger->error(fmt::format("Error: No default input device.\n"));
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
                logger->error(fmt::format("{}: {}\n",
                           error, Pa_GetErrorText(error)));
                exit(1);
        }

        logger->info(fmt::format("starting mic loop\n"));
        error = Pa_StartStream(input_stream);
        if (error != paNoError) {
                logger->error(fmt::format("{}: {}\n",
                           error,
                           Pa_GetErrorText(error)));
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
                      const PacketMetadata& meta,
                      std::mutex& bit_mutex,
                      std::mutex& msg_mutex)
{
    const auto packet_size = meta.bits_per_packet();

    logger->info(fmt::format("starting extract msg loop\n"));
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


// Valid packet prefix looks like "0111011101110111..."
bool valid_prefix(const std::vector<BitType>& bits)
{
    for (auto i = 0u; i < bits.size()-4; i++) {
        if (bits[i] + bits[i+1] + bits[i+2] + bits[i+3] + bits[i+4] != 1) {
            return false;
        }
    }

    return true;
}


std::size_t find_best_delay(CircularIter signal_first,
                            CircularIter signal_last,
                            SignalMapFunc map_func,
                            SignalCostFunc cost_func,
                            const CarrierSpecs& specs)
{
    const auto samples_per_bit = specs.samples_per_bit;
    const auto total_samples = static_cast<std::size_t>(
        std::distance(signal_first, signal_last));

    assert(total_samples > samples_per_bit);

    auto best_delay = 0;
    auto best_cost = cost_func(map_func(signal_first, signal_last, specs));

    for (auto delay = 1u; delay <= samples_per_bit; ++delay) {
        auto cost = cost_func(map_func(signal_first+delay, signal_last, specs));

        if (cost < best_cost) {
            best_cost = cost;
            best_delay = delay;
        }
    }

    return best_delay;
}


std::vector<BitType> decode(CircularIter signal_first,
                            CircularIter signal_last,
                            SignalMapFunc map_func,
                            SignalReduceFunc reduce_func,
                            const CarrierSpecs& specs,
                            bool invert)
{
    auto result = reduce_func(map_func(signal_first, signal_last, specs));

    if (invert) {
        std::for_each(result.begin(), result.end(), [](auto& bit) {
            bit = 1 - bit;
        });
    }

    return result;
}



void decode_loop(boost::circular_buffer<float>& raw_buffer,
                 boost::circular_buffer<BitType>& bit_buffer,
                 const CarrierSpecs& specs,
                 const PacketMetadata& packet_meta,
                 SignalMapFunc map_func,
                 SignalReduceFunc reduce_func,
                 SignalCostFunc cost_func,
                 std::mutex& raw_mutex,
                 std::mutex& bit_mutex)
{
    const auto samples_per_bit = specs.samples_per_bit;
    const auto signal_probe_size = specs.bits_per_prefix * samples_per_bit / 2;
    const auto samples_to_consume =
        (packet_meta.bits_per_packet() + specs.bits_per_prefix)
        * samples_per_bit;
    const auto max_delay = samples_per_bit;
    const auto min_raw_buffer_size = samples_to_consume + max_delay;

    logger->info(fmt::format("start decode loop\n"));
    while (true) {
        while (thread_safe::size(raw_buffer, raw_mutex) <= min_raw_buffer_size)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        auto lock = std::lock_guard{raw_mutex};

        if (raw_buffer.size() <= min_raw_buffer_size) {
            continue;
        }

        const auto best_delay = find_best_delay(
            raw_buffer.begin(),
            raw_buffer.begin() + signal_probe_size,
            map_func,
            cost_func,
            specs);

        raw_buffer.erase_begin(best_delay);


        auto normal_match = valid_prefix(decode(
                raw_buffer.begin(),
                raw_buffer.begin() + signal_probe_size,
                map_func,
                reduce_func,
                specs,
                false));
        auto inverted_match = valid_prefix(decode(
                raw_buffer.begin(),
                raw_buffer.begin() + signal_probe_size,
                map_func,
                reduce_func,
                specs,
                true));
        auto any_match = normal_match || inverted_match;

        if (!any_match) {
            raw_buffer.erase_begin(signal_probe_size);
            continue;
        }

        auto invert = inverted_match;

        static int k = 0;
        logger->info(fmt::format("-----{}-----{}-----",
                                 k++, raw_buffer.size()));
        std::vector<BitType> data = decode(
            raw_buffer.begin(),
            raw_buffer.begin() + samples_to_consume,
            map_func,
            reduce_func,
            specs,
            invert);

        {
            auto bit_lock = std::lock_guard{bit_mutex};
            for (auto bit : data) {
                // fmt::print("{}", bit);
                bit_buffer.push_back(bit);
            }
            // fmt::print("\n");
        }

        raw_buffer.erase_begin(samples_to_consume);
    }
}


// Move raw data from the thread-safe buffer to the thread-unsafe buffer
void pour_to_raw_buffer(boost::lockfree::spsc_queue<float>& raw_queue,
                        boost::circular_buffer<float>& raw_buffer,
                        std::mutex& raw_mutex)
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


Receiver::Receiver(const CarrierSpecs& specs, const PacketMetadata& meta,
                   SignalMapFunc map_func, SignalReduceFunc reduce_func,
                   SignalCostFunc cost_func)
    : lockfree_raw_buffer_(lockfree_raw_buffer_size),
    raw_buffer_(raw_buffer_size),
    bit_buffer_(bit_buffer_size),
    msg_buffer_(msg_buffer_size),
    specs_{specs},
    meta_{meta},
    map_func_{map_func},
    reduce_func_{reduce_func},
    cost_func_{cost_func}
{
    using std::ref, std::cref;
    msg_loop_thread_ = std::thread(extract_msg_loop, ref(bit_buffer_),
                                   ref(msg_buffer_), cref(meta_), ref(bit_mutex_),
                                   ref(msg_mutex_));
    decode_loop_thread_ = std::thread(decode_loop, ref(raw_buffer_),
                                     ref(bit_buffer_),
                                     ref(specs_), ref(meta_),
                                     map_func_, reduce_func_, cost_func_,
                                     ref(raw_mutex_), ref(bit_mutex_));
    pour_thread_ = std::thread(pour_to_raw_buffer, ref(lockfree_raw_buffer_),
                               ref(raw_buffer_), ref(raw_mutex_));
    launch_mic_loop(lockfree_raw_buffer_);
}
