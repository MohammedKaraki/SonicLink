#include <cmath>
#include <numbers>
#include <thread>
#include <mutex>
#include <chrono>
#include <tuple>
#include <string>
#include <cstddef>
#include <stdexcept>
#include <boost/circular_buffer.hpp>
#include <portaudio.h>
#include <cstring>
#include <iostream>
#include "core.h"

#define FMT_HEADER_ONLY
#include <fmt/format.h>

auto raw_mutex = std::mutex{};

struct PlaybackUserdata {
    boost::circular_buffer<float> *raw_buffer_ptr;
    const CarrierSpecs *carrier_specs_ptr;
};

static int playback_callback(const void * /* input_buffer */,
                             void *output_buffer,
                             unsigned long /* frame_count */,
                             const PaStreamCallbackTimeInfo * /* time_info */,
                             PaStreamCallbackFlags /* status_flags */,
                             void *user_data_ptr)
{
    using std::numbers::pi, std::sin;

    auto& user_data = *static_cast<PlaybackUserdata *>(user_data_ptr);
    auto& raw_buffer = *user_data.raw_buffer_ptr;
    const auto& specs = *user_data.carrier_specs_ptr;

    auto out_ptr = static_cast<float *>(output_buffer);
    static auto k = 0u;

    if (output_buffer != nullptr)
    {
        auto lock = std::lock_guard{raw_mutex};
        for (auto i = 0u; i < frames_per_buffer; ++i)
        {
            if (!raw_buffer.empty()) {
                *out_ptr++ = *raw_buffer.begin();
                raw_buffer.pop_front();
            }
            else {

                // No signal here, play whatever you like:
                *out_ptr++ = sin(2.0 * std::numbers::pi *
                                      specs.frequency / 5 * k++);
            }
        }
    }

    return paContinue; // Forever!
}


void start_broadcasting(boost::circular_buffer<float>& raw_buffer,
                        const CarrierSpecs& carrier_specs)
{
    PaStream *output_stream;
    PaStreamParameters output_params;
    PaError error;

    error = Pa_Initialize();
    if (error != paNoError) {
        Pa_Terminate();
        fmt::print("{}: {}\n", error, Pa_GetErrorText(error));
        exit(1);
    }

    output_params.device = Pa_GetDefaultOutputDevice();
    if (output_params.device == paNoDevice) {
        fmt::print("Error: No default input device.\n");
        exit(1);
    }

    output_params.channelCount = 1;
    output_params.sampleFormat = paFloat32;
    output_params.suggestedLatency =
        Pa_GetDeviceInfo(output_params.device)->defaultLowOutputLatency;
    output_params.hostApiSpecificStreamInfo = nullptr;


    static auto user_data =  PlaybackUserdata {
        .raw_buffer_ptr = &raw_buffer,
        .carrier_specs_ptr = &carrier_specs
    };

    error = Pa_OpenStream(&output_stream,
                          nullptr /* input params */,
                          &output_params,
                          sample_rate,
                          frames_per_buffer,
                          paClipOff /* stream flags */,
                          playback_callback,
                          &user_data /* user data */);
    if (error != paNoError) {
        fmt::print("{}: {}\n", error, Pa_GetErrorText(error));
        exit(1);
    }

    error = Pa_StartStream(output_stream);
    if (error != paNoError) {
        fmt::print("{}: {}\n", error, Pa_GetErrorText(error));
        exit(1);
    }
}

// Encode the given bit sequence and load the result to the raw_buffer
void bits_to_raw_buffer(const std::vector<BitType>& bits,
                        boost::circular_buffer<float>& raw_buffer,
                        const CarrierSpecs& specs)
{
    using std::numbers::pi, std::sin;
    const auto N = specs.samples_per_bit;

    auto lock = std::lock_guard{raw_mutex};
    for (auto bit_it = bits.begin()+1; bit_it != bits.end(); ++bit_it) {
        auto sign = *bit_it==1 ? +1.0 : -1.0;

        for (auto i = 0u; i < N; ++i) {
            raw_buffer.push_back(
                sign * sin(2.0 * pi * specs.frequency * double(i))
                );
        }
    }
}


int main()
{
    auto carrier_specs = CarrierSpecs {
        .samples_per_bit = samples_per_bit,
        .bits_per_prefix = strlen(PACKET_PREFIX),
        .bits_per_suffix = strlen(PACKET_SUFFIX),
        .frequency = frequency
    };

    auto raw_buffer = boost::circular_buffer<float>(1ul << 20);


    auto packet = bits_from_str(
        PACKET_PREFIX
        HEAD_SIGNATURE
        PACKET_BODY
        TAIL_SIGNATURE
        PACKET_SUFFIX
        );

    for (auto b : packet) {
        fmt::print(stderr, "{} ", b);
    }

    start_broadcasting(raw_buffer, carrier_specs);

    while (std::getchar() != EOF) {
        bits_to_raw_buffer(packet, raw_buffer, carrier_specs);
    }
}
