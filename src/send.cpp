#include <cmath>
#include <numbers>
#include <mutex>
#include <cstddef>
#include <boost/circular_buffer.hpp>
#include <portaudio.h>
#include "core.h"
#include "send.h"

#include <fmt/format.h>

struct PlaybackUserdata {
    boost::circular_buffer<float> *raw_buffer_ptr;
    const CarrierSpecs *carrier_specs_ptr;
    std::mutex *raw_mutex_ptr;
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
    // const auto& specs = *user_data.carrier_specs_ptr;
    auto& raw_mutex = *user_data.raw_mutex_ptr;

    auto out_ptr = static_cast<float *>(output_buffer);

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
                *out_ptr++ = 0.0;
                // *out_ptr++ = sin(2.0 * std::numbers::pi *
                //                       specs.frequency / 5 * k++);
            }
        }
    }

    return paContinue;
}


void start_broadcasting(boost::circular_buffer<float>& raw_buffer,
                        const CarrierSpecs& carrier_specs,
                        std::mutex& raw_mutex)
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
        .carrier_specs_ptr = &carrier_specs,
        .raw_mutex_ptr = &raw_mutex
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
void encode_and_dump(const std::vector<BitType>& bits,
                     BitsEncodeFunc encode_func,
                     const CarrierSpecs& specs,
                     boost::circular_buffer<float>& raw_buffer,
                     std::mutex& raw_mutex)
{
    auto raw_result = encode_func(bits, specs);

    auto lock = std::lock_guard{raw_mutex};
    for (auto f : raw_result) {
        raw_buffer.push_back(f);
    }
}

Sender::Sender(const CarrierSpecs& specs, BitsEncodeFunc encode_func)
    : raw_buffer_(raw_buffer_size), specs_{specs}, encode_func_{encode_func}
{
    start_broadcasting(raw_buffer_, specs_, raw_mutex_);
}

void Sender::encode_and_queue(const std::vector<BitType>& bits)
{
    encode_and_dump(bits, encode_func_, specs_, raw_buffer_, raw_mutex_);
}
