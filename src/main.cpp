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
#include <cstdio>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <map>
#include "core.h"
#include "recv.h"
#include "send.h"
#include "packet.h"
#include "binary.h"

#include <fmt/ostream.h>
#include <fmt/format.h>
#include <fmt/color.h>

#include <spdlog/spdlog.h>


int last_printed = 0;
std::map<int, Bytes> recv_msgs;

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


// Phase-Shift Keying
namespace psk {
    SignalMapFunc map = [](CircularIter signal_first,
                           CircularIter signal_last,
                           const CarrierSpecs& specs)
    {
        assert(std::distance(signal_first, signal_last) > 0);
        const auto total_samples = static_cast<std::size_t>(
            std::distance(signal_first, signal_last));
        const auto samples_per_bit = specs.samples_per_bit;
        const auto num_bits = total_samples / samples_per_bit;

        assert(samples_per_bit < total_samples);

        std::vector<float> result;
        result.reserve(num_bits); // first bit is ambiguous in diff method

        auto it = signal_first;

        float omega = 2.0 * std::numbers::pi * specs.frequency;
        for (auto j = 0u; j < num_bits; ++j) {

            float tmp = 0.0f;
            for (auto i = 0u; i < samples_per_bit; ++i) {
                tmp += (*it) * std::sin(omega * float(i));

                ++it;
            }

            tmp /= float(samples_per_bit);

            result.emplace_back(tmp);
        }

        return result;
    };

    SignalReduceFunc reduce = [](const std::vector<float>& mapped)
    {
        std::vector<BitType> result;
        result.reserve(mapped.size());
        for (auto f : mapped) {
            result.emplace_back(f > 0.0f ? 1 : 0);
        }

        return result;
    };

    SignalCostFunc cost = [](const std::vector<float>& mapped)
    {
        auto result = 0.0f;

        auto cost_func = [](float x) {
            return 1.0 / (x*x + 1e-100);
        };

        for (auto d : mapped) {
            result += cost_func(d);
        }

        result /= mapped.size();

        return result;
    };

    BitsEncodeFunc encode = [](const std::vector<BitType>& bits,
                               const CarrierSpecs& specs)
    {
        using std::numbers::pi, std::sin;

        auto result = std::vector<float>{};
        result.reserve(bits.size() * specs.samples_per_bit);

        for (auto bit : bits) {
            auto sign = (bit==1) ? 1.0f : -1.0f;

            for (auto i = 0u; i < specs.samples_per_bit; ++i) {
                result.push_back(
                    sign * sin(2.0 * pi * specs.frequency * double(i))
                    );
            }
        }

        return result;
    };

}


int main(int argc, char *[])
{
    if (std::freopen("/dev/null", "w", stderr)) {
        std::fclose(stderr);
    }

    std::cin >> std::noskipws;
    std::cout << std::noskipws;
    auto msg_str = std::string{};
    if (argc == 2) {
      for (char c; std::cin.read(&c, 1); ) {
        msg_str += c;
      }
    }
    fmt::print("********* START *********\n");


    auto packet_bodies = prepare_packet_bodies(msg_str, bytes_per_msg_data);


    const auto samples_per_cycle = 12;
    const auto samples_per_bit = 4 * samples_per_cycle;
    const auto frequency = 1.0f / (samples_per_cycle);

    auto specs = CarrierSpecs {
        .samples_per_bit = samples_per_bit,
        .bits_per_prefix = strlen(PACKET_PREFIX),
        .bits_per_suffix = strlen(PACKET_SUFFIX),
        .frequency = frequency
    };

    auto meta = PacketMetadata {
        .head_signature = bits_from_str(HEAD_SIGNATURE),
        .tail_signature = bits_from_str(TAIL_SIGNATURE),
        .bits_per_body = 8 * bytes_per_msg_body
    };


    Receiver receiver{specs, meta, psk::map, psk::reduce, psk::cost};
    Sender sender{specs, psk::encode};

    auto sender_loop = [&msg_str, &packet_bodies](Sender& sender) {
      if (msg_str.empty()) {
        return;
      }
        std::vector<std::vector<BitType>> digit_vec_packets;
        for (auto& packet_body : packet_bodies) {
            digit_vec_packets.emplace_back(
                vectorize(
                    binarize_digits(PACKET_PREFIX)
                    + binarize_digits(HEAD_SIGNATURE)
                    + binarize(packet_body)
                    + binarize_digits(TAIL_SIGNATURE)
                    + binarize_digits(PACKET_SUFFIX)
                    )
                );
        }

        while (true) {
            for (auto& digit_vec_packet : digit_vec_packets) {
                sender.encode_and_queue(digit_vec_packet);
            }

            std::this_thread::sleep_for(std::chrono::seconds{5});
        }
    };

    auto sender_loop_thread = std::thread(sender_loop, std::ref(sender));

    auto receiver_loop = [](Receiver& receiver) {
        // const auto body = std::string{PACKET_BODY};

        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});

            auto msg_lock = std::lock_guard{receiver.msg_mutex()};
            auto& msg_buffer = receiver.msg_buffer();

            while (!msg_buffer.empty()) {
                auto msg_vec = *msg_buffer.begin();
                msg_buffer.pop_front();

                auto msg_bin = binarize_digits(stringify_bit_vec(msg_vec));
                auto msg = msg_from_binary(msg_bin);

                fmt::print("{}\n", std::string(msg.data));

                if (msg.valid()) {
                    if (msg.id > last_printed) {
                        if (!recv_msgs.contains(msg.id)) {
                            recv_msgs[msg.id] = msg.data;
                        }
                    }
                }

                while (recv_msgs.contains(last_printed+1)) {
                    ++last_printed;
                    // fmt::print("{}", std::string(recv_msgs[last_printed]));
                    fflush(stdout);
                }
            }
        }
    };
    auto receiver_loop_thread = std::thread(receiver_loop, std::ref(receiver));

    sender_loop_thread.join();
    receiver_loop_thread.join();
}

