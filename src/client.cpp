#include "sonic_link/core.hpp"
#include <cstdint>
#include <vector>
#include <utility>
#include <fmt/format.h>
#include <cstddef>
#include <string_view>
#include <span>


int main()
{
  const auto sample_rate = 44100u;

  // A single bit is carried by multiple periods of oscillation. This increases
  // the quality of the signal, but obviously at the cost of reduced throughput.
  const auto cycles_per_bit_server = 3u;
  const auto cycles_per_bit_client = 3u;

  // Next two constants should not be equal, in order to avoid
  // interference between wave signals.
  const auto samples_per_cycle_server = 12u;
  const auto samples_per_cycle_client = 8u;


  // auto client = SonicLink::Client{sample_rate,
  //                                 cycles_per_bit_server,
  //                                 cycles_per_bit_client,
  //                                 samples_per_cycle_server,
  //                                 samples_per_cycle_client};

  const auto message = std::string_view{
    "This is a message.\n"
    "It is not just any message. It is a special sort of message.\n"
    "Air molecules are responsible for the transmission of this message...\n"
    "...by dancing with an appropriate pattern.\n"
    "So there only need be speakers and microphones.\n"
    "And air molecules!\n"
    "Checksums are going to guarantee the integrity of this message.\n"
    "Acknowledgement packets are going to guarantee\n"
    "the arrival of this message.\n"
    "This message is going to be split and sent as separate packets.\n"
    "And although the packets are read sequentially through the interface\n"
    "at the server side,\n"
    "they can be delivered out of order behind the scenes.\n"
    "Thus, a dropped packet is not going to block the traffic.\n"
    "And this was the entire message.\n"
    "Which was a slightly verbose message.\n"
    "And not a very eloquent message.\n"
    "But this is a wonderful message.\n"
    "And this is the end of the message.\n"};

  fmt::print("{}\n{}\n", message, message.size());

  const auto bytes = std::as_bytes(std::span(message.begin(), message.end()));
  // client.send_bytes(bytes);
  //
  // client.wait();
}
