#include "sonic_link/core.hpp"
#include <cstdint>
#include <vector>
#include <utility>
#include <cstddef>
#include <fmt/format.h>


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


  auto server = SonicLink::Server{sample_rate,
                                  cycles_per_bit_server,
                                  cycles_per_bit_client,
                                  samples_per_cycle_server,
                                  samples_per_cycle_client};

  auto buffer = std::vector<std::byte>(1024);
  while (server) {
    const auto [begin, end] = server.read_bytes(buffer.begin(), buffer.end());

    for (auto it = begin; it != end; ++it) {
      fmt::print("{}", static_cast<char>(*it));
    }
  }
}
