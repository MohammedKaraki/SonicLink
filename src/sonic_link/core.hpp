#include <cstddef>
#include <utility>
#include <span>

namespace SonicLink {
  class Server {
  public:
    Server(unsigned int sample_rate,
           unsigned int cycles_per_bit_server,
           unsigned int cycles_per_bit_client,
           unsigned int samples_per_bit_server,
           unsigned int samples_per_bit_client)
      : m_sample_rate{sample_rate},
        m_cycles_per_bit_server{cycles_per_bit_server},
        m_cycles_per_bit_client{cycles_per_bit_client},
        m_samples_per_bit_server{samples_per_bit_server},
        m_samples_per_bit_client{samples_per_bit_client}
    { }

    operator bool();

    auto read_bytes(auto begin, auto end) -> std::pair<decltype(begin),
                                                       decltype(end)>;

  private:
    const unsigned int m_sample_rate;
    const unsigned int m_cycles_per_bit_server;
    const unsigned int m_cycles_per_bit_client;
    const unsigned int m_samples_per_bit_server;
    const unsigned int m_samples_per_bit_client;
  };

  class Client {
  public:
    Client(unsigned int sample_rate,
           unsigned int cycles_per_bit_server,
           unsigned int cycles_per_bit_client,
           unsigned int samples_per_bit_server,
           unsigned int samples_per_bit_client)
      : m_sample_rate{sample_rate},
        m_cycles_per_bit_server{cycles_per_bit_server},
        m_cycles_per_bit_client{cycles_per_bit_client},
        m_samples_per_bit_server{samples_per_bit_server},
        m_samples_per_bit_client{samples_per_bit_client}
    { }

    void send_bytes(std::span<const std::byte> bytes);
    void wait();

  private:
    const unsigned int m_sample_rate;
    const unsigned int m_cycles_per_bit_server;
    const unsigned int m_cycles_per_bit_client;
    const unsigned int m_samples_per_bit_server;
    const unsigned int m_samples_per_bit_client;
  };

} // namespace SonicLink
