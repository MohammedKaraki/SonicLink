#include <vector>
#include <cstdint>
#include <string>
#include <algorithm>
#include <boost/crc.hpp>
#include <fmt/format.h>
#include <fmt/color.h>

#include "core.h"
#include "packet.h"


auto create_msg_body(uint16_t id, Bytes& data) -> ErrorCheckedMessage
{
    auto result = ErrorCheckedMessage {
        .id = id,
        .data = data,
        .checksum = 0
    };
    result.checksum = result.calc_checksum();

    return result;
}

auto byte_to_bits(Byte byte)
{
    const auto N = 8;

    auto result = std::vector<BitType>{};
    result.reserve(N);
    for (auto i = int{N}-1; i >= 0 ; --i) {
        result.push_back(BitType((byte >> i) & 1));
    }

    return result;
}

auto bits_to_byte(const std::vector<BitType>& bits)
{
    const auto N = 8;
    assert(bits.size() == N);

    auto result = Byte{0};
    for (auto i = 0; i < N; ++i) {
        auto bit = bits[N - i - 1];
        assert(bit == 0 || bit == 1);
        result += bit << i;
    }

    return result;
}

auto bytes_to_bits(const Bytes& bytes)
{
    const auto N = 8;
    auto result = std::vector<BitType>{};
    result.reserve(N * bytes.size());

    for (auto byte : bytes) {
        auto tmp = byte_to_bits(byte);
        for (auto bit : tmp) {
            result.emplace_back(bit);
        }
    }

    return result;
}

auto bits_to_bytes(const std::vector<BitType>& bits) -> Bytes
{
    const auto N = 8;
    assert(bits.size()%N == 0);

    auto result = Bytes{};
    result.reserve(bits.size() / N);

    for (auto it = bits.begin(); it != bits.end(); it += N) {
        auto eight_bits = std::vector<BitType>(N);
        std::copy_n(it, N, eight_bits.begin());
        result.emplace_back(bits_to_byte(eight_bits));
    }

    return result;
}

auto msg_body_to_bits(const ErrorCheckedMessage& msg_body) -> std::vector<BitType>
{
    auto result = std::vector<BitType>{};

    auto pour = [](const auto& source, auto& dist) {
        for (auto elem : source) {
            dist.emplace_back(elem);
        }
    };

    auto id_high = (msg_body.id >> 8) & 0xFF;
    auto id_low = msg_body.id & 0xFF;

    auto checksum_high = (msg_body.checksum >> 8) & 0xFF;
    auto checksum_low = msg_body.checksum & 0xFF;


    pour(byte_to_bits(id_high), result);
    pour(byte_to_bits(id_low), result);
    pour(bytes_to_bits(msg_body.data), result);
    pour(byte_to_bits(checksum_high), result);
    pour(byte_to_bits(checksum_low), result);

    return result;
}

auto bits_to_msg_body(const std::vector<BitType>& bits) -> ErrorCheckedMessage
{
    auto result = ErrorCheckedMessage{};

    const auto N = 8;

    auto all_packet_bytes = bits_to_bytes(bits);

    auto id_high = all_packet_bytes[0];
    auto id_low = all_packet_bytes[1];
    result.id = (uint16_t(id_high) << 8) + id_low;
    fmt::print(stderr, "id_high: {}\n"
                       "id_low:  {}\n",
                       id_high,
                       id_low);

    result.data.resize(bits.size()/N - 4);
    std::copy_n(all_packet_bytes.begin() + 2,
                all_packet_bytes.size() - 2,
                result.data.begin());


    auto checksum_high = all_packet_bytes[all_packet_bytes.size() - 2];
    auto checksum_low = all_packet_bytes[all_packet_bytes.size() - 1];
    result.checksum = (uint16_t(checksum_high) << 8) + checksum_low;


    return result;
}

auto print_msg_body(const ErrorCheckedMessage& msg_body)
{
    auto data_str = std::string(msg_body.data);

    fmt::print("==========================\n");
    fmt::print("id: {}\n"
               "data: {}\n"
               "checksum: {}\n",
               msg_body.id,
               data_str,
               msg_body.checksum);
    fmt::print("==========================\n");
}

auto print_bits(const std::vector<BitType>& bits)
{
    fmt::print("==========================\n");
    for (auto bit : bits) {
        fmt::print("{}", char('0'+bit));
    }
    fmt::print("\n");
    fmt::print("==========================\n");
}


// int main()
// {
//     auto data = Bytes{};
//     data = "555";
//
//     auto body = create_msg_body(5, data);
//     auto body_bits = msg_body_to_bits(body);
//     auto body_received = bits_to_msg_body(body_bits);
//
//     print_msg_body(body);
//     print_bits(body_bits);
//     print_msg_body(body_received);
//
//     fmt::print("valid: {}\n", body_received.valid());
// }


void Bytes::load(const ErrorCheckedMessage& msg)
{
    this->clear();
    this->reserve(msg.data.size() + 4);

    auto id_high = static_cast<Byte>(msg.id >> 8);
    auto id_low = static_cast<Byte>(msg.id & 0xFF);
    this->emplace_back(id_high);
    this->emplace_back(id_low);

    for (auto& d : msg.data) {
        this->emplace_back(d);
    }

    auto checksum_high = static_cast<Byte>(msg.checksum >> 8);
    auto checksum_low = static_cast<Byte>(msg.checksum & 0xFF);
    this->emplace_back(checksum_high);
    this->emplace_back(checksum_low);
}

Bytes::operator ErrorCheckedMessage() const
{
    auto id_high = static_cast<uint16_t>((*this)[0]);
    auto id_low = static_cast<uint16_t>((*this)[1]);
    auto id = static_cast<uint16_t>((id_high << 8) + id_low);

    auto data = Bytes{};
    data.reserve(this->size() - 4);
    for (auto i = 2u; i < this->size() - 2; ++i) {
        data.push_back((*this)[i]);
    }

    auto checksum_high = static_cast<uint16_t>((*this)[this->size()-2]);
    auto checksum_low = static_cast<uint16_t>((*this)[this->size()-1]);
    auto checksum = static_cast<uint16_t>((checksum_high << 8) + checksum_low);

    return {
        .id = id,
        .data = data,
        .checksum = checksum
    };

}

auto prepare_packet_bodies(const std::string& str, std::size_t packet_body_size)
    -> std::vector<ErrorCheckedMessage>
{
    auto result = std::vector<ErrorCheckedMessage>{};

    auto id = uint16_t{0};

    auto i = std::size_t{0};
    for (; i+packet_body_size < str.size(); i += packet_body_size) {
        auto body_str = str.substr(i, packet_body_size);
        auto body_data = Bytes(body_str);

        ++id;

        result.emplace_back(create_msg_body(id, body_data));
    }

    auto body_str = str.substr(i);
    while (body_str.size() < packet_body_size) {
        body_str += ' ';
    }
    auto body_data = Bytes(body_str);

    ++id;
    result.emplace_back(create_msg_body(id, body_data));

    return result;
}

std::ostream& operator<<(std::ostream& out, const ErrorCheckedMessage& msg)
{
    auto c1 = msg.checksum;
    auto c2 = msg.calc_checksum();
    auto color = c1==c2 ? fmt::color::green : fmt::color::red;
    out << fmt::format(fmt::bg(color), "{}=={}?", c1, c2);
    out << fmt::format("id: {}, '", msg.id);

    for (auto b : msg.data) {
        out << char(b);
    }

    return out << "'";

}
