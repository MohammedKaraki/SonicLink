#ifndef SONICLINK_PACKET_H
#define SONICLINK_PACKET_H

#include <vector>
#include <boost/crc.hpp>
#include <fmt/format.h>

#include "core.h"


using Byte = unsigned char;

struct ErrorCheckedMessage;

class Bytes : public std::vector<Byte> {
private:
    void load(const std::string& str)
    {
        this->clear();
        for (auto c : str) {
            this->push_back(Byte(c));
        }
    }

    void load(uint16_t n)
    {
        auto high = (n >> 8) & 0xFF;
        auto low = n & 0xFF;

        this->clear();
        this->reserve(2);

        (*this).push_back(high);
        (*this).push_back(low);
    }

    void load(const ErrorCheckedMessage& msg);
public:
    Bytes() = default;

    Bytes(const std::string& str)
    {
        load(str);
    }

    Bytes(uint16_t n)
    {
        load(n);
    }

    Bytes(const ErrorCheckedMessage& msg)
    {
        load(msg);
    }

    Bytes& operator=(const std::string& str)
    {
        load(str);
        return *this;
    }

    Bytes& operator=(uint16_t n)
    {
        load(n);
        return *this;
    }

    Bytes& operator=(const ErrorCheckedMessage& msg)
    {
        load(msg);
        return *this;
    }

    operator std::string() const
    {
        auto result = std::string{};
        for (auto byte : *this) {
            result += char(byte);
        }
        return result;
    }

    operator uint16_t() const
    {
        auto high = (*this)[0];
        auto low = (*this)[1];
        return (high << 8) + low;
    }

    operator ErrorCheckedMessage() const;
};

struct ErrorCheckedMessage {
    uint16_t id;
    Bytes data;
    uint16_t checksum;

    uint16_t calc_checksum() const
    {
        auto crc = boost::crc_16_type{};
        crc.process_bytes(&id, sizeof(id));
        crc.process_bytes(data.data(), data.size());

        return crc.checksum();
    }

    bool valid() const
    {
        return checksum == calc_checksum();
    }
};

std::ostream& operator<<(std::ostream& out, const ErrorCheckedMessage& msg);


auto bits_to_msg_body(const std::vector<BitType>& bits) -> ErrorCheckedMessage;
auto msg_body_to_bits(const ErrorCheckedMessage& msg_body)
    -> std::vector<BitType>;
auto create_msg_body(uint16_t id, Bytes& data) -> ErrorCheckedMessage;


auto prepare_packet_bodies(const std::string& str, std::size_t packet_body_size)
    -> std::vector<ErrorCheckedMessage>;

#endif
