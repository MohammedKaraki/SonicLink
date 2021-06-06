#include <fmt/format.h>
#include <fmt/ostream.h>

#include <vector>
#include <cassert>
#include <cctype>
#include "binary.h"
#include "core.h"


Bit::Bit(std::vector<Byte>& bytes, std::size_t bit_index)
    : bytes_(bytes), bit_index_{bit_index}
{ }

Bit& Bit::operator=(BitVal bit_val)
{
    auto byte_index = bit_index_ / 8;
    auto bit_shift = 7 - bit_index_ % 8;

    auto& target_byte = bytes_[byte_index];

    auto n = bit_val == BitVal::One ? 1 : 0;
    target_byte = (target_byte & ~(1 << bit_shift)) | (n << bit_shift);

    return *this;
}

Bit::operator BitVal()
{
    auto byte_index = bit_index_ / 8;
    auto bit_shift = 7 - bit_index_ % 8;

    auto n = (bytes_[byte_index] >> bit_shift) & 1;
    return n == 1 ? BitVal::One : BitVal::Zero;
}

ConstBit::ConstBit(const std::vector<Byte>& bytes, std::size_t bit_index)
    : bytes_(bytes), bit_index_{bit_index}
{ }


ConstBit::operator BitVal() const
{
    auto byte_index = bit_index_ / 8;
    auto bit_shift = 7 - bit_index_ % 8;

    auto n = (bytes_[byte_index] >> bit_shift) & 1;
    return n == 1 ? BitVal::One : BitVal::Zero;
}


Byte& Binary::byte(std::size_t index)
{
    return bytes_[index];
}

const Byte& Binary::byte(std::size_t index) const
{
    return bytes_[index];
}

Bit Binary::bit(std::size_t index)
{
    return Bit(bytes_, index);
}

ConstBit Binary::bit(std::size_t index) const
{
    return ConstBit(bytes_, index);
}

std::ostream& operator<<(std::ostream& out, const Binary& binary)
{
    out << '\'';
    for (auto i = 0u; i < binary.byte_count(); ++i)
    {
        for (auto j = 0u; j < 8; ++j) {
            auto bit_index = 8*i + j;
            out << (binary.bit(bit_index)==BitVal::One ? '1' : '0');
        }
        out << '(' << static_cast<int>(binary.byte(i));

        if (std::isprint(char(binary.byte(i)))) {
            out << ',' << char(binary.byte(i));
        }

        out << ')';

        if (i+1 % 8 == 0) {
            out << '\n';
        }
    }
    return out << '\'';
}

auto vectorize(const Binary& bin) -> std::vector<BitType>
{
    auto result = std::vector<BitType>{};
    result.reserve(bin.bit_count());

    for (auto i = 0u; i < bin.bit_count(); ++i) {
        result.emplace_back(bin.bit(i) == BitVal::Zero ? 0 : 1);
    }

    return result;
}

auto binarize_digits(const std::string& digits) -> Binary
{
    assert(digits.size() % 8 == 0);

    auto result = Binary{};
    result.resize(digits.size() / 8);

    for (auto i = 0u; i < digits.size(); ++i) {
        auto digit = digits[i];
        if (digit == '0') {
            result.bit(i) = BitVal::Zero;
        }
        else if (digit == '1') {
            result.bit(i) = BitVal::One;
        }
        else {
            assert(false);
        }
    }

    return result;
}

auto binarize(const ErrorCheckedMessage& msg) -> Binary
{
    auto result = Binary{};

    result.resize(msg.data.size() + 4);
    result.byte(0) = msg.id >> 8;
    result.byte(1) = msg.id & 0xFF;
    result.byte(2) = msg.checksum >> 8;
    result.byte(3) = msg.checksum & 0xFF;
    for (auto i = 0u; i < msg.data.size(); ++i) {
        result.byte(4 + i) = msg.data[i];
    }

    return result;
}

auto msg_from_binary(const Binary& bin) -> ErrorCheckedMessage
{
    auto result = ErrorCheckedMessage{};

    result.id = (bin.byte(0) << 8) + bin.byte(1);
    result.checksum = (bin.byte(2) << 8) + bin.byte(3);

    for (auto i = 4u; i < bin.byte_count(); ++i) {
        result.data.push_back(bin.byte(i));
    }

    return result;
}

Binary operator+(const Binary& l, const Binary& r)
{
    auto result = Binary{};
    result.resize(l.byte_count() + r.byte_count());

    for (auto i = 0u; i < l.byte_count(); ++i) {
        result.byte(i) = l.byte(i);
    }
    for (auto i = 0u; i < r.byte_count(); ++i) {
        result.byte(i + l.byte_count()) = r.byte(i);
    }

    return result;
}

auto stringify_bit_vec(const std::vector<BitType>& bit_vec) -> std::string
{
    auto result = std::string{};
    for (auto bit : bit_vec) {
        result.push_back(bit == 0 ? '0' : '1');
    }
    return result;
}
