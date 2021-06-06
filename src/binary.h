#ifndef SONICLINK_BINARY_H
#define SONICLINK_BINARY_H

#include <vector>
#include <iostream>
#include "core.h"
#include "packet.h"


using Byte = unsigned char;

enum class BitVal : unsigned char { Zero = 0, One = 1 };

class Bit {
public:
    Bit(std::vector<Byte>& bytes, std::size_t bit_index);
    Bit& operator=(BitVal bit_val);
    operator BitVal();

private:
    std::vector<Byte>& bytes_;
    std::size_t bit_index_;
};

class ConstBit {
public:
    ConstBit(const std::vector<Byte>& bytes, std::size_t bit_index);
    operator BitVal() const;

private:
    const std::vector<Byte>& bytes_;
    std::size_t bit_index_;
};


class Binary {
public:
    Byte& byte(std::size_t index);
    const Byte& byte(std::size_t index) const;
    Bit bit(std::size_t index);
    ConstBit bit(std::size_t index) const;

    std::size_t byte_count() const { return bytes_.size(); }
    std::size_t bit_count() const { return 8 * byte_count(); }
    void resize(std::size_t byte_count) { bytes_.resize(byte_count); }
private:
    std::vector<Byte> bytes_;
};

std::ostream& operator<<(std::ostream& out, const Binary& binary);

Binary operator+(const Binary& l, const Binary& r);

auto vectorize(const Binary& bin) -> std::vector<BitType>;
auto binarize_digits(const std::string& digits) -> Binary;
auto binarize(const ErrorCheckedMessage& msg) -> Binary;
auto stringify_bit_vec(const std::vector<BitType>& bit_vec) -> std::string;
auto msg_from_binary(const Binary& bin) -> ErrorCheckedMessage;

#endif
