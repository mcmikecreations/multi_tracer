#include "random.h"

#include <algorithm>
#include <cstdint>
#include <limits>

static std::uint32_t x = 123456789, y = 362436069, z = 521288629;

std::uint32_t xorshf96(void) {          //period 2^96-1
    std::uint32_t t;
    x ^= x << 16;
    x ^= x >> 5;
    x ^= x << 1;

    t = x;
    x = y;
    y = z;
    z = t ^ x ^ y;

    return z;
}

void srandom(std::uint32_t seed)
{
    x = 123456789 ^ ((seed & 0xFFF00000) ^ 0x55555555);
    y = 362436069 ^ ((seed & 0x00FFF000) ^ 0x55555555);
    z = 521288629 ^ ((seed & 0x00000FFF) ^ 0x55555555);
}
std::uint32_t random_u32()
{
    return xorshf96();
}
float random_f32()
{
    return std::clamp<float>(
            static_cast<float>(random_u32()) 
            / 
            static_cast<float>(std::numeric_limits<std::uint32_t>::max()),
        0.0f, 1.0f);
}
float random_f32(float max)
{
    return std::clamp<float>(
        static_cast<float>(random_u32())
        /
        static_cast<float>(std::numeric_limits<std::uint32_t>::max() / max),
        0.0f, max);
}
float random_f32(float min, float max)
{
    return std::clamp<float>(
        min 
        + 
        static_cast<float>(random_u32())
        /
        static_cast<float>(std::numeric_limits<std::uint32_t>::max() / (max - min)),
        min, max);
}