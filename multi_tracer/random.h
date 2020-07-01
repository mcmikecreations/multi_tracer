#pragma once

#include <cstdint>

void srandom(std::uint32_t seed);
std::uint32_t random_u32();
float random_f32();
float random_f32(float max);
float random_f32(float min, float max);