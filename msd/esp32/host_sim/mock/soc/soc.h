#pragma once
#include <cstdint>
uint32_t mock_reg_read(uint32_t addr);
void mock_reg_write(uint32_t addr, uint32_t v);
#define REG_READ(a) mock_reg_read(a)
#define REG_WRITE(a, v) mock_reg_write((a), (v))
