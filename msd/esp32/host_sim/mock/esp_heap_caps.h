#pragma once
#include <cstddef>
#define MALLOC_CAP_8BIT 0
inline size_t heap_caps_get_largest_free_block(unsigned) { return 110000; }
