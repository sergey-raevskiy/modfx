#pragma once

/* Optimization hints. */
#define unlikely(x) __builtin_expect(x, 0)
#define likely(x)   __builtin_expect(x, 1)

/* 24-bit data types. */
typedef __uint24 uint24_t;
typedef __int24  int24_t;
