#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

typedef int8_t  i8;
typedef uint8_t u8;

typedef int16_t  i16;
typedef uint16_t u16;

typedef int32_t  i32;
typedef uint32_t u32;

typedef uint8_t bool;


#define null (0)

#define true (1)
#define false (0)

// fixed point
typedef int32_t fp32;

#define fp32_from(R) (fp32)(R * (1 << 16))
#define fp32_toi(F) (u32)((F >> 16))

fp32 fp_mul(const fp32 a, const fp32 b);
fp32 fp_sin(const u32 degrees);
fp32 fp_cos(const u32 degrees);

#endif // !TYPES_H
