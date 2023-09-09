#ifndef TEXT_H
#define TEXT_H

#include "types.h"

void text_print(const char* text, const u32 length);
void text_printi(i32 n);
void text_rasterize(u8* buffer, const u16 y);

#endif // !TEXT_H
