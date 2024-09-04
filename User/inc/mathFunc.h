#ifndef __MATHFUNC_H__
#define __MATHFUNC_H__

#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"

#define Pi 3.1415926f
#define ABS(x) ((x>0) ? x : (-x))
#define GetSign(x) ((x>0) ? 1 : -1)
#define PeakLimit(a,b) if(ABS(a) > ABS(b)) a = GetSign(a) * b

typedef short s16;
typedef unsigned short u16;

#define EncodeS32Data(f, buff) \
    {                          \
        *(int32_t *)buff = *f; \
    }
#define DecodeS32Data(f, buff) \
    {                          \
        *f = *(int32_t *)buff; \
    }
#define EncodeS16Data(f, buff) \
    {                          \
        *(s16 *)buff = *f;     \
    }
#define DecodeS16Data(f, buff) \
    {                          \
        *f = *(s16 *)buff;     \
    }
#define EncodeU16Data(f, buff) \
    {                          \
        *(u16 *)buff = *f;     \
    }
#define DecodeU16Data(f, buff) \
    {                          \
        *f = *(u16 *)buff;     \
    }

void ChangeDataByte(uint8_t *p1, uint8_t *p2);
float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index);
float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index);
int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index);
int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index);
void buffer_append_int16(uint8_t *buffer, int16_t source, int32_t *index);
double cvtFloat2Double(float n1, float n2);


#endif /* __MATHFUNC_H__ */


