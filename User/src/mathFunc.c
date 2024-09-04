#include "mathFunc.h"
#include "main.h"
#include "math.h"
/**
 * @brief 将p1和p2进行互换，在地址上操作
 * 
 * @param p1 
 * @param p2 
 */
void ChangeDataByte(uint8_t *p1, uint8_t *p2)
{
    uint8_t t;
    t = *p1;
    *p1 = *p2;
    *p2 = t;
}

/**
 * @brief Get the s32 from buffer object 将4个8位合成32位 同时改变index
 * 
 * @param buffer 
 * @param index 
 * @return int32_t 
 */
int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int32_t res = (((uint32_t)buffer[*index]) << 24) |
                  (((uint32_t)buffer[*index + 1]) << 16) |
                  (((uint32_t)buffer[*index + 2]) << 8) |
                  (((uint32_t)buffer[*index + 3]));
    *index += 4;
    return res;
}

/**
 * @brief Get the s16 from buffer object 将两个8位合成16位 同时改变index
 * 
 * @param buffer 
 * @param index 
 * @return int16_t 
 */
int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int16_t res = (((uint32_t)buffer[*index]) << 8) |
                  (((uint32_t)buffer[*index + 1]));
    *index += 2;
    return res;
}

/**
 * @brief 将32位转为float 同时改变index 除以scale
 * 
 * @param buffer 
 * @param scale 
 * @param index 
 * @return float 
 */
float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s32_from_buffer(buffer, index) / scale;
}

/**
 * @brief 将16位转为float
 * 
 * @param buffer 
 * @param scale 
 * @param index 
 * @return float 
 */
float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s16_from_buffer(buffer, index) / scale;
}

/**
 * @brief 将一个32位转为4个8位
 * 
 * @param buffer 
 * @param source 
 * @param index 
 */
void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index)
{
    buffer[(*index)++] = source >> 24;
    buffer[(*index)++] = source >> 16;
    buffer[(*index)++] = source >> 8;
    buffer[(*index)++] = source;
}
/**
 * @brief 将一个16位转为2个8位
 * 
 * @param buffer 
 * @param source 
 * @param index 
 */
void buffer_append_int16(uint8_t *buffer, int16_t source, int32_t *index)
{
    buffer[(*index)++] = source;
    buffer[(*index)++] = source >> 8;//小端模式
}

/**
 * @brief 将两个float转化为double
 * 
 * @param n1 
 * @param n2 
 * @return double 
 */
double cvtFloat2Double(float n1, float n2)
{
	struct {float n1;float n2;} s;
	s.n1 = n1;
	s.n2 = n2;
	return *((double*)&s);
}

