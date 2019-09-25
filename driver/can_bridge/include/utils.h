/*
 * @Description: the bit read and write function
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-26 11:01:52
 * @LastEditTime: 2019-03-26 11:15:23
 */

#ifndef CANLIB_BITBUFFER_H
#define CANLIB_BITBUFFER_H

#include <cstdint>
#include <stdint-gcc.h>

namespace USB2CAN
{

#define GRP_OFFSET(x) ((x)&0x07) // 获取整数x的最低3bit位  // &-按位与运算,两边均换算成二进制bit格式后再进行运算
#define BYT_OFFSET(x) ((x) >> 3) // can数据Data长度为8,此处相当于x/8并取整,以得到其在can消息字段中的位置[0-7]

inline int readBit(uint8_t *buf, int pos)
{
    int index = BYT_OFFSET(pos);
    int grp = GRP_OFFSET(pos);
    return (buf[index] >> grp) & 1;
}

inline int readAsInt(uint8_t *buf, int offset, int length)
{
    int ret = 0;
    for (int i = length - 1; i >= 0; --i)
    {
        ret = ret * 2 + readBit(buf, offset + i);
    }
    return ret;
}

static inline void setBit(uint8_t *buf, int pos)
{
    int index = BYT_OFFSET(pos);
    int grp = GRP_OFFSET(pos);
    buf[index] |= (1 << grp);
}

static inline void clearBit(uint8_t *buf, int pos)
{
    int index = BYT_OFFSET(pos);
    int grp = GRP_OFFSET(pos);
    uint8_t v = ~(1u << grp);
    buf[index] &= v;
}

inline void writeBit(uint8_t *buf, int pos, bool b)
{
    if (b)
        setBit(buf, pos);
    else
        clearBit(buf, pos);
}

inline void writeInt(uint8_t *buf, int offset, int length, int v) // 按位写二进制数据 // 把v转换成二进制数写到buf的offset到(offset+length)之间
{
    for (int i = 0; i < length; ++i)
    {
        if (v & 1)
        {
            setBit(buf, offset + i);
        }
        else
        {
            clearBit(buf, offset + i);
        }
        v >>= 1;
    }
}
} // namespace USB2CAN

#endif // !CANLIB_BITBUFFER_H