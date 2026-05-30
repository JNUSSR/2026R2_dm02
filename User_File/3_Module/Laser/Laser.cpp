#include "3_Module/Laser/Laser.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>



int Laser::Data_Process(const uint8_t *raw_data, size_t length, LaserData *out_data)
{
    // 1. 基础检查：一帧最短也要有类似 "0,0\r\n" 共 5 个字节
    if (raw_data == NULL || length < 5 || out_data == NULL) {
        return 0; 
    }

    // 2. 寻找帧尾 0x0D 0x0A (\r\n)
    size_t tail_index = 0;
    int frame_found = 0;
    for (size_t i = 0; i < length - 1; i++) {
        if (raw_data[i] == 0x0D && raw_data[i+1] == 0x0A) {
            tail_index = i;
            frame_found = 1;
            break;
        }
    }

    if (!frame_found) {
        return 0; // 没找到帧尾，可能数据还没接收全
    }

    // 3. 手动解析 "float,int"，避免 scanf 浮点支持依赖
    size_t i = 0;
    int sign = 1;
    float value = 0.0f;
    float frac = 0.0f;
    float base = 0.1f;
    int has_digit = 0;

    if (i < tail_index && raw_data[i] == '-') {
        sign = -1;
        i++;
    }

    while (i < tail_index && raw_data[i] >= '0' && raw_data[i] <= '9') {
        value = value * 10.0f + (float)(raw_data[i] - '0');
        has_digit = 1;
        i++;
    }

    if (i < tail_index && raw_data[i] == '.') {
        i++;
        while (i < tail_index && raw_data[i] >= '0' && raw_data[i] <= '9') {
            frac += (float)(raw_data[i] - '0') * base;
            base *= 0.1f;
            has_digit = 1;
            i++;
        }
    }

    if (!has_digit) {
        return 0;
    }

    if (i >= tail_index || raw_data[i] != ',') {
        return 0;
    }
    i++;

    int status = 0;
    int status_digit = 0;
    while (i < tail_index && raw_data[i] >= '0' && raw_data[i] <= '9') {
        status = status * 10 + (int)(raw_data[i] - '0');
        status_digit = 1;
        i++;
    }

    if (!status_digit) {
        return 0;
    }

    out_data->distance = (value + frac) * (float)sign;
    out_data->status = status;
    return 1;
}