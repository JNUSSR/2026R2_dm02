#ifndef Laser_H
#define Laser_H

#include <stdint.h>
#include <stddef.h>


typedef struct {
    float distance; // 距离 (单位: mm)
    int status;     // 置信度状态 (0-1000)
} LaserData;

class Laser
{
    public:
        LaserData Data; // 激光数据

        int Data_Process(const uint8_t *raw_data, size_t length, LaserData *out_data);
    protected:
        
};



#endif // Laser_H