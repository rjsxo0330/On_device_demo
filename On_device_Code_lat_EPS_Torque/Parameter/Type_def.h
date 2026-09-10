#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#include <stdint.h>

#define RX_FLAG_OFF    0
#define RX_FLAG_ON     1

typedef float			float32_t;
typedef double			float64_t;

typedef struct
{
           uint16_t   Left_Lane_A_766          :1;
           uint16_t   Left_Lane_B_767          :1;
           uint16_t   Right_Lane_A_768         :1;
           uint16_t   Right_Lane_B_769         :1;

} MobileyeFlag4_t; 

typedef struct 
{
           uint16_t   u1Bit_0          :1;
           uint16_t   u1Bit_1          :1;
           uint16_t   u1Bit_2          :1;
           uint16_t   u1Bit_3          :1;
           uint16_t   u1Bit_4          :1;
           uint16_t   u1Bit_5          :1;
           uint16_t   u1Bit_6          :1;
           uint16_t   u1Bit_7          :1;
} flag8_t;   

typedef struct 
{
           uint16_t   u1Bit_0          :1;
           uint16_t   u1Bit_1          :1;
           uint16_t   u1Bit_2          :1;
           uint16_t   u1Bit_3          :1;
           uint16_t   u1Bit_4          :1;
           uint16_t   u1Bit_5          :1;
           uint16_t   u1Bit_6          :1;
           uint16_t   u1Bit_7          :1;
           uint16_t   u1Bit_8          :1;
           uint16_t   u1Bit_9          :1;
           uint16_t   u1Bit_10         :1;
           uint16_t   u1Bit_11         :1;
           uint16_t   u1Bit_12         :1;
           uint16_t   u1Bit_13         :1;
           uint16_t   u1Bit_14         :1;
           uint16_t   u1Bit_15         :1;
} flag16_t;
#endif