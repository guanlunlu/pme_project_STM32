#ifndef __PMW3901_H__
#define __PMW3901_H__

#include "stdint.h"
#include "GPIO.h"
#include "stdbool.h"

typedef struct PMW3901
{
    GPIO_TypeDef *_cs_port;
    uint16_t _cs_pin;
    int16_t deltaX;
    int16_t deltaY;
    bool transmit_flag;
    SPI_HandleTypeDef *hspi;
    bool startReq;
    bool endReg;
    double getData;
    int16_t data1;
    uint8_t data2;
    int16_t data3;
    uint8_t data4;


} pmw;

bool pmw_init(pmw *pmw3901);
void registerWrite(pmw *this, uint8_t reg, uint8_t value);
uint8_t registerRead(pmw *this, uint8_t reg);
uint8_t registerRead_IT(pmw *this, uint8_t reg);

void initRegisters(pmw *this);
void readMotionCount(pmw *this);


#endif
