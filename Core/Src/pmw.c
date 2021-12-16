#include "pmw.h"

#include "spi.h"
#include "robot.h"

bool pmw_init(pmw *this)
{
    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 1);
    HAL_Delay(10);
    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 1);
    HAL_Delay(10);
    registerWrite(this, 0x3A, 0x5A);
    HAL_Delay(10);

    uint8_t chipId = registerRead(this, 0x00);
    uint8_t dIpihc = registerRead(this, 0x5F);

    if (chipId != 0x49 && dIpihc != 0xB8)
        return false;

    registerRead(this, 0x02);
    registerRead(this, 0x03);
    registerRead(this, 0x04);
    registerRead(this, 0x05);
    registerRead(this, 0x06);
    HAL_Delay(1);

    initRegisters(this);
    return true;
}

void registerWrite(pmw *this, uint8_t reg, uint8_t value)
{
    reg |= 0x80u;
    // SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 0);
    // digitalWrite(_cs, LOW);

//    delayMicroseconds(50);
//    for (int i = 0; i < 5; i++){
//
//    }
    HAL_SPI_Transmit_IT(this->hspi, &reg, 1);
    HAL_SPI_Transmit_IT(this->hspi, &value, 1);

//    SPI.transfer(reg);
//    SPI.transfer(value);
//    for (int i = 0; i < 5; i++)
//    {
//    }
    // delayMicroseconds(50);
    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 1);

    // digitalWrite(_cs, HIGH);

    // SPI.endTransaction();

    // delayMicroseconds(200);
//    for (int i = 0; i < 5; i++)
//    {
//    }
}

uint8_t registerRead(pmw *this, uint8_t reg)
{
    reg &= ~0x80u;


    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 0);

    HAL_SPI_Transmit_IT(this->hspi, &reg, 1);

    uint8_t value = 0;
    uint8_t pupu = 0;

    HAL_SPI_TransmitReceive_IT(this->hspi, &pupu, &value, 1);
    HAL_GPIO_WritePin(this->_cs_port, this->_cs_pin, 1);


    return value;
}

void initRegisters(pmw *this)
{
    registerWrite(this, 0x7F, 0x00);
    registerWrite(this, 0x61, 0xAD);
    registerWrite(this, 0x7F, 0x03);
    registerWrite(this, 0x40, 0x00);
    registerWrite(this, 0x7F, 0x05);
    registerWrite(this, 0x41, 0xB3);
    registerWrite(this, 0x43, 0xF1);
    registerWrite(this, 0x45, 0x14);
    registerWrite(this, 0x5B, 0x32);
    registerWrite(this, 0x5F, 0x34);
    registerWrite(this, 0x7B, 0x08);
    registerWrite(this, 0x7F, 0x06);
    registerWrite(this, 0x44, 0x1B);
    registerWrite(this, 0x40, 0xBF);
    registerWrite(this, 0x4E, 0x3F);
    registerWrite(this, 0x7F, 0x08);
    registerWrite(this, 0x65, 0x20);
    registerWrite(this, 0x6A, 0x18);
    registerWrite(this, 0x7F, 0x09);
    registerWrite(this, 0x4F, 0xAF);
    registerWrite(this, 0x5F, 0x40);
    registerWrite(this, 0x48, 0x80);
    registerWrite(this, 0x49, 0x80);
    registerWrite(this, 0x57, 0x77);
    registerWrite(this, 0x60, 0x78);
    registerWrite(this, 0x61, 0x78);
    registerWrite(this, 0x62, 0x08);
    registerWrite(this, 0x63, 0x50);
    registerWrite(this, 0x7F, 0x0A);
    registerWrite(this, 0x45, 0x60);
    registerWrite(this, 0x7F, 0x00);
    registerWrite(this, 0x4D, 0x11);
    registerWrite(this, 0x55, 0x80);
    registerWrite(this, 0x74, 0x1F);
    registerWrite(this, 0x75, 0x1F);
    registerWrite(this, 0x4A, 0x78);
    registerWrite(this, 0x4B, 0x78);
    registerWrite(this, 0x44, 0x08);
    registerWrite(this, 0x45, 0x50);
    registerWrite(this, 0x64, 0xFF);
    registerWrite(this, 0x65, 0x1F);
    registerWrite(this, 0x7F, 0x14);
    registerWrite(this, 0x65, 0x60);
    registerWrite(this, 0x66, 0x08);
    registerWrite(this, 0x63, 0x78);
    registerWrite(this, 0x7F, 0x15);
    registerWrite(this, 0x48, 0x58);
    registerWrite(this, 0x7F, 0x07);
    registerWrite(this, 0x41, 0x0D);
    registerWrite(this, 0x43, 0x14);
    registerWrite(this, 0x4B, 0x0E);
    registerWrite(this, 0x45, 0x0F);
    registerWrite(this, 0x44, 0x42);
    registerWrite(this, 0x4C, 0x80);
    registerWrite(this, 0x7F, 0x10);
    registerWrite(this, 0x5B, 0x02);
    registerWrite(this, 0x7F, 0x07);
    registerWrite(this, 0x40, 0x41);
    registerWrite(this, 0x70, 0x00);

    HAL_Delay(100);
    registerWrite(this, 0x32, 0x44);
    registerWrite(this, 0x7F, 0x07);
    registerWrite(this, 0x40, 0x40);
    registerWrite(this, 0x7F, 0x06);
    registerWrite(this, 0x62, 0xf0);
    registerWrite(this, 0x63, 0x00);
    registerWrite(this, 0x7F, 0x0D);
    registerWrite(this, 0x48, 0xC0);
    registerWrite(this, 0x6F, 0xd5);
    registerWrite(this, 0x7F, 0x00);
    registerWrite(this, 0x5B, 0xa0);
    registerWrite(this, 0x4E, 0xA8);
    registerWrite(this, 0x5A, 0x50);
    registerWrite(this, 0x40, 0x80);
}

void readMotionCount(pmw* this)
{

    registerRead(this,0x02);
    this->data1 = (int16_t)registerRead(this, 0x04);
    this->data1 <<= 8;
    this->data2 = registerRead(this, 0x03);
    this->data3 = (int16_t)registerRead(this, 0x06);
    this->data3 <<= 8;
    this->data4 = registerRead(this, 0x05);
	this->deltaX = (this->data1) | this->data2;
	this->deltaY = (this->data3) | this->data4;


}
