/***************************************************
 * This library is for interfacing with the BME280
 * sensor using the ESP8266 chip. These sensors use 
 * the I2C protocol.
 * Page references refer to Bosch's datasheet of
 * 2016-05-03, BST-BME280-DS-001-12
 *
 * Sponsored by https://wombatdashboard.com
 * Connect your IoT sensor to WombatDashboard today!
 *
 * Copyright 2018 Perspicacity Ltd. MIT licenced.
 ***************************************************/

#include "Wire.h"
#include <Arduino.h>

// various special values and bit masks

#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11
#define RESET_WORD 0xB6 // see ch 5.4.2 of datasheet
#define CHIP_ID 0x60

#define STANDBY_TIME_MASK           0b00011111
#define OVERSAMPLE_TEMP_MASK        0b00011111
#define OVERSAMPLE_PRESSURE_MASK    0b11100011
#define OVERSAMPLE_HUMIDITY_MASK    0b11111000


// Register names. All DIG registers are factory-calibrated compensation params
#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 // Chip ID, should be 0x60
#define BME280_RST_REG					0xE0 // Soft reset, see ch 5.4.2 of datasheet
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 // Humidity oversampling
#define BME280_STAT_REG					0xF3 // Status
#define BME280_CTRL_MEAS_REG			0xF4 // Temp & pressure oversampling
#define BME280_CONFIG_REG				0xF5 // Configuration
#define BME280_PRESSURE_MSB_REG			0xF7 // Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 // Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 // Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA // Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB // Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC // Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD // Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE // Humidity LSB

// constants to convert between various SI and imperial units
#define INCHES_MERCURY 0.000295333727 // this many inches of mercury per hPa
#define KELVIN_OFFSET 273.15

struct Calibration {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
};

class BME280 {
    public:
        BME280(uint8_t addr, uint8_t filter);
        void    reset(void);
        bool    begin(uint8_t sda, uint8_t sdc);
        float   getTemp(char units = 'C');
        float   getHumidity(void);
        float   getPressure(char units = 'P');
        void    setMode(uint8_t mode);
        uint8_t getMode(void);
        void    sleep(void);
        void    setStandbyTime(uint8_t t);
        void    setTempOversample(uint8_t t);
        void    setPressureOversample(uint8_t t);
        void    setHumidityOversample(uint8_t t);
        uint8_t readByte(uint8_t a);
    private:
        uint8_t  _addr;
        uint8_t  _filter;
        uint32_t _tfine;
        Calibration _cal;
        void    readWord(uint8_t *dest, uint8_t a, uint8_t length);
        uint8_t writeData(uint8_t a, uint8_t data);
        void setFilter(uint8_t filter);
};
