/***************************************************
 * This library is for interfacing with the BME280
 * sensor using the ESP266 chip via the I2C protocol. 
 * Common tasks like setting filter coefficients and 
 * getting sensor readings are abstracted.
 * Inspired by SparkFun's BME280 Arduino library.
 *
 * Sponsored by https://wombatdashboard.com
 * Connect your IoT sensor to WombatDashboard today!
 *
 * Copyright 2018 Perspicacity Ltd. MIT licenced.
 * Last updated: 2018-07-31
 ***************************************************/

#include "wd_bme280.h"

/*
 * Constructor
 * @param addr: the I2C address of the BME280 slave device
 * @param filter: the 3-bit value corresponding to various IIR filter
 * coefficients:
 *   0b000: off
 *   0b001: 2
 *   0b010: 4
 *   0b011: 8
 *   0b100: 16
 */
BME280::BME280(uint8_t addr, uint8_t filter) {
    _addr = addr;
    _filter = filter;
    _cal = {}; // set all calibration constants to zero when instantiating
}

/*
 * Resets the sensor (ch5.4.2 of datasheet). Don't forget to call begin() afterwards
 */
void BME280::reset(void) {
    writeData(BME280_RST_REG, RESET_WORD);
}

/*
 * Wrapper function to begin I2C communication with the BME280 and set up the
 * sensor's IIR filter. Returns false upon error, true otherwise.
 */
bool BME280::BME280::begin(uint8_t sda, uint8_t scl) {
    delay(3); // allow the sensor 3ms to boot
    Wire.begin(sda, scl);
    Wire.beginTransmission(_addr);
    uint8_t error = Wire.endTransmission();
    if (error != 0){
        Serial.print("endTransmission returned "); Serial.println(error);
        return false;
    }
    // now we need to read and store all the calibration constants for use later
	// (ranges 0x88:A1, 0xE1:E7 as per datasheet ch4.2.2)
	_cal.dig_T1 = ((uint16_t)((readByte(BME280_DIG_T1_MSB_REG) << 8) + readByte(BME280_DIG_T1_LSB_REG)));
	_cal.dig_T2 = ((int16_t)((readByte(BME280_DIG_T2_MSB_REG) << 8) + readByte(BME280_DIG_T2_LSB_REG)));
	_cal.dig_T3 = ((int16_t)((readByte(BME280_DIG_T3_MSB_REG) << 8) + readByte(BME280_DIG_T3_LSB_REG)));

	_cal.dig_P1 = ((uint16_t)((readByte(BME280_DIG_P1_MSB_REG) << 8) + readByte(BME280_DIG_P1_LSB_REG)));
	_cal.dig_P2 = ((int16_t)((readByte(BME280_DIG_P2_MSB_REG) << 8) + readByte(BME280_DIG_P2_LSB_REG)));
	_cal.dig_P3 = ((int16_t)((readByte(BME280_DIG_P3_MSB_REG) << 8) + readByte(BME280_DIG_P3_LSB_REG)));
	_cal.dig_P4 = ((int16_t)((readByte(BME280_DIG_P4_MSB_REG) << 8) + readByte(BME280_DIG_P4_LSB_REG)));
	_cal.dig_P5 = ((int16_t)((readByte(BME280_DIG_P5_MSB_REG) << 8) + readByte(BME280_DIG_P5_LSB_REG)));
	_cal.dig_P6 = ((int16_t)((readByte(BME280_DIG_P6_MSB_REG) << 8) + readByte(BME280_DIG_P6_LSB_REG)));
	_cal.dig_P7 = ((int16_t)((readByte(BME280_DIG_P7_MSB_REG) << 8) + readByte(BME280_DIG_P7_LSB_REG)));
	_cal.dig_P8 = ((int16_t)((readByte(BME280_DIG_P8_MSB_REG) << 8) + readByte(BME280_DIG_P8_LSB_REG)));
	_cal.dig_P9 = ((int16_t)((readByte(BME280_DIG_P9_MSB_REG) << 8) + readByte(BME280_DIG_P9_LSB_REG)));

	_cal.dig_H1 = ((uint8_t)(readByte(BME280_DIG_H1_REG)));
	_cal.dig_H2 = ((int16_t)((readByte(BME280_DIG_H2_MSB_REG) << 8) + readByte(BME280_DIG_H2_LSB_REG)));
	_cal.dig_H3 = ((uint8_t)(readByte(BME280_DIG_H3_REG)));
	_cal.dig_H4 = ((int16_t)((readByte(BME280_DIG_H4_MSB_REG) << 4) + (readByte(BME280_DIG_H4_LSB_REG) & 0x0F)));
	_cal.dig_H5 = ((int16_t)((readByte(BME280_DIG_H5_MSB_REG) << 4) + ((readByte(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
    _cal.dig_H6 = ((int8_t)readByte(BME280_DIG_H6_REG));
    
//    Serial.println(_cal.dig_T1);
//    Serial.println(_cal.dig_T2);
//    Serial.println(_cal.dig_T3);
//    Serial.println(_cal.dig_P1);
//    Serial.println(_cal.dig_P2);
//    Serial.println(_cal.dig_P3);
//    Serial.println(_cal.dig_P4);
//    Serial.println(_cal.dig_P5);
//    Serial.println(_cal.dig_P6);
//    Serial.println(_cal.dig_P7);
//    Serial.println(_cal.dig_P8);
//    Serial.println(_cal.dig_P9);
//    Serial.println(_cal.dig_H1);
//    Serial.println(_cal.dig_H2);
//    Serial.println(_cal.dig_H3);
//    Serial.println(_cal.dig_H4);
//    Serial.println(_cal.dig_H5);
//    Serial.println(_cal.dig_H6);

    getTemp('C'); // set this->_tfine
    setFilter(_filter); // set the IIR filter value

    return true;
}

/*
 * Reads a word of length "length" read at address 'a' and saves it to the
 * "dest" array. It is the caller's responsibility to ensure at least
 * length*sizeof(uint8_t) memory is allocated for this.
 */
void BME280::readWord(uint8_t *dest, uint8_t a, uint8_t length) {
    Wire.beginTransmission(_addr); // slave I2C device is at address _addr
    Wire.write(a); // tell the device which register we want to read
    Wire.endTransmission();
    Wire.requestFrom(_addr, length); // get bytes from slave
    // The slave can only send 1 byte at a time, so we request byte by byte.
    for(uint8_t i=0; Wire.available() && i<length; i++) {
        *dest = Wire.read();
        dest++;
    }
}

/*
 * Reads a byte at address 'a' and returns it as a uint8_t
 */
uint8_t BME280::readByte(uint8_t a) {
    Wire.beginTransmission(_addr); // slave I2C device is at address _addr
    Wire.write(a); // tell the device which register we want to read
    Wire.endTransmission();
    Wire.requestFrom(_addr, 1); // get 1 byte from slave
    return Wire.read();
}

/*
 * Write a byte of data to address 'a'
 */
uint8_t BME280::writeData(uint8_t a, uint8_t data) {
    Wire.beginTransmission(_addr); // slave I2C device is at address _addr
    Wire.write(a); // tell the device which register we want to write to
    Wire.write(data); // write the data
    Wire.endTransmission();
}


// Below methods are utilities for performing device control/calibration

/*
 * Sets the measurement mode in CTRL_MEAS register) from a choice of values:
 * 00: Sleep
 * 01: Forced
 * 10: Forced
 * 11: Normal
 */
void BME280::setMode(uint8_t mode) {
    
	if(mode > MODE_NORMAL) mode = 0; // validate mode, default to sleep if invalid
	uint8_t data = readByte(BME280_CTRL_MEAS_REG);
//    Serial.print("sM: original CTRL_MEAS="); Serial.println(readByte(BME280_CTRL_MEAS_REG), BIN);
    // the operating mode is bits 0 & 1 of CTRL_MEAS register. See ch5.3 of datasheet.
	data &= ~( (1<<1) | (1<<0) ); // zero bits 1 and 0
	data |= mode; // set bits 1 and 0
	writeData(BME280_CTRL_MEAS_REG, data);
//    Serial.print("sM: new CTRL_MEAS="); Serial.print(readByte(BME280_CTRL_MEAS_REG), BIN);
//    Serial.print(" (versus mode "); Serial.print(data & 0b00000011); Serial.println(')');
}

/*
 * Gets the measurement mode in CTRL_MEAS register)
 */
uint8_t BME280::getMode(void) {
	uint8_t data = readByte(BME280_CTRL_MEAS_REG);
	return(data & 0b00000011); // zero bits 7 to 2
}

/*
 * Causes the sensor to sleep (not take any measurements). Set mode back to Normal or Forced
 * in order to "wake" the sensor.
 */
void BME280::sleep(void) {
    setMode(MODE_SLEEP);
}

/*
 * Set the IRR filter value. See Ch9 of datasheet for calcs.
 * Allowable values are:
 * 0b000: off
 * 0b001: 2
 * 0b010: 4
 * 0b011: 8
 * 0b100, others : 16
 */
void BME280::setFilter(uint8_t filter) {
    if(filter > 0b100) filter = 0; // validate value, filter off if invalid
    // filter value is stored in bits 2,3,4 of CONFIG register. See ch5.3 of datasheet
    uint8_t data = readByte(BME280_CONFIG_REG);
    data &= ~( (1<<4) | (1<<3) | (1<<2) ); // zero bits 2, 3, 4
    data |= (filter << 2); // replace bits 2, 3, 4
    writeData(BME280_CONFIG_REG, data);
}

/*
 * In Normal mode, the sensor cycles between active measurement, and inactive standby.
 * This method sets the standby time duration 't' of the cycle. See ch3.3.4 of datasheet.
 * Acceptable values (and their corresponding time in milliseconds):
 *  0: 0.5ms
 *  1: 62.5ms
 *  2: 125ms
 *  3: 250ms
 *  4: 500ms
 *  5: 1000ms
 *  6: 10ms
 *  7: 20ms
 */
void BME280::setStandbyTime(uint8_t t) {
	if(t > 0b111) t = 4; // validate specified standby duration. Default to 500ms
	uint8_t data = readByte(BME280_CONFIG_REG);
	data &= STANDBY_TIME_MASK; // zero bits 5,6,7
	data |= (t << 5); // set bits 5,6,7
    writeData(BME280_CONFIG_REG, data);
}

/*
 * Sets temperature oversampling value.
 * Note value of 0 disables temperature sensing. See ch5.4.5 of datasheet.
 */
void BME280::setTempOversample(uint8_t t) {
    if(t > 0b101) t = 0b101;
	uint8_t mode = getMode(); // remember the present mode
	setMode(MODE_SLEEP); // needs to be in sleep mode to write config
	uint8_t data = readByte(BME280_CTRL_MEAS_REG);
    Serial.print("sTO CTRL_MEAS before: ");
    Serial.print(data, BIN); Serial.print(", ");
	data &= OVERSAMPLE_TEMP_MASK; // zero bits 7, 6, and 5
	data |= t << 5; // write 't' in bits 7, 6, 5
	writeData(BME280_CTRL_MEAS_REG, data);
    Serial.print("sTO CTRL_MEAS after: ");
    Serial.println(readByte(BME280_CTRL_MEAS_REG), BIN);
    setMode(mode); // return to original mode
}

/*
 * Sets pressure oversampling value.
 * Note that value of 0 disables pressure sensing. See ch5.4.5 of datasheet.
 */
void BME280::setPressureOversample(uint8_t t) {
    if(t > 0b101) t = 0b101;
	uint8_t mode = getMode(); // remember the present mode
	setMode(MODE_SLEEP); // needs to be in sleep mode to write config
	uint8_t data = readByte(BME280_CTRL_MEAS_REG);
    Serial.print("sPO CTRL_MEAS before: ");
    Serial.print(data, BIN); Serial.print(", ");
	data &= OVERSAMPLE_PRESSURE_MASK; // zero bits 4, 3, and 2
	data |= t << 2; // write 't' in bits 4, 3, 2
	writeData(BME280_CTRL_MEAS_REG, data);
    Serial.print("sPO CTRL_MEAS after: ");
    Serial.println(readByte(BME280_CTRL_MEAS_REG), BIN);

    Serial.print("sPO CTRL_HUMIDITY after: ");
    Serial.println(readByte(BME280_CTRL_HUMIDITY_REG), BIN);

    setMode(mode); // return to original mode
}

/*
 * Sets humidity oversampling value.
 * Note that value of 0 disables humidity sensing. See ch3.5 of datasheet.
 * Note that changes to the CTRL_HUMIDITY_REG only become effective after
 * writing to CTRL_MEAS_REG, i.e. you must call setPressureOversample or
 * setTempOversample for the humidity oversample settings to take effect.
 */
void BME280::setHumidityOversample(uint8_t t) {
    if(t > 0b101) t = 0b101;
	uint8_t mode = getMode(); // remember the present mode
    Serial.print("sHO Current mode is "); Serial.println(mode, BIN);
	setMode(MODE_SLEEP); // needs to be in sleep mode to write config
	uint8_t meas_data = readByte(BME280_CTRL_MEAS_REG);
	uint8_t data = readByte(BME280_CTRL_HUMIDITY_REG);
    Serial.print("sHO value is "); Serial.println(data, BIN);
	data &= OVERSAMPLE_HUMIDITY_MASK; // zero bits 2, 1, and 0
	data |= t << 0; // write 't' in bits 2, 1, 0

    Wire.beginTransmission(_addr); // slave I2C device is at address _addr
    Wire.write(BME280_CTRL_HUMIDITY_REG); // tell the device which register we want to write to
    Wire.write(data); // write the data
    delay(10);
    Wire.write(BME280_CTRL_MEAS_REG); // tell the device which register we want to write to
    Wire.write(meas_data); // write the data
    Wire.endTransmission();

//	writeData(BME280_CTRL_HUMIDITY_REG, data);
    // changes to CTRL_HUMIDITY_REG only take effect after writing to
    // CTRL_MEAS_REG so do that now (see ch 5.4.3)
//	writeData(BME280_CTRL_MEAS_REG, meas_data);
    Serial.print("sHO wrote data "); Serial.println(data, BIN);
    setMode(mode); // return to original mode
    Serial.print("sHO re-read CTRL_HUMIDITY_REG ");
    Serial.println(readByte(BME280_CTRL_HUMIDITY_REG) & 0b00000111, BIN);
}

// Next 3 methods fetch temperature, humidity, and pressure data. As per ch4
// of datasheet, "burst" reading these data registers all at once (registers F7 to FE)
// would be ideal, due to lower latency. However, for code readability, we won't do
// that because our anticipated usage cases don't require the ultra low latency.

/*
 * Returns pressure in Pascals in Q24.8 format (24 integer bits and 8 fraction bits)
 */
float BME280::getPressure(char units) {
	uint8_t pres_msb = readByte(BME280_PRESSURE_MSB_REG);
	uint8_t pres_lsb = readByte(BME280_PRESSURE_LSB_REG);
	uint8_t pres_xlsb = readByte(BME280_PRESSURE_XLSB_REG);
    int32_t adc_P = ((uint32_t)pres_msb << 12)|
            ((uint32_t)pres_lsb << 4) |
            ((uint32_t)pres_xlsb >> 4); // assemble the 3*8bits into a 32bit word
    // The pressure reading is affected by temperature, and requires compensation.
    // Bosch provide the following compensation formula in section 4.2.3 of their datasheet
    int64_t var1, var2, p_acc;
	var1 = ((int64_t)_tfine) - 128000;
	var2 = var1 * var1 * (int64_t)_cal.dig_P6;
	var2 = var2 + ((var1 * (int64_t)_cal.dig_P5)<<17);
	var2 = var2 + (((int64_t)_cal.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)_cal.dig_P3)>>8) + ((var1 * (int64_t)_cal.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_cal.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)_cal.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)_cal.dig_P8) * p_acc) >> 19;
    p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)_cal.dig_P7)<<4);
    
    // finally, divide by 2^8 to account for fractional bits in representation
    float output = (float)p_acc / 256.0;
    switch(units) {
    case 'H':
        return output * INCHES_MERCURY;
    default:
        return output;
    }
}

/*
 * Returns humidity in % in Q22.10 format (22 integer and 10 fractional bits)
 */
float BME280::getHumidity(void) {
	uint8_t humid_msb = readByte(BME280_HUMIDITY_MSB_REG);
	uint8_t humid_lsb = readByte(BME280_HUMIDITY_LSB_REG);
    Serial.print("getHumidity registers are ");
    Serial.print(humid_msb); Serial.print(", ");
    Serial.println(humid_lsb);

    // assemble the 2*8 bits from the ADC into a 16 bit word
    int32_t adc_H = ((uint32_t)humid_msb << 8) | ((uint32_t)humid_lsb);
    // The humidity reading from the ADC must be compensated by factory calibration params
    // Bosch provide the following compensation formula in section 4.2.3 of their datasheet
	int32_t var1;
	var1 = (_tfine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)_cal.dig_H4) << 20) - (((int32_t)_cal.dig_H5) * var1)) + ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)_cal.dig_H6)) >> 10) * (((var1 * ((int32_t)_cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)_cal.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)_cal.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	// divide by 2^10 to account for fractional bits in representation
	return (float)(var1>>12) / 1024.0;
}

/*
 * Returns temperature in degree C by default, or optionally in 'K' or 'F' via the "units" argument
 */
float BME280::getTemp(char units) {
	// Note compensated readings of humidity and pressure depend on temperature. This method
	// updates the private instance variable _tfine upon which pressure and humidity calcs depend

	uint8_t temp_msb = readByte(BME280_TEMPERATURE_MSB_REG);
	uint8_t temp_lsb = readByte(BME280_TEMPERATURE_LSB_REG);
	uint8_t temp_xlsb = readByte(BME280_TEMPERATURE_XLSB_REG);
    // assemble the 3*8 bits from the ADC into a 32 bit word
    int32_t adc_T = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | ((temp_xlsb >> 4) & 0x0F);

    // The temperature reading from the ADC must be compensated by factory calibration params
    // Bosch provide the following compensation formula in section 4.2.3 of their datasheet
	int64_t var1, var2;
	var1 = ((((adc_T>>3) - ((int32_t)_cal.dig_T1<<1))) * ((int32_t)_cal.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)_cal.dig_T1)) * ((adc_T>>4) - ((int32_t)_cal.dig_T1))) >> 12) *
	((int32_t)_cal.dig_T3)) >> 14;
	_tfine = var1 + var2;
	int32_t val = ((int32_t)_tfine * 5 + 128) >> 8;
	float output = (float)val / 100.0;
	switch(units) {
    case 'K':
        return output + KELVIN_OFFSET;
    case 'F':
        return output * 9.0/5.0 + 32;
    default:
        return output;
    }
}
