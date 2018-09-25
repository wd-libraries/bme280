/***************************************************
 * This example illustrates interfacing with two BME280
 * sensors using the ESP8266 chip.
 *
 * Equipment:
 * ESP8266-12E NodeMCU development board
 * BME280 sensor breakout board with pinouts:
 * SDA, SCL, GND, PWR, and SDO
 *
 * Circuit connections {dev board: sensor}: 
 *   {D1:  SDA},
 *   {D2:  SDC/SCL},
 *   {GND: GND},
 *   {3V:  PWR}
 *   {3V:  SDO} // to set the I2C address correctly
 *
 * We use D1 (GPIO5) and D2 (GPIO4) for I2C because they
 * don't have any conflicting secondary functions.
 *
 * IMPORTANT: read Bosch's datasheet on this sensor
 * before using in a production environment.
 *
 * Sponsored by https://wombatdashboard.com
 * Connect your IoT sensor to WombatDashboard today!
 *
 * Copyright 2018 Perspicacity Ltd. MIT licenced.
 * Last updated 2018-10
 ***************************************************/

#include <Wire.h>
#include <wd_bme280.h>

#define LOOP_DELAY  3 // temperature reading time interval, in seconds
#define BAUD_RATE   9600 // ensure your serial monitor window uses the same baud rate

#define PORT_SDA    D1
#define PORT_SCL    D2
#define BME280_SLAVEADDR 0x77 // don't forget to wire SDO as HIGH
#define BME280_IRR_FILTER 0
#define BME280_MODE 0b11
#define BME280_STANDBY_TIME 4

BME280 sensor = BME280(BME280_SLAVEADDR, BME280_IRR_FILTER);

void setup() {
    Serial.begin(BAUD_RATE);
    bool success = sensor.begin(PORT_SDA, PORT_SCL);
    if(!success) {
        Serial.println("Setup failed");
        abort();
    } else {
        Serial.println("Setup complete");
    }
    sensor.reset();
    // some register writes are stateful, eg BME280_CTRL_MEAS_REG must be written
    // after writing BME280_CTRL_HUMIDITY_REG, otherwise settings don't take effect
    // so it's important to give the sensor time to cycle through its states by setting
    // the smallest sensor standby time possible (000), and delaying after such operations
    sensor.setStandbyTime(000);
    sensor.setHumidityOversample(2);
    delay(5);
    sensor.setTempOversample(1);
    delay(5);
    sensor.setPressureOversample(4);
    delay(5);
    sensor.begin(PORT_SDA, PORT_SCL);
    sensor.setMode(BME280_MODE);
    delay(5);
    sensor.setStandbyTime(BME280_STANDBY_TIME);
    delay(5);
}

void loop() {
    // ensure you set temperature, pressure, and humidity oversampling rates to non-zero
    // in setup(), otherwise the values retreived by these methods will be wrong
    Serial.print('\n');
    Serial.print("Temp: ");
    Serial.println(sensor.getTemp('C'));
    Serial.print("Pressure: ");
    Serial.println(sensor.getPressure());
    Serial.print("Humidity: ");
    Serial.println(sensor.getHumidity());

    Serial.println("*****");
    delay(LOOP_DELAY*1000);
}
