#ifndef BARO_H
#define BARO_H
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include <Adafruit_BME280.h>

class BME {

    public:

    BME() = default;

    // Returns false if the sensor is not found on the I2C bus.
    bool begin();

    // Temperature in C, pressure in Pa, altitude in m relative to seaLevelPressure.
    void read(float &temperature, float &pressure, float &altitude);

    private:
    Adafruit_BME280 bme;
};

#endif // BARO_H
