#ifndef BARO_H
#define BARO_H
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "Adafruit_BME280.h"



class BME {

    public:

    BME();
    void begin();
    float readTemperature();
    float readPressure();
    float readAltitude();

    private:
    Adafruit_BME280 bme;
};


#endif // BARO_H