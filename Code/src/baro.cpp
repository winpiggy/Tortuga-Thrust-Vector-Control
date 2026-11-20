#include "baro.h"
#include "setup.h"
#include <math.h>



bool BME::begin()
{
    // Teensy 4.1 Wire uses fixed pins (SDA=18, SCL=19), see pins.h
    Wire.begin();

    // Most breakouts are 0x76 or 0x77
    if (!bme.begin(0x77) && !bme.begin(0x76)) {
        Serial.println("Failed to initialize BME280");
        return false;
    }

    // The library default (16x oversampling on everything) only produces a new
    // reading about every 110 ms. These settings give roughly 60 readings per second,
    // and the IIR filter smooths pressure noise.
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,    // temperature
                    Adafruit_BME280::SAMPLING_X4,    // pressure
                    Adafruit_BME280::SAMPLING_NONE,  // humidity (not used)
                    Adafruit_BME280::FILTER_X4,
                    Adafruit_BME280::STANDBY_MS_0_5);
    return true;
}

void BME::read(float &temperature, float &pressure, float &altitude)
{
    temperature = bme.readTemperature();
    pressure = bme.readPressure();
    // Same formula as bme.readAltitude(), but reuses the pressure we just read
    // instead of reading the sensor a second time.
    altitude = 44330.0f * (1.0f - powf((pressure / 100.0f) / seaLevelPressure, 0.1903f));
}
