#include "baro.h"



bool BME::begin()
{
    // Most breakouts are 0x76 or 0x77
    if (!bme.begin(0x77, &Wire) && !bme.begin(0x76, &Wire)) {
        Serial.println("Failed to initialize BME280");
        return false;
    }
    return true;
}

void BME::read(float &temperature, float &pressure, float &altitude)
{
    temperature = bme.readTemperature();
    pressure = bme.readPressure();

    // Same formula as bme.readAltitude(), but reuses the pressure we just read
    // instead of doing another I2C transaction every loop.
    altitude = 44330.0f * (1.0f - powf((pressure / 100.0f) / seaLevelPressure, 0.1903f));
}
