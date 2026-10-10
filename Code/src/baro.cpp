#include "baro.h"

static const float seaLevelPressure = 1013.25f; // hPa

bool BME::begin()
{
    // Teensy 4.1 Wire uses fixed pins (SDA=18, SCL=19), see pins.h
    Wire.begin();

    // Most breakouts are 0x76 or 0x77
    if (!bme.begin(0x77) && !bme.begin(0x76)) {
        Serial.println("Failed to initialize BME280");
        return false;
    }
    return true;
}

void BME::read(float &temperature, float &pressure, float &altitude)
{
    temperature = bme.readTemperature();
    pressure = bme.readPressure();
    altitude = bme.readAltitude(seaLevelPressure);
}
