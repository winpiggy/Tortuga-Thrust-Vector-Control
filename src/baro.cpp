#include "pins.h"
#include <Wire.h>
#include <Arduino.h>
#include "baro.h"

void BME::begin(){
    Wire.begin(I2C_SCL, I2C_SDA);
    bme.begin();
    if(!bme.begin()){
        Serial.println("Failed to initialize BME280");
    }
}
float const seaLevelPressure = 1013.25f;

void BME::read (float &temperature, float &pressure, float &altitude)
{
temperature = bme.readTemperature();
pressure = bme.readPressure();
altitude = bme.readAltitude(seaLevelPressure);

}
BME::BME(){

    bme = Adafruit_BME280();

    BME::begin();

    BME::read(temperature, pressure, altitude);

}








