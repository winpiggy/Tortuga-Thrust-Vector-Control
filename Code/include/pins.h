#ifndef PINS_H
#define PINS_H

// Pin definitions for the Tortuga Thrust Vector Control system


// Servo pins

int const PitchPin = 14;
int const YawPin = 15; 

// Sensor pins

int const SCLpin = 19;
int const SDApin = 18;

#define I2C_SCL SCLpin
#define I2C_SDA SDApin

// Indicator Pins

int const BlueLED = 4;
int const WhiteLED = 13;
int const BuzzerPin = 3;

//SD card chip select pin
int const SD_CS_PIN = 5;


#endif // PINS_H