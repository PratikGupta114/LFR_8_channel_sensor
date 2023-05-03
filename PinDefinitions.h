
#include <Arduino.h>
#include <Config.h>

#ifndef PINDEFINITIONS_H
#define PINDEFINITIONS_H

// Define necessary headers
#define SENSOR_COUNT 8

// COURTESY : (Checking selected board type at compile time) https://arduino.stackexchange.com/a/21257
// Define motor driver pins and other peripherals
#define LEFT_MOTOR_PIN_1 10
#define LEFT_MOTOR_PIN_2 9

#define RIGHT_MOTOR_PIN_1 7
#define RIGHT_MOTOR_PIN_2 6

#define LEFT_MOTOR_PWM_PIN 11
#define RIGHT_MOTOR_PWM_PIN 5

#define BLUE_LED 4
#define WHITE_LED 3

#if (USE_I2C_ADAPTER == 1)
// In this case, we do not need to define any other pins (unless required)
#else
// Now let's check if the sensor count is 8
#if (SENSOR_COUNT != 8)
#error "Sensor Count should be 8"
#else
// Check if the setup is configured to use analog readings
#if (USE_ANALOG_READINGS == 1)

// If the board type is arduino uno then raise an error
#if defined(ARDUINO_AVR_UNO)
#error "Arduino Uno cannot be used for reading analog values of 8 channel sensors"
#else
// Otherwise define all analog sensor the pins
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6
#define S8 A7
#endif // #if defined(ARDUINO_AVR_UNO)

#else
// define digital sensor pins
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 2
#define S8 8

#endif // #if (USE_ANALOG_READINGS == 1)

#endif // #if (SENSOR_COUNT != 8)

#endif // #if (USE_I2C_ADAPTER == 1)
#endif // root