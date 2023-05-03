
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "Defaults.h"
#include "Sensor.h"
#include "PinDefinitions.h"
#include "Config.h"
#include "Wire.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

extern uint8_t isInverted;

// returns a byte in which each bit represents the state of each sensor
uint8_t getSensorReadings()
{
    uint8_t reading = 0x00;

    Wire.requestFrom(ADAPTER_ADDRESS, 1);
    if (Wire.available() >= 1)
        reading = Wire.read();

    // else if (reading == 0b01100000)
    //     isInverted = WHITE_LINE_BLACK_TRACK;
    // else if (reading == 0b01110000 || reading == 0b00100000)
    //     isInverted = WHITE_LINE_BLACK_TRACK;
    if (reading == 0b00110000)
        isInverted = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b00111000 || reading == 0b00010000)
        isInverted = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b00011000)
        isInverted = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b00011100 || reading == 0b00001000)
        isInverted = WHITE_LINE_BLACK_TRACK;
    else if (reading == 0b00001100)
        isInverted = WHITE_LINE_BLACK_TRACK;
    // else if (reading == 0b00001110 || reading == 0b00000100)
    //     isInverted = WHITE_LINE_BLACK_TRACK;
    // else if (reading == 0b00000110)
    //     isInverted = WHITE_LINE_BLACK_TRACK;

    // else if (reading == 0b10011111)
    //     isInverted = BLACK_LINE_WHITE_TRACK;
    // else if (reading == 0b10001111 || reading == 0b11011111)
    //     isInverted = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b11001111)
        isInverted = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b11000111 || reading == 0b11101111)
        isInverted = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b11100111)
        isInverted = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b11100011 || reading == 0b11110111)
        isInverted = BLACK_LINE_WHITE_TRACK;
    else if (reading == 0b11110011)
        isInverted = BLACK_LINE_WHITE_TRACK;
    // else if (reading == 0b11110001 || reading == 0b11111011)
    //     isInverted = BLACK_LINE_WHITE_TRACK;
    // else if (reading == 0b11111001)
    //     isInverted = BLACK_LINE_WHITE_TRACK;

    // invert the sensor readings
    if (isInverted == BLACK_LINE_WHITE_TRACK)
    {
        reading ^= 0b11111111;
    }

    return reading;
}

// the following function calculates and returns the error value
// based on the readings returned by getSensorReadings()
int getCalculatedError(int fallbackError)
{
    uint8_t sensorReading = getSensorReadings();
    int numeratorSum = 0, denominatorSum = 0;

    // Assuming that the the MSB represents the index 0 of the array (left to right)
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        // Check the digital values at the ith bit
        uint8_t sensorValue = ((sensorReading & (1 << (SENSOR_COUNT - 1 - i))) >> (SENSOR_COUNT - 1 - i));
        numeratorSum += (i + 1) * 100 * (int)sensorValue;
        denominatorSum += sensorValue;
    }

    int error = fallbackError;
    if (denominatorSum != 0)
        error = ((numeratorSum / (denominatorSum * 50)) - (SENSOR_COUNT + 1));
    return error;
}

int isOutOfLine(uint8_t sensorReadings)
{
    uint8_t options[] = {
        0b01100000,
        0b01110000,
        0b00110000,
        0b00111000,
        0b00011000,
        0b00011100,
        0b00001100,
        0b00001110,
        0b00000110,
        0b10011111,
        0b10001111,
        0b11001111,
        0b11000111,
        0b11100111,
        0b11100011,
        0b11110011,
        0b11110001,
        0b11111001};

    int n = sizeof(options) / sizeof(uint8_t);

    for (int i = 0; i < n; i++)
    {
        if (sensorReadings == options[i])
            return 0;
    }

    return 1;
}
