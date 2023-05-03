#pragma once

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

uint8_t getSensorReadings();
int getCalculatedError(int fallbackError);
int isOutOfLine(uint8_t sensorReadings);

#endif