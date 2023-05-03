#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

void moveStraight(int leftMotorSpeed, int rightMotorSpeed);
void turnCCW(int leftMotorSpeed, int rightMotorSpeed);
void turnCW(int leftMotorSpeed, int rightMotorSpeed);
void shortBrake(int durationMillis);
void stop();

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)
void my_delay_ms(int ms);
#endif

#endif