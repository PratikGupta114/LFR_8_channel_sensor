#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <Arduino.h>
#include "PinDefinitions.h"
#include "Config.h"
#include "Defaults.h"
// #include "GlobalVariables.h"
#include "MotorControl.h"
#include "Sensor.h"
#include "Wire.h"

#if BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1
#include <ArduinoJson.h>
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#endif

#define TX_DOC_MAX_DATA_LEN 192
#define RX_DOC_MAX_DATA_LEN 64
#define BLUETOOTH_SERIAL Serial

#if BLUETOOTH_LOGGING_ENABLED == 1
StaticJsonDocument<TX_DOC_MAX_DATA_LEN> txDoc;
#endif
#if BLUETOOTH_TUNING_ENABLED == 1
StaticJsonDocument<RX_DOC_MAX_DATA_LEN> rxDoc;
#endif

// #define MID_6_SENSORS_DETECT_LINE_COLOR (s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1 && s6 == 1 && s7 == 1)
#define ALL_SENSORS_DETECT_LINE_COLOR(R) (R == 0b11111111)
#define ALL_SENSORS_OUT_OF_LINE_COLOR(R) (R == 0b00000000)

#define MID_6_SENSORS_DETECT_LINE_COLOR(R) ((R & 0b01111110) == 0b01111110)

// Macro function definitions

int Kp = DEFAULT_KP;
int Ki = DEFAULT_KI;
int Kd = DEFAULT_KD;
int baseMotorSpeed = DEFAULT_MOTOR_SPEED;
int loopDelay = DEFAULT_LOOP_DELAY;
int error = 0;
int leftMotorOffset = 0;
int rightMotorOffset = 0;
int P = 0;
int I = 0;
int D = 0;
int error_dir = 0;
int previousError = 0;
int PID_value = 0;
uint8_t isInverted = BLACK_LINE_WHITE_TRACK;

void indicateOn()
{
	digitalWrite(BLUE_LED, HIGH);
}

void indicateOff()
{
	digitalWrite(BLUE_LED, LOW);
}

void readSensors()
{

	uint8_t sensorData = getSensorReadings();
	error = getCalculatedError(0);

	// left most sensor value
	int s1 = (sensorData & (1 << 7)) >> 7;

#if BLUETOOTH_LOGGING_ENABLED == 1

	// The following variables shall only be used for printing output via Serial
	int s2 = (sensorData & (1 << 6)) >> 6;
	int s3 = (sensorData & (1 << 5)) >> 5;
	int s4 = (sensorData & (1 << 4)) >> 4;
	int s5 = (sensorData & (1 << 3)) >> 3;
	int s6 = (sensorData & (1 << 2)) >> 2;
	int s7 = (sensorData & (1 << 1)) >> 1;

#endif

	// right most sensor value
	int s8 = (sensorData & (1 << 0)) >> 0;

	if (s1 != s8)
		error_dir = s1 - s8;

	// This else block bypasses the fallback error value of 255
	if (ALL_SENSORS_OUT_OF_LINE_COLOR(sensorData))
	{
		// Moved out of the line
		if (error_dir < 0)
			error = OUT_OF_LINE_ERROR_VALUE;
		else if (error_dir > 0)
			error = -1 * OUT_OF_LINE_ERROR_VALUE;
	}
	else if (ALL_SENSORS_DETECT_LINE_COLOR(sensorData))
	{
		moveStraight(baseMotorSpeed, baseMotorSpeed);
		delay(STOP_CHECK_DELAY);
		uint8_t sensorDataAgain = getSensorReadings();
		if (ALL_SENSORS_DETECT_LINE_COLOR(sensorDataAgain))
		{
			shortBrake(100);
			stop();
			delay(10000);
		}
	}

	if (MID_6_SENSORS_DETECT_LINE_COLOR(sensorData))
		indicateOn();
	else
		indicateOff();

	if (isInverted == BLACK_LINE_WHITE_TRACK)
		digitalWrite(WHITE_LED, HIGH);
	else if (isInverted == WHITE_LINE_BLACK_TRACK)
		digitalWrite(WHITE_LED, LOW);

#if BLUETOOTH_LOGGING_ENABLED == 1

	char readingString[12];
	char jsonOutputBuffer[TX_DOC_MAX_DATA_LEN];

	sprintf(readingString, "%d%d%d%d%d%d%d%d", s1, s2, s3, s4, s5, s6, s7, s8);

	txDoc["di"] = String(readingString);
	serializeJson(txDoc, jsonOutputBuffer);
	BLUETOOTH_SERIAL.println(jsonOutputBuffer);

	BLUETOOTH_SERIAL.print("I|Value : ");
	BLUETOOTH_SERIAL.print(String(readingString));
	BLUETOOTH_SERIAL.print(" | Error : ");
	BLUETOOTH_SERIAL.print(error);
	BLUETOOTH_SERIAL.print(" i-");
	BLUETOOTH_SERIAL.println(isInverted);
	txDoc.clear();

#endif
}

void calculatePID()
{
	P = error;
	I = (error = 0) ? 0 : I + error;
	I = constrain(I, -200, 200);
	D = error - previousError;
	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	PID_value = constrain(PID_value, -150, 150);
	previousError = error;
}

void controlMotors()
{
	if (error == OUT_OF_LINE_ERROR_VALUE)
	{
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif
		uint8_t sensorReadings = getSensorReadings();
		while (isOutOfLine(sensorReadings))
		{
			turnCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
			sensorReadings = getSensorReadings();
		}
#if GAPS_ENABLED == 1
		error_dir = 0;
#endif
	}
	else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE))
	{
#if BRAKING_ENABLED == 1
		shortBrake(BRAKE_DURATION_MILLIS);
#endif
		uint8_t sensorReadings = getSensorReadings();
		while (isOutOfLine(sensorReadings))
		{
			turnCCW(baseMotorSpeed - leftMotorOffset, baseMotorSpeed - rightMotorOffset);
			sensorReadings = getSensorReadings();
		}
#if GAPS_ENABLED == 1
		error_dir = 0;
#endif
	}
	else
	{
		int leftMotorSpeed = baseMotorSpeed + PID_value - leftMotorOffset;
		int rightMotorSpeed = baseMotorSpeed - PID_value - rightMotorOffset;

		moveStraight(leftMotorSpeed, rightMotorSpeed);

		if (D != 0)
			delay(loopDelay);
	}
}

void setup()
{
	pinMode(LEFT_MOTOR_PIN_1, OUTPUT);
	pinMode(LEFT_MOTOR_PIN_2, OUTPUT);
	pinMode(RIGHT_MOTOR_PIN_1, OUTPUT);
	pinMode(RIGHT_MOTOR_PIN_2, OUTPUT);
	pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
	pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

	pinMode(BLUE_LED, OUTPUT);
	pinMode(WHITE_LED, OUTPUT);

#if (USE_I2C_ADAPTER == 1)
	Wire.setClock(400000);
	Wire.begin();
#else
	pinMode(S1, INPUT);
	pinMode(S2, INPUT);
	pinMode(S3, INPUT);
	pinMode(S4, INPUT);
	pinMode(S5, INPUT);
	pinMode(S6, INPUT);
	pinMode(S7, INPUT);
	pinMode(S8, INPUT);
#endif

#if BLUETOOTH_LOGGING_ENABLED == 1 || BLUETOOTH_TUNING_ENABLED == 1
	BLUETOOTH_SERIAL.begin(115200);
#endif
}

void loop()
{

#if BLUETOOTH_TUNING_ENABLED == 1

	if (BLUETOOTH_SERIAL.available())
	{
		BLUETOOTH_SERIAL.flush();
		String data = BLUETOOTH_SERIAL.readStringUntil('\n');
		DeserializationError error = deserializeJson(rxDoc, data);

		if (error)
		{
			// Serial.print("E|deseriaize json failed : ");
			// Serial.println(error.f_str());
			return;
			rxDoc.clear();
		}

		Kp = rxDoc["P"];			  // 0
		Ki = rxDoc["I"];			  // 0
		Kd = rxDoc["D"];			  // 0
		baseMotorSpeed = rxDoc["ms"]; // 255
		loopDelay = rxDoc["de"];	  // 100
		rxDoc.clear();
		// char buff[64];
		// sprintf(buff, "P : %d | I : %d | D : %d | ms : %d | de : %d", P, I, D, ms, de);
		// Serial.println(buff);
	}

#endif

	readSensors();
	calculatePID();
	controlMotors();
}