#include <Arduino.h>
#include "SPI.h"
#include <nRF24L01.h>
#include <RF24.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "RTClib.h"

// #define SERIAL_DEBUGGING
// #define POWER_VIA_USB
// #define DEBUGGING_VALUES
// #define RADIO
#define HIGHER_TIMER_FREQ
// #define USE_INTERRUPTS_FOR_BUTTONS
#define WATER_AFTER_BOOT
#define CTRL_WATER_TIME_VIA_POTI

const int ledWaterPin = 0;
const int ledRefillPin = 1;
const int btnRefillPin = 2;
const int btnWateringPin = 3;
const int btnLimitPin = 4;
const int pumpPin = 5;
const int tftLedPin = 6;
const int tftCsPin = 7;
const int tftDcPin = 8;
const int nrf24CePin = 9;
const int nrf24CsnPin = 10;
const int mosiPin = 11;
const int misoPin = 12;
const int spiClkPin = 13;
const int moistureSensor1Pin = A0;
const int moistureSensor2Pin = A1;
const int moistureSensorPowerPin = A2;
const int potiPin = A3;

#ifdef DEBUGGING_VALUES
const int moistureMeasuringIntervalInSeconds = 20; // every 20 seconds
#else
const int moistureMeasuringIntervalInSeconds = 60 * 60; // every hour
#endif
const int pumpWarmupPeriod = 250;
const int minWateringPeriodSeconds = 1;		  // 1 second
const int maxWateringPeriodSeconds = 15 * 60; // 15 minutes
const int refillCheckDelay = 1500;
const int minPumpVal = 130;
const int numMoistureReadings = 5;
const int moistureSettleInterval = 100;

const int tftWidth = 320;
const int tftHeight = 240;
const int tftHeaderTextSize = 2;
const int tftTimeTextSize = 1;
const int tftValueTextSize = 4;
const int tftMargin = 3;
const int tftTimeBarHeight = 14;

uint32_t lastClockUpdate = 0L;
float currentPumpVoltage = 12.0;
uint32_t lastMoistureMeasurement = 0L;
uint32_t wateringStart = 0L;
uint32_t nextWateringTime = 0L;
#ifndef RADIO
#ifdef DEBUGGING_VALUES
uint32_t wateringIntervalInSeconds = 60L; // once per minute
uint32_t pumpWateringPeriod = 2 * 1000L;  // 2 seconds
#else
uint32_t wateringIntervalInSeconds = 86400L;	 // once per day
uint32_t currentWateringPeriod = 2 * 60 * 1000L; // 2 minutes
#endif
#endif
bool nextWateringValid = false;
bool watering = false;
bool tankIsEmpty = false;
bool displayIsOn = false;

volatile bool tankRefillWasPressed = false;
uint32_t lastTankRefillPress = 0L;
volatile bool manualWateringWasPressed = false;
uint32_t lastManualWateringPress = 0L;
const int btnDebouncingPeriod = 1000;

const int TANK_UNDECIDED = 0;
const int TANK_FULL = 1;
const int TANK_EMPTY = 2;
int currentTankStatus = TANK_UNDECIDED;
const int minPumpValForTankDecision = 100;

#ifdef POWER_VIA_USB
const float tankFullSlope = 4.902077801266418;
const float tankFullIntercept = -122.62657410119039 - 300.0;
const float tankEmptySlope = 1.779161939543686;
const float tankEmptyIntercept = 133.7240951609847 - 300.0;
#else
const float tankFullSlope = 4.217734374999999;
const float tankFullIntercept = -115.65226562499993;
const float tankEmptySlope = 2.2445312499999996;
const float tankEmptyIntercept = 69.94453125000007;
#endif
const float displayCurrentConsumption = 80.0;

#ifdef RADIO
RF24 radio(nrf24CePin, nrf24CsnPin);
const uint8_t receivingAddress[] = "water";
const uint8_t sendingAddress[] = "wwitc";
#endif

Adafruit_ILI9341 tft = Adafruit_ILI9341(tftCsPin, tftDcPin);

Adafruit_INA219 ina219;

RTC_DS3231 ds3231;
DateTime datetime;

void initTft();
void handleRadioMsg(uint32_t nowUnix);
void logMoistureValue();
void updatePotiValue();
void initWatering(bool manual);
void wateringLoop(const float progress);
void endWatering();
void updateCurrentTankStatus(int currentPumpVal);
void checkIfTankWasRefilled();
void updateClockAndWateringTimeAndMoistureBar(DateTime now, uint32_t nextWateringTime);
void sendViaRadio(const char *msg);
void updateTftStatus(const char *status);
void updateTftStatusProgress(const float progress);
void updateTftNextWatering(const char *nextWatering);
void updateTftNextWateringProgress(const float progress);
void updateTftMoisture(const char *moistureLevel);
void updateTftMoistureProgress(const float progress);
void updateTftPoti(const char *potiValue);
void updateTftPotiBar(const float bar);
void updateTftTime(const char *timeStr);
void updateTftCurrent(const char *currentStr);
void tankRefillISR();
void manualWateringISR();

void setup()
{
#ifdef HIGHER_TIMER_FREQ
	TCCR0B = (TCCR0B & 0b11111000) | 0x02;
#endif

#ifdef SERIAL_DEBUGGING
	Serial.begin(9600);
	Serial.println("Begin setup");
#else
	pinMode(ledRefillPin, OUTPUT);
	pinMode(ledWaterPin, OUTPUT);
#endif
	pinMode(btnRefillPin, INPUT_PULLUP);
	pinMode(btnWateringPin, INPUT_PULLUP);
	pinMode(btnLimitPin, INPUT_PULLUP);
	pinMode(pumpPin, OUTPUT);
	pinMode(tftLedPin, OUTPUT);
	pinMode(moistureSensor1Pin, INPUT);
	pinMode(moistureSensor2Pin, INPUT);
	pinMode(moistureSensorPowerPin, OUTPUT);
	pinMode(potiPin, INPUT);

#ifdef SERIAL_DEBUGGING
	Serial.println("Setup: init Tft");
#endif
	initTft();

#ifdef RADIO
#ifdef SERIAL_DEBUGGING
	Serial.println("Setup: init radio");
#endif
	radio.begin();
	radio.setChannel(0);
	if (!radio.setDataRate(RF24_250KBPS))
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Warning: Could not set radio data rate successful.");
#endif
	}
	radio.setAutoAck(true);
	// radio.enableDynamicPayloads();
	radio.setCRCLength(RF24_CRC_16);
	radio.setPALevel(RF24_PA_LOW);
	radio.setPayloadSize(32);
	// radio.setRetries(15, 15);
	if (!radio.isPVariant())
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Warning: The radio is not a nRF24L01+ radio.");
#endif
	}
	radio.openWritingPipe(sendingAddress);
	radio.openReadingPipe(1, receivingAddress);
	radio.startListening();
#endif

#ifdef SERIAL_DEBUGGING
	Serial.println("Setup: init ina219");
#endif
	ina219.begin();

#ifdef SERIAL_DEBUGGING
	Serial.println("Setup: init ds3231");
#endif
	ds3231.begin();

#ifdef SERIAL_DEBUGGING
	Serial.println("Setup: init interrupts");
#endif
#ifdef USE_INTERRUPTS_FOR_BUTTONS
	attachInterrupt(digitalPinToInterrupt(btnRefillPin), tankRefillISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(btnWateringPin), manualWateringISR, FALLING);
#endif

#ifndef SERIAL_DEBUGGING
	digitalWrite(ledRefillPin, LOW);
	digitalWrite(ledWaterPin, LOW);
#endif
	digitalWrite(moistureSensorPowerPin, LOW);
	digitalWrite(tftLedPin, LOW);

	const char *statusStr = "Idle";
	updateTftStatus(statusStr);

#ifdef WATER_AFTER_BOOT
	manualWateringWasPressed = true;
#endif

#ifdef SERIAL_DEBUGGING
	Serial.println("Setup: start loop");
#endif
}

void initTft()
{
	tft.begin();
	tft.setRotation(3);
	tft.fillScreen(ILI9341_BLACK);

	tft.fillRect(0, 0, tftWidth / 2, tftHeight / 2 - tftTimeBarHeight / 2, ILI9341_DARKGREEN);
	tft.fillRect(tftWidth / 2, 0, tftWidth / 2, tftHeight / 2 - tftTimeBarHeight / 2, ILI9341_RED);
	tft.fillRect(0, tftHeight / 2 - tftTimeBarHeight / 2, tftWidth / 2, tftHeight / 2 - tftTimeBarHeight / 2, ILI9341_BLUE);
	tft.fillRect(tftWidth / 2, tftHeight / 2 - tftTimeBarHeight / 2, tftWidth / 2, tftHeight / 2 - tftTimeBarHeight / 2, ILI9341_PURPLE);

	tft.setTextColor(ILI9341_WHITE);
	tft.setTextSize(tftHeaderTextSize);
	tft.setCursor(tftMargin, tftMargin);
	tft.println("STATUS");
	tft.setCursor(tftWidth / 2 + tftMargin, tftMargin);
	tft.println("NEXT WATERING");
	tft.setCursor(tftMargin, tftHeight / 2 + tftMargin - tftTimeBarHeight / 2);
	tft.println("MOISTURE");
	tft.setCursor(tftWidth / 2 + tftMargin, tftHeight / 2 + tftMargin - tftTimeBarHeight / 2);
#ifdef CTRL_WATER_TIME_VIA_POTI
	tft.println("WATERING TIME");
#else
	tft.println("PUMP VOLTAGE");
#endif

	tft.setTextSize(tftValueTextSize);
	tft.setCursor(tftMargin, tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println("Init");
	tft.setCursor(tftWidth / 2 + tftMargin, tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println("Init");
	tft.setCursor(tftMargin, 3 * tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println("Init");
	tft.setCursor(tftWidth / 2 + tftMargin, 3 * tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println("Init");

	tft.setTextSize(tftTimeTextSize);
	tft.setCursor(tftMargin, tftHeight + tftMargin - tftTimeBarHeight);
	tft.println("23.06.1993 07:00");
	tft.setCursor(tftWidth / 2 + tftMargin, tftHeight + tftMargin - tftTimeBarHeight);
	tft.println("0.0mA");

	updateTftStatusProgress(0.0);
}

uint32_t freq_millis()
{
#ifdef HIGHER_TIMER_FREQ
	return (uint32_t)millis() / 8;
#else
	return (uint32_t)millis();
#endif
}

void freq_delay(int ms)
{
#ifdef HIGHER_TIMER_FREQ
	delay(ms * 8);
#else
	delay(ms);
#endif
}

void loop()
{
	DateTime now = ds3231.now();
	uint32_t nowUnix = now.unixtime();

#ifdef RADIO
	if (radio.available())
	{
		handleRadioMsg(nowUnix);
	}
#else
	if (!nextWateringValid)
	{
		nextWateringValid = true;
		nextWateringTime = nowUnix + wateringIntervalInSeconds;
	}
#endif

	if (nowUnix - lastClockUpdate > 1)
	{
		lastClockUpdate = nowUnix;
		updateClockAndWateringTimeAndMoistureBar(now, nextWateringTime);
	}

	if (nowUnix - lastMoistureMeasurement > moistureMeasuringIntervalInSeconds)
	{
		lastMoistureMeasurement = nowUnix;
		logMoistureValue();
	}

	if (nextWateringValid && nowUnix >= nextWateringTime)
	{
		nextWateringValid = false;
		initWatering(false);
	}

	updatePotiValue();

	if (watering)
	{
		const long currentWateringMillis = freq_millis() - wateringStart;
#ifdef SERIAL_DEBUGGING
		Serial.print("currentWateringMillis=");
		Serial.println(currentWateringMillis);
#endif
		if (currentWateringMillis < currentWateringPeriod)
		{
			const float progress = (float)currentWateringMillis / currentWateringPeriod;
			wateringLoop(progress);
		}
		else
		{
			endWatering();
		}
	}

#ifndef USE_INTERRUPTS_FOR_BUTTONS
	if (digitalRead(btnRefillPin) == LOW)
	{
		tankRefillWasPressed = true;
	}
#endif

	if (tankRefillWasPressed)
	{
		tankRefillWasPressed = false;
		if (freq_millis() - lastTankRefillPress > btnDebouncingPeriod)
		{
			lastTankRefillPress = freq_millis();
#ifdef SERIAL_DEBUGGING
			Serial.println("Refill button was pressed.");
#endif
			if (tankIsEmpty)
			{
				checkIfTankWasRefilled();
			}
		}
		tankRefillWasPressed = false;
	}

#ifndef USE_INTERRUPTS_FOR_BUTTONS
	if (digitalRead(btnWateringPin) == LOW)
	{
		manualWateringWasPressed = true;
	}
#endif

	if (manualWateringWasPressed)
	{
		manualWateringWasPressed = false;
		if (freq_millis() - lastManualWateringPress > btnDebouncingPeriod)
		{
			lastManualWateringPress = freq_millis();
#ifdef SERIAL_DEBUGGING
			Serial.println("Watering button was pressed.");
#endif
			if (watering)
			{
				endWatering();
			}
			else
			{
				initWatering(true);
			}
		}
		manualWateringWasPressed = false;
	}

	if (!displayIsOn && digitalRead(btnLimitPin) == LOW)
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Limit button is low -> turn TFT on.");
#endif
		displayIsOn = true;
		digitalWrite(tftLedPin, HIGH);
	}

	if (displayIsOn && digitalRead(btnLimitPin) == HIGH)
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Limit button is high -> turn TFT off.");
#endif
		displayIsOn = false;
		digitalWrite(tftLedPin, LOW);
	}

	freq_delay(5);
}

#ifdef RADIO
void handleRadioMsg(uint32_t nowUnix)
{
	int len = 32;
#ifdef SERIAL_DEBUGGING
	Serial.print("Radio received message with length=");
	Serial.print(len);
#endif
	char command[len + 1];
	radio.read(&command, len);
	// radio.flush_rx();
	command[len + 1] = 0;
#ifdef SERIAL_DEBUGGING
	Serial.print(" and payload=");
	Serial.println(command);
	Serial.println("Confirm command.");
#endif
	char logMsg[32];
	sprintf(logMsg, "[c] %.28s", command);
	sendViaRadio(logMsg);

	/*Serial.print("strlen(command) = ");
	Serial.println(strlen(command));
	Serial.print("15 <= strlen(command) = ");
	Serial.println(15 <= strlen(command));
	Serial.print("strncmp('[nextWatering] ', command, 15) == 0 = ");
	Serial.println(strncmp("[nextWatering] ", command, 15) == 0);*/

	if ((15 <= strlen(command)) && (strncmp("[nextWatering] ", command, 15) == 0))
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Process nextWatering command.");
#endif
		uint32_t nextWateringReceived;
		sscanf(command, "%*s %ld", &nextWateringReceived);
#ifdef SERIAL_DEBUGGING
		Serial.print("Next watering unix time: ");
		Serial.println(nextWateringReceived);
		Serial.print("Current unix time: ");
		Serial.println(nowUnix);
#endif
		bool rtiitp = nextWateringReceived <= nowUnix;
#ifdef SERIAL_DEBUGGING
		Serial.println("rtiitp=");
		Serial.println(rtiitp);
#endif
		if (rtiitp)
		{
			Serial.println("if...");
		}
		else
		{
			Serial.println("else...");
			// int i = 0;
			// bool testNextWateringValid = true;
			// uint32_t testNextWateringTime = nextWateringReceived;
			// Serial.println(testNextWateringValid);
			// Serial.println(testNextWateringTime);
		}

		/*if (rtiitp)
		{
#ifdef SERIAL_DEBUGGING
			Serial.println("Ignore received watering time, since it is in the past.");
#endif
		}
		else
		{
			//nextWateringValid = true;
			//nextWateringTime = nextWateringReceived;
#ifdef SERIAL_DEBUGGING
			Serial.print("Update next watering time to ");
			Serial.println(nextWateringTime);
#endif
		}*/
	}
	/*else if ((7 <= strlen(command)) && (strncmp("[time] ", command, 7) == 0))
	{
		uint32_t currentTime;
		sscanf(command, "%*s %ld", &currentTime);
		ds3231.setDateTime(currentTime);
#ifdef SERIAL_DEBUGGING
		Serial.print("Received current unix time: ");
		Serial.println(currentTime);
#endif
	}
	else if ((8 <= strlen(command)) && (strncmp("[query] ", command, 8) == 0))
	{
		char *commandPtr = command;
		commandPtr += 8;
		if(strncmp("status", commandPtr, 6) == 0)
		{
#ifdef SERIAL_DEBUGGING
			Serial.println("Received query for status.");
#endif
			if(tankIsEmpty){
				const char msg[] = "[tank] empty";
				sendViaRadio(msg);
			} else if(watering) {
				const char msg[] = "[watering] start";
				sendViaRadio(msg);
			} else {
				const char msg[] = "[watering] end";
				sendViaRadio(msg);
			}
		}
		else
		{
#ifdef SERIAL_DEBUGGING
			Serial.println("Unknown query.");
#endif
		}
	}
	else
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Unknown command.");
#endif
	}*/
}
#endif

void logMoistureValue()
{
	digitalWrite(moistureSensorPowerPin, HIGH);
	freq_delay(moistureSettleInterval);
	int moistureSensor1Value = 0;
	int moistureSensor2Value = 0;
	for (int readCount = 0; readCount < numMoistureReadings; readCount++)
	{
		moistureSensor1Value += analogRead(moistureSensor1Pin);
		moistureSensor2Value += analogRead(moistureSensor2Pin);
		freq_delay(5);
	}
	digitalWrite(moistureSensorPowerPin, LOW);
	moistureSensor1Value /= numMoistureReadings;
	moistureSensor2Value /= numMoistureReadings;
#ifdef SERIAL_DEBUGGING
	Serial.print("Moisture 1 value=");
	Serial.print(moistureSensor1Value);
	Serial.print("\tMoisture 2 value=");
	Serial.println(moistureSensor2Value);
#endif
	char msg[20];
	sprintf(msg, "%d", (moistureSensor1Value + moistureSensor2Value) / 2);
	updateTftMoisture(msg);
	updateTftMoistureProgress(0.0);
#ifdef RADIO
	char logMsg[100];
	sprintf(logMsg, "[moisture] %d %d", moistureSensor1Value, moistureSensor2Value);
	sendViaRadio(logMsg);
#endif
}

void updatePotiValue()
{
	int potiAnalogValue = analogRead(potiPin);
	float potiRelativeValue = 1.1 * potiAnalogValue / 1024.0;

#ifdef CTRL_WATER_TIME_VIA_POTI
	int minValue = minWateringPeriodSeconds;
	int maxValue = maxWateringPeriodSeconds;
#else
	float minValue = 12.0 * minPumpVal / 255.0;
	float maxValue = 12.0;
#endif

	float value = (maxValue - minValue) * potiRelativeValue + minValue;
	if (value > maxValue)
	{
		potiRelativeValue = 1.0;
		value = maxValue;
	}

#ifdef CTRL_WATER_TIME_VIA_POTI
	uint32_t wateringPeriod = value * 1000L;
	bool valueChanged = abs(currentWateringPeriod - wateringPeriod) > 5000L;
#else
	bool valueChanged = abs(currentPumpVoltage - value) > 0.05;
#endif

	if (valueChanged)
	{
		char potiStr[50];
#ifdef CTRL_WATER_TIME_VIA_POTI
		int wateringPeriodMinutes = wateringPeriod / (60 * 1000L);
		int wateringPeriodSeconds = (wateringPeriod / 1000L) % 60;
		sprintf(potiStr, "%dm%ds", wateringPeriodMinutes, wateringPeriodSeconds);
		currentWateringPeriod = wateringPeriod;
#else
		int voltageInt = int(value);
		int voltageDec = int(value * 100) % 100;
		sprintf(potiStr, "%d.%dV", voltageInt, voltageDec);
		currentPumpVoltage = value;
#endif
		updateTftPoti(potiStr);
		updateTftPotiBar(potiRelativeValue);
	}
}

void initWatering(bool manual)
{
	if (!manual && tankIsEmpty)
	{
		checkIfTankWasRefilled();
	}
	if (tankIsEmpty)
	{
#ifdef RADIO
		const char *msg = "[watering] skip";
		sendViaRadio(msg);
#endif
#ifdef SERIAL_DEBUGGING
		Serial.println("Skip watering due to empty tank");
#endif
	}
	else
	{
		if (manual)
		{
			updateTftStatus("mWater");
		}
		else
		{
			updateTftStatus("Water");
		}
#ifdef RADIO
		if (manual)
		{
			const char *msg = "[watering] manual";
			sendViaRadio(msg);
		}
		else
		{
			const char *msg = "[watering] start";
			sendViaRadio(msg);
		}
#endif
#ifdef SERIAL_DEBUGGING
		Serial.println("Start pump warm up period");
#else
		digitalWrite(ledWaterPin, HIGH);
#endif
		analogWrite(pumpPin, 255);
		freq_delay(pumpWarmupPeriod);
#ifdef SERIAL_DEBUGGING
		Serial.println("Start watering period");
#endif
		updateTftStatusProgress(0.0);
		wateringStart = freq_millis();
		watering = true;
	}
}

void wateringLoop(const float progress)
{
	updateTftStatusProgress(progress);
	int pumpValue = 255.0 * currentPumpVoltage / 12.0;
	if (pumpValue > 252)
	{
		pumpValue = 255;
	}
#ifdef SERIAL_DEBUGGING
	Serial.print("Pump value=");
	Serial.println(pumpValue);
#endif
	analogWrite(pumpPin, pumpValue);
	updateCurrentTankStatus(pumpValue);
	if (currentTankStatus != TANK_FULL)
	{
		tankIsEmpty = true;
#ifdef SERIAL_DEBUGGING
		Serial.println("Tank is empty");
#else
		digitalWrite(ledRefillPin, HIGH);
#endif
		endWatering();
#ifdef RADIO
		const char *msg = "[tank] empty";
		sendViaRadio(msg);
#endif
		const char *statusStr = "Empty";
		updateTftStatus(statusStr);
	}
}

void endWatering()
{
	analogWrite(pumpPin, 0);
	watering = false;
#ifdef SERIAL_DEBUGGING
	Serial.println("End watering period");
#else
	digitalWrite(ledWaterPin, LOW);
#endif
	updateTftStatus("Idle");
	updateTftStatusProgress(0.0);
#ifdef RADIO
	const char *msg = "[watering] end";
	sendViaRadio(msg);
#endif
}

void updateCurrentTankStatus(int currentPumpVal)
{
	float currentMeasurement = 0.0;
	for (int i = 0; i < 5; i++)
	{
		currentMeasurement += ina219.getCurrent_mA() / 5.0;
		freq_delay(5);
	}
	float tankFullThres = tankFullSlope * currentPumpVal + tankFullIntercept;
	float tankEmptyThres = tankEmptySlope * currentPumpVal + tankEmptyIntercept;

	char currentStr[100];
	int currentInt = int(currentMeasurement);
	int currentDec = int((currentMeasurement - currentInt) * 10);
	int emptyInt = int(tankEmptyThres);
	int fullInt = int(tankFullThres);
	sprintf(currentStr, "C:%d.%dmA E:%dmA F:%dmA ", currentInt, currentDec, emptyInt, fullInt);

	updateTftCurrent(currentStr);

	if (!displayIsOn)
	{
		currentMeasurement += displayCurrentConsumption;
	}

	if (currentPumpVal < minPumpValForTankDecision)
	{
		currentTankStatus = TANK_UNDECIDED;
	}
	else if ((currentTankStatus != TANK_FULL) && (currentMeasurement > tankFullThres))
	{
		currentTankStatus = TANK_FULL;
	}
	else if ((currentTankStatus != TANK_EMPTY) && (currentMeasurement < tankEmptyThres))
	{
		currentTankStatus = TANK_EMPTY;
	}
}

void checkIfTankWasRefilled()
{
	updateTftStatus("Check");
	analogWrite(pumpPin, 255);
	freq_delay(refillCheckDelay);
	updateCurrentTankStatus(255);
	if (currentTankStatus != TANK_FULL)
	{
		updateTftStatus("Empty");
#ifdef SERIAL_DEBUGGING
		Serial.println("Tank was not refilled");
#endif
	}
	else
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Tank was refilled");
#endif
		tankIsEmpty = false;
#ifndef SERIAL_DEBUGGING
		digitalWrite(ledRefillPin, LOW);
#endif
		const char *statusStr = "Idle";
		updateTftStatus(statusStr);
#ifdef RADIO
		const char *msg = "[tank] refilled";
		sendViaRadio(msg);
#endif
	}
	analogWrite(pumpPin, 0);
}

void updateClockAndWateringTimeAndMoistureBar(DateTime now, uint32_t nextWateringTime)
{
	uint32_t secondsTillNextWatering = nextWateringTime - now.unixtime();
	char nextWateringStr[50];
	if (nextWateringTime >= now.unixtime())
	{
		uint32_t minutesTillNextWatering = secondsTillNextWatering / 60;
		if (minutesTillNextWatering > 0)
		{
			uint32_t hoursTillNextWatering = minutesTillNextWatering / 60;
			if (hoursTillNextWatering > 0)
			{
				int leftMinutesTillNextWatering = minutesTillNextWatering % 60;
				sprintf(nextWateringStr, "%ldh%d", hoursTillNextWatering, leftMinutesTillNextWatering);
			}
			else
			{
				int leftSecondsTillNextWatering = secondsTillNextWatering % 60;
				sprintf(nextWateringStr, "%ldm%d", minutesTillNextWatering, leftSecondsTillNextWatering);
			}
		}
		else
		{
			sprintf(nextWateringStr, "%lds", secondsTillNextWatering);
		}
	}
	else
	{
		sprintf(nextWateringStr, "nsy");
	}
	updateTftNextWatering(nextWateringStr);
	char timeStr[100];
	sprintf(timeStr, "%d-%d-%d %d:%d:%d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
	updateTftTime(timeStr);

	float moistureProgress = (float)(now.unixtime() - lastMoistureMeasurement) / moistureMeasuringIntervalInSeconds;
	updateTftMoistureProgress(moistureProgress);
}

#ifdef RADIO
void sendViaRadio(const char msg[])
{
	char msgCopy[32];
	strncpy(msgCopy, msg, 32);
#ifdef SERIAL_DEBUGGING
	Serial.print("Send message via radio: '");
	Serial.print(msgCopy);
	Serial.print("' <-- strlen = ");
	Serial.println(strlen(msgCopy));

#endif
	// Serial.println("Send message via radio was not successful.");
	radio.stopListening();
	if (!radio.write(&msgCopy, strlen(msgCopy)))
	{
#ifdef SERIAL_DEBUGGING
		Serial.println("Send message via radio was not successful.");
#endif
	}
	radio.startListening();
}
#endif

void updateTftStatus(const char status[])
{
#ifdef SERIAL_DEBUGGING
	Serial.print("TFT: set status to ");
	Serial.println(status);
#endif
	tft.fillRect(0, tftHeight / 4 - 3.5 * tftValueTextSize, tftWidth / 2, 8 * tftValueTextSize, ILI9341_DARKGREEN);
	tft.setTextSize(tftValueTextSize);
	tft.setCursor(tftMargin, tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println(status);
}

void updateTftStatusProgress(const float progress)
{
#ifdef SERIAL_DEBUGGING
	Serial.print("TFT: set status progress to ");
	Serial.println(progress);
#endif
	int progessbarWidth = progress * tftWidth / 2;
	tft.fillRect(0, tftHeight / 2 - tftTimeBarHeight / 2 - 10, progessbarWidth - 1, 5, ILI9341_WHITE);
	tft.fillRect(progessbarWidth + 1, tftHeight / 2 - tftTimeBarHeight / 2 - 10, tftWidth / 2 - progessbarWidth - 1, 5, ILI9341_DARKGREEN);
}

void updateTftNextWatering(const char nextWatering[])
{
#ifdef SERIAL_DEBUGGING
//	Serial.print("TFT: set next watering to ");
//	Serial.println(nextWatering);
#endif
	tft.fillRect(tftWidth / 2, tftHeight / 4 - 3.5 * tftValueTextSize, tftWidth / 2, 8 * tftValueTextSize, ILI9341_RED);
	tft.setTextSize(tftValueTextSize);
	tft.setCursor(tftWidth / 2 + tftMargin, tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println(nextWatering);
}

void updateTftNextWateringProgress(const float progress)
{
#ifdef SERIAL_DEBUGGING
	Serial.print("TFT: set next watering progress to ");
	Serial.println(progress);
#endif
	int progessbarWidth = progress * tftWidth / 2;
	tft.fillRect(tftWidth / 2, tftHeight / 2 - tftTimeBarHeight / 2 - 10, progessbarWidth - 1, 5, ILI9341_WHITE);
	tft.fillRect(tftWidth / 2 + progessbarWidth + 1, tftHeight / 2 - tftTimeBarHeight / 2 - 10, tftWidth / 2 - progessbarWidth - 1, 5, ILI9341_RED);
}

void updateTftMoisture(const char moistureLevel[])
{
#ifdef SERIAL_DEBUGGING
	Serial.print("TFT: set moisture to ");
	Serial.println(moistureLevel);
#endif
	tft.fillRect(0, 3 * tftHeight / 4 - 3.5 * tftValueTextSize, tftWidth / 2, 8 * tftValueTextSize, ILI9341_BLUE);
	tft.setTextSize(tftValueTextSize);
	tft.setCursor(tftMargin, 3 * tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println(moistureLevel);
}

void updateTftMoistureProgress(const float progress)
{
#ifdef SERIAL_DEBUGGING
	Serial.print("TFT: set moisture progress to ");
	Serial.println(progress);
#endif
	int progessbarWidth = progress * tftWidth / 2;
	tft.fillRect(0, tftHeight - tftTimeBarHeight - 10, progessbarWidth - 1, 5, ILI9341_WHITE);
	tft.fillRect(progessbarWidth + 1, tftHeight - tftTimeBarHeight - 10, tftWidth / 2 - progessbarWidth - 1, 5, ILI9341_BLUE);
}

void updateTftPoti(const char potiStr[])
{
#ifdef SERIAL_DEBUGGING
//	Serial.print("TFT: set poti str to ");
//	Serial.println(potiStr);
#endif
	tft.fillRect(tftWidth / 2, 3 * tftHeight / 4 - 3.5 * tftValueTextSize, tftWidth / 2, 8 * tftValueTextSize, ILI9341_PURPLE);
	tft.setTextSize(tftValueTextSize);
	tft.setCursor(tftWidth / 2 + tftMargin, 3 * tftHeight / 4 - 3.5 * tftValueTextSize);
	tft.println(potiStr);
}

void updateTftPotiBar(const float bar)
{
#ifdef SERIAL_DEBUGGING
	Serial.print("TFT: set poti bar to ");
	Serial.println(bar);
#endif
	int barWidth = bar * tftWidth / 2;
	tft.fillRect(tftWidth / 2, tftHeight - tftTimeBarHeight - 10, barWidth - 1, 5, ILI9341_WHITE);
	tft.fillRect(tftWidth / 2 + barWidth + 1, tftHeight - tftTimeBarHeight - 10, tftWidth / 2 - barWidth - 1, 5, ILI9341_PURPLE);
}

void updateTftTime(const char timeStr[])
{
#ifdef SERIAL_DEBUGGING
//	Serial.print("TFT: set time to ");
//	Serial.println(timeStr);
#endif
	tft.fillRect(0, tftHeight - tftTimeBarHeight, tftWidth / 2, tftTimeBarHeight, ILI9341_BLACK);
	tft.setTextSize(tftTimeTextSize);
	tft.setCursor(tftMargin, tftHeight - tftTimeBarHeight + tftMargin);
	tft.println(timeStr);
}

void updateTftCurrent(const char *currentStr)
{
#ifdef SERIAL_DEBUGGING
//	Serial.print("TFT: set current to ");
//	Serial.println(currentStr);
#endif
	tft.fillRect(tftWidth / 2, tftHeight - tftTimeBarHeight, tftWidth, tftTimeBarHeight, ILI9341_BLACK);
	tft.setTextSize(tftTimeTextSize);
	tft.setCursor(tftMargin + tftWidth / 2, tftHeight - tftTimeBarHeight + tftMargin);
	tft.println(currentStr);
}

void tankRefillISR()
{
	tankRefillWasPressed = true;
}

void manualWateringISR()
{
	manualWateringWasPressed = true;
}
