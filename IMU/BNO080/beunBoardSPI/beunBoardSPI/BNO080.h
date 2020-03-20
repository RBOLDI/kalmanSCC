/*
 * BNO080.h
 *
 * Created: 2/6/2020 3:59:01 PM
 *  Author: Rowan
 */ 


#ifndef BNO080_H_
#define BNO080_H_

#define F_CPU 2000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "avr_compiler.h"
#include "spi_driver.h"
#include "serialF0.h"

#define NUM_BYTES			4
#define ID_REQUEST_BYTES	2
//Registers
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_SET_FEATURE_COMMAND	0xFD
#define SHTP_REPORT_BASE_TIMESTAMP		0xFB
//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ME_CALIBRATE 7

#define MAX_PACKET_SIZE		128

#define REPORT_ID_INDEX	5
//BNO080 pins
#define _INT	PIN0_bm
#define _WAKE	PIN6_bm
#define _RST	PIN1_bm

#define HIGH	1
#define LOW		0

void setBNO080pins();
uint8_t BNO080BeginSPI();
uint8_t BNO080waitForSPI(void);
uint8_t initBNO080();
bool BNO080sendPacket(uint8_t channelNumber, uint8_t dataLength);
void BNO080setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
bool BNO080receivePacket(void);
void BNO080parseInputReport(void);
void BNO080enableLinearAccelerometer(uint16_t timeBetweenReports);
void BNO080parseCommandReport(void);
bool BNO080dataAvailable(void);
//acceleration components
float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
float BNO080getLinAccelX(void);
float BNO080getLinAccelY(void);
float BNO080getLinAccelZ(void);
uint8_t BNO080getLinAccelAccuracy(void);
int CheckPinLevel(PORT_t* PortCheck, uint8_t pinCheck);


#endif /* BNO080_H_ */