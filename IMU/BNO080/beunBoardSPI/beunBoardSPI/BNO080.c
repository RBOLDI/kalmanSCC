/*
 * BNO080.c
 *
 * Created: 2/6/2020 3:58:45 PM
 *  Author: Rowan
 */ 

#include "BNO080.h"
#include "spi.h"
#include <avr/io.h>

uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	
//These are the raw sensor values (without Q applied) pulled from the user requested Input Report
uint32_t timeStamp;
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
uint8_t calibrationStatus;	

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
//See the read metadata example for more info
int16_t rotationVector_Q1 = 14;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;

/*! \brief SPI master module on PORT D. */
SPI_Master_t spiMasterD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;

/*! \brief Test data to send from master. */
uint8_t masterSendData[NUM_BYTES] = {0x11, 0x22, 0x33, 0x44};

/*! \BNO080 product ID Request. */
uint8_t BNO080_IDrequest[ID_REQUEST_BYTES] = {0xF9, 0x00};

/*! \brief Data received from slave. */
uint8_t masterReceivedData[32];

PORT_t *ssPort = &PORTD;

/*! \brief Result of the test. */
bool success = true;

int CheckPinLevel(PORT_t* PortCheck, uint8_t pinCheck) {
	int PortMask = PortCheck->IN;
	if((PortMask & pinCheck) == pinCheck) return 1;
	else return 0;
};

//BNO080 function:
void setBNO080pins(void);
uint8_t BNO080BeginSPI(void);
uint8_t BNO080waitForSPI(void);
uint8_t initBNO080(void);
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

void setBNO080pins(void){
	//The INT pin: input, acvtive low
	PORTA.DIRCLR	= _INT;			//PIN0
	PORTA.PIN0CTRL	= PORT_OPC_PULLUP_gc;
	//The RST pin: output, active low
	PORTA.DIRSET	= _RST;			//PIN1
	//PORTA.PIN1CTRL	= PORT_OPC_PULLUP_gc;
	//The PSO/Wake pin: output, needs to be high to enable SPI-mode serves as wake operation, active low.
	PORTA.DIRSET	= _WAKE;		//PIN6
	PORTA.PIN6CTRL	= PORT_OPC_PULLUP_gc;



}

uint8_t BNO080BeginSPI(void){
	setBNO080pins();
	spi_init();	//Turn on SPI hardware
	
	PORTD.OUTSET = SPI_SS_bm;	//Deselect BNO080
	
	//Configure the BNO080 for SPI communication
	PORTA.OUTSET = _WAKE;	//Before boot up the PS0/WAK pin must be high to enter SPI mode
	PORTA.OUTCLR = _RST;	//Reset BNO080
	_delay_ms(2);			//Wait 2 milisec
	PORTA.OUTSET = _RST;	//Bring out of reset
	
	//Wait for first assertion of INT before using WAK pin. Can take ~104ms
	printf("- 1st wait\n");
	BNO080waitForSPI();
	
	//At system startup, the hub must send its full advertisement message (see 5.2 and 5.3) to the
	//host. It must not send any other data until this step is complete.
	//When BNO080 first boots it broadcasts big startup packet
	//Read it and dump it
	printf("- 2nd wait\n");
	BNO080waitForSPI(); //Wait for assertion of INT before reading advert message.
	BNO080receivePacket();
	//The BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)
	//Read it and dump it
	printf("- 3rd wait\n");
	BNO080waitForSPI(); //Wait for assertion of INT before reading Init response
	BNO080receivePacket();
	
	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;
	
	//Transmit packet on channel 2, 2 bytes
	printf("-f BNO080sendPacket\n");
	BNO080sendPacket(CHANNEL_CONTROL, 2);
	
	//Now we wait for response
	printf("- await packet\n");
	BNO080waitForSPI();
	if (BNO080receivePacket())
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			printf("correct ID response!");
			return 1;
		}
		printf("ID response failed...\n");
	}
	return 0;
}

uint8_t BNO080waitForSPI(void){
	for (uint8_t counter = 0; counter < 125; counter++)
	{
		if (CheckPinLevel(&PORTA, _INT) == LOW)
		{
			printf("_INT == LOW\n");
			return 1;
		}
		_delay_ms(1);
	}
	//response failed
	printf("SPI INT timeout\n");
	return 0;
}

uint8_t initBNO080(void){
	if( BNO080BeginSPI()) {
		return 1;
	}
	else return 0;
}

bool BNO080sendPacket(uint8_t channelNumber, uint8_t dataLength){
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header
	//Wait for BNO080 to indicate it is available for communication
	PORTA.OUTCLR = _WAKE;
	//Check for wake response BNO080
	if(BNO080waitForSPI()) {
		printf("BNO080 is awake!\n");
		//Select the BNO080 and release from wake
		PORTD.OUTCLR = SPI_SS_bm;
		PORTA.OUTSET = _WAKE;
		printf("Host send packet to BNO080\n");
		//Send the 4 byte packet header
		spi_transfer((packetLength & 0xFF));
		spi_transfer((packetLength >> 8));
		spi_transfer(channelNumber);
		spi_transfer((sequenceNumber[channelNumber]++));
		for (uint8_t i = 0; i < dataLength; i++) {
			printf("shtpdata[%d]: %d \n",i, shtpData[i]);
			spi_transfer(shtpData[i]);
			//SPI_MasterTransmitByte(&spiMasterD,shtpData[i]);
		}
		//SPI_MasterSSHigh(ssPort, PIN4_bm);
		PORTD.OUTSET = SPI_SS_bm;
		uint16_t lenght = ((uint16_t) (packetLength >> 8)<< 8 | (packetLength & 0xFF));
		printf("-----Lenght = %d\n", lenght);
		
		return true;
	}
	PORTA.OUTSET = _WAKE;
	return false;
}

void BNO080setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig){
	printf("setFeatureCommand! \n");
	uint32_t microsBetweenReports = (uint32_t)timeBetweenReports * 1000L;
	
	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)
	
	//Transmit packet on channel 2, 17 bytes
	BNO080sendPacket(CHANNEL_CONTROL, 17);
}

bool BNO080receivePacket(void){

	if (!BNO080waitForSPI()){
		printf("nothing to receive...\n");
		return false;
	}
	printf("ready to receive!\n");
			
	//SPI_MasterSSLow(ssPort, PIN4_bm);
	PORTD.OUTCLR = SPI_SS_bm;
		
	//Get first four bytes to find out how much data we need to read
	uint8_t packetLSB = spi_transfer(FOO);//SPI_MasterReceiveByte(&spiMasterD);
	uint8_t packetMSB  = spi_transfer(FOO);//SPI_MasterReceiveByte(&spiMasterD);
	uint8_t channelNumber  = spi_transfer(FOO);//SPI_MasterReceiveByte(&spiMasterD);
	uint8_t sequenceNumber  = spi_transfer(FOO);//SPI_MasterReceiveByte(&spiMasterD);
	shtpHeader[0] = packetLSB;
	shtpHeader[1] = packetMSB;
	shtpHeader[2] = channelNumber;
	shtpHeader[3] = sequenceNumber;
	//Calculate the number of data bytes in this packet
	uint16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
	dataLength &= ~(1 << 15); //Clear the MSbit.
	if (dataLength == 0){
		printf("dataLength = 0...\n");
		PORTD.OUTSET = SPI_SS_bm;
		return false; //Packet is empty
	}
	dataLength -= 4; //Remove the header bytes from the data count
	//Read incoming data into the shtpData array
	uint8_t incoming = 0;
	for(uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++) {
		incoming = spi_transfer(FOO);//SPI_MasterReceiveByte(&spiMasterD);
		if (dataSpot < MAX_PACKET_SIZE) //BNO080 can respond with upto 270 bytes, avoid overflow
			shtpData[dataSpot] = incoming;
	}
	//SPI_MasterSSHigh(ssPort, PIN4_bm); //Release BNO080
	PORTD.OUTSET = SPI_SS_bm;
	printf("SHPT header 1: %d \n", shtpHeader[0]);
	printf("SHPT header 2: %d \n", shtpHeader[1]);
	printf("SHPT header 3: %d \n", shtpHeader[2]);
	printf("SHPT header 4: %d \n", shtpHeader[3]);
	printf("dataLength = %d\n", dataLength);
	return true;
}

void BNO080parseInputReport(void) {
	printf("Parse input report \n");
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	dataLength -= 4; //Remove the header bytes from the data count
	
	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));
	
	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[REPORT_ID_INDEX + 5] << 8 | shtpData[REPORT_ID_INDEX + 4];
	uint16_t data2 = (uint16_t)shtpData[REPORT_ID_INDEX + 7] << 8 | shtpData[REPORT_ID_INDEX + 6];
	uint16_t data3 = (uint16_t)shtpData[REPORT_ID_INDEX + 9] << 8 | shtpData[REPORT_ID_INDEX + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0;
	
	if (dataLength - 5 > 9)
		data4 = (uint16_t)shtpData[REPORT_ID_INDEX + 11] << 8 | shtpData[REPORT_ID_INDEX + 10];
	if (dataLength - 5 > 11)
		data5 = (uint16_t)shtpData[REPORT_ID_INDEX + 13] << 8 | shtpData[REPORT_ID_INDEX + 12];
	
	//Store these generic values to their proper global variable
	if (shtpData[REPORT_ID_INDEX] == SENSOR_REPORTID_ACCELEROMETER)
	{
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if (shtpData[REPORT_ID_INDEX] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		printf("Linear ACC!!\n");
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
}

void BNO080enableLinearAccelerometer(uint16_t timeBetweenReports){
	printf("Set Linear Accelerometer! \n");
	BNO080setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

void BNO080parseCommandReport(void) {
	printf("Parse command report \n");
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
}

bool BNO080dataAvailable(void){
	if(!bit_is_clear(PORTA.IN, PIN1_bm))
		return false;
	
	if (BNO080receivePacket() == true)
	{
		printf("Received packet!\n");
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP){
			BNO080parseInputReport();
			printf("Data available!\n");
			return (true);
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			BNO080parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
			return (true);
		}
	}
	return false;
}

float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}
//Return the acceleration component
float BNO080getLinAccelX(void) {
	float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
	return (accel);
}
//Return the acceleration component
float BNO080getLinAccelY(void)
{
	float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
float BNO080getLinAccelZ(void)
{
	float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
	return (accel);
}

//Return the acceleration component
uint8_t BNO080getLinAccelAccuracy(void)
{
	return (accelLinAccuracy);
}
