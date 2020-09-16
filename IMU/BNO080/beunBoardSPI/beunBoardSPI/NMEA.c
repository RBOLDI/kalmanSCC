/*
* NMEA.c
*
* Created: 21-2-2018 10:58:16
*  Author: de beste ooit A.K.A.
*
@@@@@@@(     &@@@&@@@@   .@&     %@@      @@     @@@      .@@@     .@%         %@#/@@@             @@@@@@@#    @@@@@@@@       /@@&      .@@@.       /@@@
@@    %@*   @@       @@   @@     @%@(    &@,    @@.@%     .@#@@    .@%         @@    @@            @@    %@,   @@     @@      @((@*     .@%@@      .@/@@
@@    @@   @@         @@   @@   &@ @@    @%    &@  %@,    .@# @@   .@%          @@/@@.             @@    @@    @@    /@%     @@  @@     .@% @@     @& @@
@@@@@@     @@         @@   %@   @/  @%  @@    *@,   @@    .@#  @@. .@%         &@@@@    @@         @@@@@@@@    @@@@@@/      &@    @@    .@% .@&   @@  @@
@@   #@/   @@        .@%    @@ &@   &@  @*    @@@@@@@@@   .@#   &@(.@%        @@   &@& #@*         @@     @@   @@    @@    ,@@@@@@@@#   .@#  (@/ @@   @@
@@    &@.   @@      .@@     /@.@/    @&@@    @@      (@(  .@#    %@%@%        @@     %@@.          @@     @@   @@     @@   @@      @@.  .@#   @@%@    @@
@@     @@    %@@@@@@@,       @@@     #@@.   %@,       @@  .@#     ,@@%         @@@@@@@..@@@        @@@@@@@%    @@     *@% @@        @@  .@#    @@*    @@
*/
#define F_CPU     2000000UL
#define TRUE	1
#define	FALSE	0
#define NMEACHECKSUMSIGN '*'
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include "beunBoard_controlSerial.h"
#include "NMEA.h"

static char sLongitude[STRLENGPS];
static char sLatitude[STRLENGPS];
static char sSnelheidGPS[STRLENGPS];
static char sGPSrichting[STRLENGPS];
static char sGPSstatus[STRLENGPS];
static char sGPStijd[STRLENGPS];
static char sCorrectieCC[STRLENGPS];
static volatile char received_data;
static char NMEA_str[80];
static volatile uint8_t adress = 0;
static volatile int string_complete = 0;
static float latitude;
static float longitude;
float speed_kmH (char speed[STRLENGPS]);

//nmea struct
struct NMEA
{
	char nmeatype[STRLENGPS];
	char time[STRLENGPS];
	char status[STRLENGPS];
	char latitude[STRLENGPS];
	char NorS[STRLENGPS];
	char longitude[STRLENGPS];
	char EorW[STRLENGPS];
	char speed[STRLENGPS];
	char angle[STRLENGPS];
	char date[STRLENGPS];
	char magnetic_var[STRLENGPS];
	char magnetic_dir[STRLENGPS];
	char mode_indicator[STRLENGPS];
};
struct NMEA NMEA_use;

//Fixed locations structs
typedef struct {
	double lat;
	double lon;
} tRMC;

static long int CCDistance;
//Update data in case of a snapshot
int TakeSnapshotGPS(const char *subadress){
	int err = 1;
	sprintf(sLongitude, "%s", NMEA_use.longitude);
	sprintf(sLatitude,  "%s", NMEA_use.latitude);
	sprintf(sSnelheidGPS, "%.1f", speed_kmH(NMEA_use.speed));
	sprintf(sGPStijd, "%s", NMEA_use.time);
	sprintf(sGPSrichting, "%s", NMEA_use.angle);
	sprintf(sGPSstatus, "%s", NMEA_use.status);
	sprintf(sCorrectieCC, "%ld", CCDistance);
	return err;
}

int GetLongLatData_GPS(const char *subadress, char *printbuf, int maxChars)
{
	int err = 1;
	if(!strcmp(subadress,"LL01"))
	{
		if((snprintf(printbuf, maxChars -strlen(printbuf),"%s,%s", sLongitude, sLatitude )) >= maxChars) printbuf[0] = '\0';
		else err = 0;
	}
	return err;
}
int GetDirectionData_GPS(const char *subadress, char *printbuf, int maxChars)
{
	int err = 1;
	if(!strcmp(subadress,"GR01"))
	{
		if((snprintf(printbuf, maxChars -strlen(printbuf),"%s", sGPSrichting)) >= maxChars) printbuf[0] = '\0';
		else err = 0;
	}
	return err;
}
int GetTimeData_GPS(const char *subadress, char *printbuf, int maxChars){
	int err = 1;
	if(!strcmp(subadress,"GT01")){
		if((snprintf(printbuf, maxChars -strlen(printbuf),"%s", sGPStijd)) >= maxChars) printbuf[0] = '\0';
		else err = 0;
	}
	return err;
}

int GetStatusData_GPS(const char *subadress, char *printbuf, int maxChars){
	int err = 1;
	if(!strcmp(subadress,"GS01")){
		if((snprintf(printbuf, maxChars -strlen(printbuf),"%s", sGPSstatus)) >= maxChars) printbuf[0] = '\0';
		else err = 0;
	}
	return err;
}

int GetSpeedData_GPS(const char *subadress, char *printbuf, int maxChars){
	int err = 1;
	if(!strcmp(subadress,"SG01")){
		if((snprintf(printbuf, maxChars -strlen(printbuf),"%.1f", speed_kmH(NMEA_use.speed))) >= maxChars)	printbuf[0] = '\0';
		else err = 0;
	}
	return err;
}

int GetCorrectieData(const char *subadress, char *printbuf, int maxChars){
	int err = 1;
	if(!strcmp(subadress,"CC01")){
		if((snprintf(printbuf, maxChars -strlen(printbuf),"%ld", CCDistance)) >= maxChars) printbuf[0] = '\0';
		else err = 0;
	}
	return err;
}

//NMEA checksum check
static int checkChecksum(char str[])
{
	uint8_t calChecksum = 0;
	uint8_t coordinateChecksum = 1;			// 1 since the first character of the NMEA string, the "$", doesn't count in the checksum

	while (str[coordinateChecksum] != '\n'  && str[coordinateChecksum] != NMEACHECKSUMSIGN) // until it encounters a checksum sign the values are exor-ed
	{																						// the \n bit is as a failsafe so it won't stay in the while loop in case
		calChecksum ^= str[coordinateChecksum++];											// the checksum sign is never found
	}

	if (str[coordinateChecksum] == NMEACHECKSUMSIGN) // if this is not true then the previous action has failed and the function will return false
	{
		coordinateChecksum++;			//this is where the checksum begins

		char *endPtr;
		int Csum = strtol(str + coordinateChecksum, &endPtr, 16); // strtol can convert the hex value of the sum to decimal so it will match the calculated sum

		if (endPtr == &str[coordinateChecksum + 2]);		// the checksum is always 2 characters so if this is not true the function has failed and returns false
		{
			if (Csum == calChecksum) return TRUE;	// if the calculated checksum and the checksum in the received string are the same return true
		}
	}
	return FALSE;
}

//Load nmea data in to struct strings
void load_nmeadata (void)
{
	//points to all variables in the struct
	char *fields[14] = {NMEA_use.nmeatype, NMEA_use.time, NMEA_use.status, NMEA_use.latitude, NMEA_use.NorS, NMEA_use.longitude,
		NMEA_use.EorW, NMEA_use.speed, NMEA_use.angle, NMEA_use.date, NMEA_use.magnetic_var, NMEA_use.magnetic_dir,
	NMEA_use.mode_indicator};
	char *here = NMEA_str;	//points to the first character of the NMEA-string
	int j = 0;
	
	for (int i = 0; i < 13; i ++)
	{
		while(*here != ',' && *here != '\0' && j < STRLENGPS && *here != NMEACHECKSUMSIGN)	//j represents the maximum length of the string in the struct
		{
			(fields[i])[j++] = *here++;		//fills struct-string i with the characters of NMEA_str with *here
		}
		(fields[i])[j] = '\0';		//when the whileloop isn't true, this makes sure that the last character of the string is \0

		if(*here == ',') //if the while loop isn't true and the character pointed to is a comma then:
		{
			*here++; //move to the next character after the comma
			j = 0;	//fill struct-string i from point j=0 again (start filling aray from 0 again)
		}
	}
}

static double decimal_lat_or_long(char *input)
{
	char min_str[10];
	int degrees = 0, i = 0;
	char *minPtr = input, *degrPtr = input; //minPtr and degrPtr are the now both the first value of NMEA longitude/latitude
	float decimal;
	while (*minPtr != '.' && *minPtr != '\0') minPtr++; //find the dot in their string

	minPtr = minPtr-2; //both longitude and latitude start their min 2 positions away from the dot

	while (degrPtr != minPtr) {	//as long as the degrPtr isn't in the same position as minPtr, the number pointed to is part of the degree
		degrees = (10 * degrees) + *degrPtr - '0';
		degrPtr++;
	}

	while (*minPtr != '\0') min_str[i++] = *minPtr++; //fill the min_str
	min_str[i] = '\0';
	decimal = (float)(atof(min_str) / 60) + degrees; //decimal longitude or latitude is degrees + min/60

	return decimal;
}

//Converts the GPS-speed from knots to km/H
float speed_kmH (char speed[STRLENGPS])
{
	float kmH = atof(speed)*1.852;
	return kmH;
}
//print NMEA data to debug datalogger
void printAll_UART(void){
	longitude	= (NMEA_use.EorW[0] == 'W') ? -decimal_lat_or_long(NMEA_use.longitude) : decimal_lat_or_long(NMEA_use.longitude); //If West then longitude is negative
	latitude	= (NMEA_use.NorS[0] == 'S') ? -decimal_lat_or_long(NMEA_use.latitude)  : decimal_lat_or_long(NMEA_use.latitude);  //If South then latitude is negative
	printf(">08|03: %s,%s,%s,%s,%s,%s,%s,%.1f,%s,%s,%s,%s,%s,%s<",
	NMEA_use.nmeatype,
	NMEA_use.time,
	NMEA_use.status,
	NMEA_use.latitude,
	NMEA_use.NorS,
	NMEA_use.longitude,
	NMEA_use.EorW,
	speed_kmH(NMEA_use.speed),
	NMEA_use.angle,
	NMEA_use.date,
	NMEA_use.magnetic_var,
	NMEA_use.magnetic_dir,
	NMEA_use.mode_indicator
	);
	printf("\r\n");
}
//Read from the circular GPS buffer
static uint8_t ReadLineGPS( char *Data )
{
	static char getKarakter;
	uint8_t len, returnValue = 0;
	//Check if you can Read data
	while( (CanRead_Ctrl() != 0) && (returnValue == 0) ){
		getKarakter = (char )ReadByte_Ctrl();
		if( getKarakter == NMEASENTENCE ){		//Check if it's a NMEA string
			Data[0] = getKarakter;
			Data[1] = '\0';
		}
		else if( Data[0] == NMEASENTENCE ){		//If the first character of the collected data is an $, continue collecting
			len = strlen(Data);
			if(len < MAXCHARACTERSSENTENCE - 1) {
				Data[len] = getKarakter;
				Data[len + 1] = '\0';
				if( getKarakter == '\n' || '\0' ){	//Check if you're at the end of a sentence, if yes then returnvalue = 1 to get out of the wile loop.
					returnValue = 1;
				}
				else{
					returnValue = 0;
				}
			}
		}
	}
	return returnValue;
}

//NMEA_ready4use puts the functions of NMEA.c in the correct order and makes sure they're useful
void NMEA_ready4use (void)
{
	//Check if there is a new NMEA sentence
	int done = ReadLineGPS(NMEA_str);
	if (done)
	{
		//Check if it's an RMC type sentence
		if ((!strncmp(NMEA_str,"$GPRMC",6))||(!strncmp(NMEA_str,"$GNRMC",6)))
		{
			//check if it's a valid NMEA sentence by checking the checksum
			if (checkChecksum(NMEA_str) == TRUE)
			{
				load_nmeadata();
				printAll_UART(); //print all NMEA data to the GPS debug line for the datalogger
				/*
				PORTB.OUTTGL = PIN3_bm;
				//Update current location
				cur_posPtr->lat = decimal_lat_or_long(NMEA_use.latitude);
				cur_posPtr->lon = decimal_lat_or_long(NMEA_use.longitude);
				//Compare your location with the fixed locations for cruise control
				Point_Ptr = &Point1;
				if (Location_cmp(Point_Ptr, cur_posPtr))
				{
					PORTD.DIRSET = 0b11110001;
					PORTD.OUTTGL = 0b11110001;
				}
				printf("Bearing = %.5f \n\r", Bearing(cur_posPtr, Point_Ptr));
				Point_Ptr = &Point2;
				if (Location_cmp(Point_Ptr, cur_posPtr))
				{
					PORTD.DIRSET = 0b11110001;
					PORTD.OUTTGL = 0b11110001;
				}
				*/
			}
		}
	}
}