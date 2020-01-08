import csv

raw = open("dataRaw/ttyS2", "r")
line = raw.readline()

with open('output/kalmanSet.csv', mode='w+') as kalmanSet:
    kalmanWriter = csv.writer(kalmanSet , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    kalmanWriter.writerow(['Time', 'Lat', 'Long', 'Theta', 'PDOP', 'V'])
    while(line):
        line = line.split()
        lineGPS = line[2]
        #Check for GPS lines
        if lineGPS[0] == '$':
            #Check for GSA sentence
            if lineGPS[1:6] == "GNGSA":
                lineGSA = lineGPS.split(',')
                PDOP = lineGSA[15]
                #Mark flag
                GSAflag = 1
            #Check for RMC sentence
            if lineGPS[1:6] == "GNRMC":
                lineRMC = lineGPS.split(',')
                time = lineRMC[1]
                speed = lineRMC[7]
                angle = lineRMC[8]
                #Calc latitude in degrees
                latitude = lineRMC[3]
                pos = latitude.find('.')
                minutes = float(latitude[pos-2:])
                degrees = float(latitude[:pos-2])
                latitude = degrees + round((minutes/60),5)
                #Calc longitude in degrees
                longitude = lineRMC[5]
                pos = longitude.find('.')
                minutes = float(longitude[pos-2:])
                degrees = float(longitude[:pos-2])
                latitude = degrees + round((minutes/60),5)
                #Mark flag
                RMCflag = 1
        #Check for datalogger debugline, end of the GPS data sample, RMC and GSA are from the same sample
        if lineGPS[0] == '>' and GSAflag == 1 and RMCflag == 1:
            kalmanWriter.writerow([time, latitude, longitude, angle, PDOP, speed])
        #Read new line
        line = raw.readline()

raw.close()

