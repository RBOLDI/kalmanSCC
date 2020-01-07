import csv

f_GSA = open("Master_GSA_","r")
f_RMC = open("Master","r")

GSA_line = f_GSA.readline()
RMC_line = f_RMC.readline()

with open('output/kalmanSet.csv', mode='w+') as kalmanSet:
    kalmanWriter = csv.writer(kalmanSet , delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    kalmanWriter.writerow(['Time', 'Lat', 'Long', 'Theta', 'PDOP', 'V'])
    while GSA_line and RMC_line:
        #GSA verwerking
        GSA_split = GSA_line.split()
        GSA_sen = GSA_split[1].split(',')
        GSA_PDOP = GSA_sen[15]
        #RMC verwerking
        RMC_line = RMC_line.split()
        latitude = RMC_line[6]
        pos = latitude.find('.')
        try:
            RMC_time = RMC_line[1]
            RMC_speed = RMC_line[10]
            RMC_angle = RMC_line[11]
            minutes = float(latitude[pos-2:])
            degrees = float(latitude[:pos-2])
            RMC_lat = degrees + round((minutes/60),5)
        except:
            print("Not a valid line")
        longitude = RMC_line[8]
        pos = longitude.find('.')
        try:
            minutes = float(longitude[pos-2:])
            degrees = float(longitude[:pos-2])
            RMC_lon = degrees + round((minutes/60),5)
        except:
            print("Not a valid line")
        kalmanWriter.writerow([RMC_time, RMC_lat, RMC_lon, RMC_angle, GSA_PDOP, RMC_speed])
        GSA_line = f_GSA.readline()
        RMC_line = f_RMC.readline()

f_GSA.close()
f_RMC.close()
