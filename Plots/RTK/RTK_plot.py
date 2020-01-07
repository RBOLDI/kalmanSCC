from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
'''
m = Basemap(projection='mill',
            llcrnrlat = 52.3409,
            llcrnrlon = 4.7990,
            urcrnrlat = 52.3339,
            urcrnrlon = 4.8113)
m.drawcoastlines()
#m.bluemarble()
'''
RTK_lon_x = []
RTK_lat_y = []
Master_lon_x = []
Master_lat_y = []
RTK_time = []
Master_time = []
f_RTK = open("RTK_RMC_","r")
RTK_line = f_RTK.readline()
while RTK_line:
    RTK_split = RTK_line.split()
    RTK_sen = RTK_split[1].split(',')
    #Latitude decimal degree calc
    latitude = RTK_sen[3]
    pos = latitude.find('.')
    try:
        minutes = float(latitude[pos-2:])
        degrees = float(latitude[:pos-2])
        RTK_lat = degrees + round((minutes/60),5)
        RTK_lat_y.append(RTK_lat)
    except:
        print("Not a valid line")
    #Longitude decimal degree calc
    longitude = RTK_sen[5]
    pos = longitude.find('.')
    try:
        minutes = float(longitude[pos-2:])
        degrees = float(longitude[:pos-2])
        RTK_lon = degrees + round((minutes/60),5)
        RTK_lon_x.append(RTK_lon)
    except:
        print("Not a valid line")
    RTK_line = f_RTK.readline()
f_RTK.close()

f_Master = open("Master","r")
MasterLine = f_Master.readline()
while MasterLine:
    MasterLine = MasterLine.split()
    latitude = MasterLine[6]
    pos = latitude.find('.')
    try:
        minutes = float(latitude[pos-2:])
        degrees = float(latitude[:pos-2])
        Master_lat = degrees + round((minutes/60),5)
        Master_lat_y.append(Master_lat)
    except:
        print("Not a valid line")
    longitude = MasterLine[8]
    pos = longitude.find('.')
    try:
        minutes = float(longitude[pos-2:])
        degrees = float(longitude[:pos-2])
        Master_lon = degrees + round((minutes/60),5)
        Master_lon_x.append(Master_lon)
    except:
        print("Not a valid line")
    MasterLine = f_Master.readline()
#PLot
plt.plot(RTK_lon_x, RTK_lat_y, label='RTK')
plt.plot(Master_lon_x, Master_lat_y, label='Master')
plt.legend()
plt.title('RTK GPS vs Master GPS')
plt.show()

