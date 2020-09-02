import matplotlib.pyplot as plt

MasterLat = []
MasterLon = []
KalmanLat = []
KalmanLon = []

f_kSet = open("ksetOutput.csv","r")
kSet_line = f_kSet.readline()
while kSet_line:
    kSet_line = kSet_line.split(',')
    index = int(kSet_line[4])
    if index > 150:
        MasterLat.append(float(kSet_line[0]))
        MasterLon.append(float(kSet_line[1]))
        KalmanLat.append(float(kSet_line[2]))
        KalmanLon.append(float(kSet_line[3]))
    kSet_line = f_kSet.readline()

#PLot
plt.plot(MasterLon, MasterLat, label='non-filtered')
plt.plot(KalmanLon, KalmanLat, label='Kalman filtered')
plt.legend()
plt.title('Kalman filtered vs. non-filtered')
plt.show()

f_kSet.close
