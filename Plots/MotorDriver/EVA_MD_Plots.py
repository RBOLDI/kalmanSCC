import matplotlib.pyplot as plt

MD_left = open("MD-LEFT", "r")
MD_right = open("MD-RIGHT", "r")
#Full power knop state
FP_state = []
state = 0
laststate = 0
#Vermogen
Power = []
PowerR = []
energie = 0.000
#Cruise control knop state
CC_state = []
#Snelheid
Speed = []
SpeedR = []
Speed_left = 0.000
Speed_right = 0.000
Average = 0.000
msec = 0.000
afstand = 0.000
#DataLogger tijd
DataLoggerTime = []
TimeR = []
beginTime = 0.000
endTime = 0.000
number = 0

leftLine = MD_left.readline()
rightLine = MD_right.readline()

while leftLine and rightLine:
    leftLine = leftLine.split()
    rightLine = rightLine.split()
    FP_state.append(int(leftLine[23]))
    state = int(leftLine[23])
    CC_state.append(int(leftLine[25]))
    Power.append(float(leftLine[10])/10)
    PowerR.append(float(rightLine[10])/10)
    Speed_left = float(leftLine[19])
    Speed_right = float(rightLine[19])
    Average = float((Speed_left + Speed_right)/2)
    Speed.append(float(Speed_left))
    SpeedR.append(float(Speed_right))
    DataLoggerTime.append(float(leftLine[1]))
    TimeR.append(float(rightLine[1]))
    if state == 1:
        laststate = 1
        msec = float(Speed_left/3.6)
        afstand = afstand + msec
        energie = energie + float(leftLine[10])
    elif laststate == 1:
        endTime = float(leftLine[1])
        energie = float(energie/(endTime - beginTime))
        afstand = float(afstand/(endTime - beginTime))
        number = number + 1
        print("{}: energie: {} J tijd: {} sec afstand: {} m".format(number, energie, (endTime - beginTime), afstand))
        energie = 0
        afstand = 0
        laststate = 0
    else:
        beginTime = float(leftLine[1])

    leftLine = MD_left.readline()
    rightLine = MD_right.readline()

MD_left.close()
MD_right.close()

plt.plot(DataLoggerTime, FP_state, label = 'FullPower state')
plt.plot(DataLoggerTime, CC_state, label = 'CruiseControl state')
plt.plot(DataLoggerTime, Speed, label = 'Speed L')
plt.plot(TimeR, SpeedR, label = 'Speed R')
plt.plot(DataLoggerTime, Power, label = 'Power/10 L')
plt.plot(TimeR, PowerR, label = 'Power/10 R')
plt.legend()
plt.xlabel("DataLoggerTime")
plt.show()
