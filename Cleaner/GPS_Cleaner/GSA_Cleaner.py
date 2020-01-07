raw = open("dataRaw/ttyS2", "r")
clean = open("dataClean/Master_GSA_", "w+")
decimal = 0
dataLoggerTime = 0.000

line = raw.readline()
while(line):
    pos = line.find('$')
    if line[pos+1:pos+6] == "GNGSA":
        RTKline = line[pos:]
        parseLine = line.split()
        decimal = int(parseLine[1], 16)
        dataLoggerTime = decimal/32768
        dataLoggerTime = round(dataLoggerTime, 4)
        writeStr = "{} {}".format(dataLoggerTime, RTKline)
        print(writeStr)
        clean.write(writeStr)
    line = raw.readline()
raw.close()
clean.close()
