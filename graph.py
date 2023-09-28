import matplotlib.pyplot as plt
import serial

iterations = 6000

#setPoint = 1000

serialPort = serial.Serial("/dev/ttyUSB0",115200)

ysA = [] 
ysB = [] 
xs = [] 

#limites do gráfico

for i in range(iterations):
    line = serialPort.readline() #entrada como string
    line_as_list = line.split(b' ')
    
    A_speed = float(line_as_list[0][1:])
    B_speed = float(line_as_list[1])
    currentTime = float(line_as_list[2][0:6])
    

    ysA.append(A_speed)
    ysB.append(B_speed)
    xs.append(currentTime)

plt.xlim([xs[0], xs[iterations - 1]])
plt.ylim([ysA[0] - 5, ysA[iterations - 1] + 5])




#plotando as linhas de interesse
plt.axhline(y = 2000, color = 'green', linestyle = '-') #setPoint
plt.axhline(y = -2000, color = 'green', linestyle = '-') #setPoint


plt.plot(xs,ysA[0:iterations], color = 'red') #comportamento do motor em velocidade ao longo do tempo
plt.plot(xs,ysB[0:iterations], color = 'blue') #comportamento do motor em velocidade ao longo do tempo

plt.show()