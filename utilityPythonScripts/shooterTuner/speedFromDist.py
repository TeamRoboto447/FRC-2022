from networktables import NetworkTables as NT
from time import sleep
import numpy as np
import matplotlib.pyplot as plt


def main():
    NT.initialize(server="10.4.47.2")
    print("Please wait while the NetworkTable connects")
    sleep(3)

    table = NT.getTable('pidTuningPVs')
    distance = table.getEntry('distanceFromTarget')

    iteration = 0
    measurements = [[0, 0, 0], [0, 0, 0]]
    while(iteration <= 2):
        speed = float(input("What speed are you shooting?\n> "))
        print("Logging measurements..")
        measurements[0][iteration] = speed
        averageX = 0
        for i in range(20):
            averageX += distance.getDouble(0)
            sleep(0.05)
        averageX /= 20
        print(averageX)
        measurements[1][iteration] = averageX
        iteration += 1
    
    results = ""
    for i in range(len(measurements)+1):
        results += "Speed: {measurements[0][i]} | X: {measurements[1][i]}\n"

    x = np.array(measurements[1])
    y = np.array(measurements[0])
    b = estimateCoef(x, y)
    plotLine(x, y, b)
    print(f"Speed slope = ({b[1]} * X value) + {b[0]}")


def estimateCoef(x, y):
    # Number of values
    n = np.size(x)
    #print(f"X: {x}\nY:{y}")

    m_x, m_y = np.mean(x), np.mean(y)

    SS_xy = np.sum(y*x) - n*m_y*m_x
    SS_xx = np.sum(x*x) - n*m_x*m_x

    #print(f"SS_xy = {SS_xy}; SS_xx = {SS_xx}")
    b_1 = SS_xy / SS_xx
    b_0 = m_y - b_1*m_x
    
    return (b_0, b_1)

def plotLine(x, y, b):
    plt.scatter(x, y, color="m", marker="o", s=30)

    y_pred = (b[1]*x)+b[0]
    plt.plot(x, y_pred, color="g")

    plt.show()

if __name__ == '__main__':
    main()
    input()
