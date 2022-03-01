from networktables import NetworkTables as NT
import yaml, time
import matplotlib.pyplot as plt

with open("constants.yaml") as file:
    constants=yaml.load(file, Loader=yaml.CLoader)

NT.initialize(server=constants["RobotAddress"]) #roboRIO-447-frc.local
print("Connecting")
end=time.time()+30
while not NT.isConnected():
    time.sleep(0.1)
    if time.time()>=end:
        raise
print("Connected")

Y=[]
Y2=[]
X=[]
def main():
    global Y, Y2, X
    ltime=0
    stime=NT.getEntry(constants["NTTime"]).value/1000
    while True:
        time.sleep(0.01)
        mtime=NT.getEntry(constants["NTTime"]).value/1000-stime
        if mtime==ltime:
            continue
        ltime=mtime
        Y.append(NT.getEntry(constants["NTGetShooterEncoder"]).value)
        Y2.append(NT.getEntry(constants["NTShooterTargetSpeed"]).value)
        X.append(mtime)
    
try:
    main()
finally:
    plt.figure(num=1,figsize=[12,6])
    plt.plot(X,Y,color=constants["GraphEncoderColor"])
    plt.plot(X,Y2,color=constants["GraphSetpointColor"])
    plt.grid()
    plt.show()
input("Press Enter To Finish")