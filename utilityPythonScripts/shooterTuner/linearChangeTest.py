from networktables import NetworkTables as NT
import time, yaml
import matplotlib.pyplot as plt, threading

def thread():
    global con, line
    Y=[]
    Y2=[]
    X=[]
    ltime=0
    stime=NT.getEntry(constants["NTTime"]).value/1000
    while con:
        time.sleep(0.01)
        mtime=NT.getEntry(constants["NTTime"]).value/1000-stime
        if mtime==ltime:
            continue
        ltime=mtime
        Y.append(NT.getEntry(constants["NTGetShooterEncoder"]).value)
        Y2.append(NT.getEntry(constants["NTShooterTargetSpeed"]).value)
        X.append(mtime)
        if line != None:
            plt.axvline(x=mtime,label=line,color=constants["GraphVlineColor"])
            plt.text(mtime-0.1,0,line,rotation=90)
            line=None
    plt.figure(num=1,figsize=[12,6])
    plt.plot(X,Y,color=constants["GraphEncoderColor"])
    plt.plot(X,Y2,color=constants["GraphSetpointColor"])
    plt.grid()
    plt.show()
con=True
line=None

with open("constants.yaml") as file:
    constants=yaml.load(file, Loader=yaml.CLoader)
constants["FFNumbOfSteps"]=int((constants["FFMaxSpeed"]-constants["FFMinSpeed"])/constants["FFStep"])

NT.initialize(server=constants["RobotAddress"]) #roboRIO-447-frc.local

end=time.time()+30
while not NT.isConnected():
    time.sleep(0.1)
    if time.time()>=end:
        raise

print("starting")
t = threading.Thread(target=thread)
t.start()

vmin=.3
vmax=1
tTime=2
m=(vmin-vmax)/tTime

NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(vmin)
time.sleep(5)

starttime=time.time()
endtime=starttime+tTime*2

while time.time()<endtime:
    NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(abs(time.time()-starttime-tTime)*m+vmax)
    #print(abs(time.time()-starttime-tTime)*m+vmax)
    time.sleep(0.01)
NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(0)

con=False
input()
