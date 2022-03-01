from networktables import NetworkTables as NT
import time, numpy as np
from progress.bar import ShadyBar
import os, yaml, msvcrt
import matplotlib.pyplot as plt, threading

speedM = 1

def regression(x,y):
    if len(x)!=len(y):
        raise ValueError("x and y should be the same length")
    n=len(x)
    b=(sum(y)*sum(X**2 for X in x)-sum(x)*sum(X*Y for X,Y in zip(x,y)))/(n*sum(X**2 for X in x)-sum(x)**2)
    m=(n*sum(X*Y for X,Y in zip(x,y))-sum(x)*sum(y))/(n*sum(X**2 for X in x)-sum(x)**2)
    return m,b
def rawtuning(time,number,prop,loopType):
    timePerOscil=time/number
    loopType=loopType.lower()
    if loopType=="p":
        return prop*0.5,0,0
    elif loopType=="pi":
        return prop*0.45,0.54*prop/timePerOscil,0
    elif loopType=="pid":
        return prop*0.6,1.2*prop/timePerOscil,3/40*timePerOscil*prop
    else:
        raise ValueError("Unknown loopType {}".format(loopType))
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
            plt.text(mtime,0,line,rotation=90)
            line=None
    plt.figure(num=1,figsize=[12,6])
    plt.plot(X,Y,color=constants["GraphEncoderColor"])
    plt.plot(X,Y2,color=constants["GraphSetpointColor"])
    plt.grid()
    plt.show()
def format_e(n):
    a = '%E' % n
    return a.split('E')[0].rstrip('0').rstrip('.') + 'e' + a.split('E')[1]
con=True
line=None

with open("constants.yaml") as file:
    constants=yaml.load(file, Loader=yaml.CLoader)
constants["FFNumbOfSteps"]=int((constants["FFMaxSpeed"]-constants["FFMinSpeed"])/constants["FFStep"])

os.system('')

NT.initialize(server=constants["RobotAddress"]) #roboRIO-447-frc.local

end=time.time()+30
while not NT.isConnected():
    time.sleep(0.1)
    if time.time()>=end:
        raise

allY=[]
allX=[]
#print(NT.getEntry(constants["NTSetShooterSpeed"]).value)
NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(0)
NT.getEntry(constants["NTBypassShooterPID"]).setBoolean(True)

totalTime=(constants["TransitionTime"]*2+
           constants["FFNumbOfSteps"]*constants["FFTimePerSpeed"]+
           constants["PIDRunTime"]+
           len(constants["TestSpeeds"])*constants["TestTime"]+
           len(constants["FFTestSpeeds"])*constants["TestTime"])

print(f"""To tune the shooter enable the robot, press {constants['ShooterButton']} on the operator controller, then click on the terminal, and press (possibly twice) any button besides Space or Enter.
Continue to hold {constants['ShooterButton']} on the controller for the untill done apears on the screen.
this should take about {totalTime} seconds""")
if msvcrt.getch() in b" \r":
    print("Wrong Button!!!!!!!!!!!!!!")
t=threading.Thread(target=thread,daemon=True)
t.start()

progressBar = ShadyBar("FFTuning",max=constants["FFNumbOfSteps"])
for speed in range(int(constants["FFMinSpeed"]), int(constants["FFMaxSpeed"]), int(constants["FFStep"])):
    progressBar.next()
    NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(speed * speedM / 100)
    time.sleep(constants["FFSettleTime"])
    endtime=time.time()+constants["FFTimePerSpeed"]
    X=[]
    ltime=0
    while time.time()<endtime:
        time.sleep(0.01)
        mtime=NT.getEntry(constants["NTTime"]).value/1000
        if mtime==ltime:
            continue
        ltime=mtime
        X.append(NT.getEntry(constants["NTGetShooterEncoder"]).value)
    allX.append(np.mean(X))
    allY.append(speed/100)
progressBar.finish()
m,b=regression(allX,allY)

NT.getEntry(constants["NTShooterFFm"]).setDouble(m)
NT.getEntry(constants["NTShooterFFb"]).setDouble(b)
NT.getEntry(constants["NTBypassShooterPID"]).setBoolean(False)
NT.getEntry(constants["NTShooterP"]).setDouble(0)
NT.getEntry(constants["NTShooterI"]).setDouble(0)
NT.getEntry(constants["NTShooterD"]).setDouble(0)
line="End of FF Tuning"

for speed in constants["FFTestSpeeds"]:
    NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(speed * speedM/100)
    time.sleep(constants["FFTestTime"])

NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(0)
NT.getEntry(constants["NTShooterP"]).setDouble(constants["PIDOscillationP"])
NT.getEntry(constants["NTShooterI"]).setDouble(0)
NT.getEntry(constants["NTShooterD"]).setDouble(0)

line="End of FF Test"



print("FFm:",format_e(m),",FFb:",format_e(b))
while NT.getEntry(constants["NTGetShooterEncoder"]).value > 0.0:
    time.sleep(0.01)

line="Start PID Tuning"
NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(constants["PIDOscillationSpeed"]*speedM)

time.sleep(constants["PIDSettleTime"])

outTime=[]
outTarg=[]
ltime=0
endTime=time.time()+constants["PIDRunTime"]
progressBar = ShadyBar("PID Tuning",max=constants["PIDRunTime"])
while time.time() < endTime:
    time.sleep(.01)
    shSpeed=NT.getEntry(constants["NTGetShooterEncoder"]).value
    mtime=NT.getEntry(constants["NTTime"]).value/1000
    if mtime==ltime:
        continue
    progressBar.goto(int(time.time()-endTime+constants["PIDRunTime"]))
    ltime=mtime
    outTime.append(mtime)
    outTarg.append(shSpeed)
progressBar.finish()

delt=constants["PIDCountDelta"]
count=0
firstT=None
lastT=0
for i in range(delt,len(outTarg)-delt):
    testVals=outTarg[i-delt:i]+outTarg[i+1:i+1+delt]
    good=True
    y=outTarg[i]
    for val in testVals:
        if val >= y:
            good=False
    if good:
        lastT=outTime[i]
        if firstT==None:
            firstT=lastT
        count+=1
p,i,d=rawtuning(lastT-firstT,count,constants["PIDOscillationP"],"pid")
NT.getEntry(constants["NTShooterP"]).setDouble(p)
NT.getEntry(constants["NTShooterI"]).setDouble(i)
NT.getEntry(constants["NTShooterD"]).setDouble(d)
NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(0)

print("P:",format_e(p),"I:",format_e(i),"D:",format_e(d))
line="End of PID"

while NT.getEntry(constants["NTGetShooterEncoder"]).value > 0.0:
    time.sleep(0.01)

for speed in constants["TestSpeeds"]:
    NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(speed * speedM/100)
    time.sleep(constants["TestTime"])

"End of Test"
NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(0)

con=False
print("Done. Press Enter To Exit")
input()
