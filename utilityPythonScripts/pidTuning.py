from networktables import NetworkTables
import time

P = "turretkP"
I = "turretkI"
D = "turretkD"


def networktable():
    NetworkTables.initialize(server='roboRIO-447-frc.local')
    end=time.time()+30
    print("Connecting...")
    while not NetworkTables.isConnected():
        time.sleep(0.1)
        if time.time()>=end:
            raise
        
    print("Connected")
    kp=0.054
    gi=NetworkTables.getTable("pidTuningPVs")
    cv=NetworkTables.getTable('chameleon-vision')
    PID=NetworkTables.getTable('PID')
    PID.getEntry(P).setDouble(kp)
    PID.getEntry(I).setDouble(0)
    PID.getEntry(D).setDouble(0)
    #PID.getEntry("bypassShooterPID").setBoolean(False)
    #cv.getEntry("shooterSpeed").setDouble(0.6)
    outTime=[]
    outTarg=[]
    ltime=0
    
    print("Start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    time.sleep(2)
    print("Starting")

    endTime=time.time()+10
    while time.time() < endTime:
        time.sleep(.01)
        ang=gi.getEntry('Turret Position').value
        mtime=gi.getEntry('timeMS').value/1000
        if mtime==ltime:
            continue
        ltime=mtime
        outTime.append(mtime)
        outTarg.append(ang)
    print("done")
    with open("pidTuning.csv","w") as file:
        file.write("\n".join(str(ti)+","+str(ta) for ti,ta in zip(outTime,outTarg)))
    return outTime, outTarg, kp

def der(tar,time):
    derY=[]
    derT=[]
    lastDerY=0
    count=0
    first0=None
    last0=None
    for i in range(len(tar)-1):
        Y=(tar[i+1]-tar[i])/(time[i+1]-time[i])
        X=(time[i+1]+time[i])/2
        if Y==0 or Y>0 and lastDerY<0 or Y<0 and lastDerY>0:
            count+=1
            if first0==None:
                first0=X
            last0=X
        derY.append(Y)
        derT.append(X)
        lastDerY=Y
    return derT, derY, count, last0, first0

def oDPDerReg(tar,time):
    from scipy import optimize
    import numpy as np
    derT, derY, count, last0, first0 = der(tar,time)
    with open('test.csv','w') as file:
        file.write("\n".join(str(a) for a in derY))
    def testFunc(x,A,w,f):
        return A*np.sin(2*np.pi/w*x+f)
    params, params_covariance = optimize.curve_fit(testFunc,derT,derY,p0=[1,0.47,0]) # 
    print(params_covariance)
    return params

def oDPDerZero(tar,time):
    derT, derY, count, last0, first0 = der(tar,time)
    return (count-1)/2,last0-first0
def oDPRawMax(tar,time):
    delt=2

    count=0
    firstT=None
    lastT=0
    for i in range(delt,len(tar)-delt):
        testVals=tar[i-delt:i]+tar[i+1:i+1+delt]
        good=True
        y=tar[i]
        for val in testVals:
            if val >= y:
                good=False
        if good:
            lastT=time[i]
            if firstT==None:
                firstT=lastT
            count+=1
    return count-1,lastT-firstT

def oscillationDataParcer(tar,time):
    if len(tar) != len(time):
        raise ValueError("tar and time must be of the same length not {}, {} respectively".format(len(tar),len(target)))
    return oDPRawMax(tar,time)
    
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
def setPIDVals(p,i,d):
    PID=NetworkTables.getTable('PID')
    PID.getEntry(P).setDouble(p)
    PID.getEntry(I).setDouble(i)
    PID.getEntry(D).setDouble(d)

def inpTuning():
    time=float(input("total time(s): "))
    numb=float(input("number of oscillations: "))
    prop=float(input("proportional value: "))
    loopType=input("P, PI, or PID loop: ")
    p,i,d=rawtuning(time,numb,prop,loopType)
    print("P:",p,"I:",i,"D:",d)
def testing():
    with open("pidTuning.csv") as file:
        raw=file.read()
    arr=[a.split(',') for a in raw.split("\n")]
    tar=[float(i[1]) for i in arr]
    time=[float(i[0]) for i in arr]
    print(*oscillationDataParcer(tar,time))
def runNetTest():
    times, targ, kp = networktable()
    count, time1 = oscillationDataParcer(targ,times)
    print("c:",count,"T:",time1,"T/c:", time1/count,"kp:",kp)
    p,i,d=rawtuning(time1,count,kp,"PID")
    setPIDVals(p,i,d)
    print("P:",p,"I:",i,"D:",d)
    time.sleep(1)


if __name__=="__main__":
    try:
        [runNetTest,testing,inpTuning][0]()
    finally:
        if NetworkTables.isConnected():
            NetworkTables.stopClient()

#P: 0.018 I: 0.047888259945333134 D: 0.001691437527537346
