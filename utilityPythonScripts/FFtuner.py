from networktables import NetworkTables
import time, numpy as np, regretion
from progress.bar import ShadyBar as Bar
import os

os.system('')
timePerSpeed=1
startSpeed=30
maxSpeed=100
step=5

NetworkTables.initialize(server='roboRIO-447-frc.local') #roboRIO-447-frc.local

end=time.time()+30
while not NetworkTables.isConnected():
    time.sleep(0.1)
    if time.time()>=end:
        raise
PID=NetworkTables.getTable("PID")
gi=NetworkTables.getTable("pidTuningPVs")
cv=NetworkTables.getTable("chameleon-vision")

allY=[]
allX=[]

cv.getEntry("shooterSpeed").setDouble(0)
PID.getEntry("bypassShooterPID").setBoolean(True)
print("Start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
time.sleep(2)

progressBar = Bar("ShadyBar",max=int((maxSpeed-startSpeed)/step))
for speed in range(int(startSpeed), int(maxSpeed), int(step)):
    progressBar.next()
    cv.getEntry("shooterSpeed").setDouble(speed/100)
    time.sleep(1)
    endtime=time.time()+timePerSpeed
    X=[]
    ltime=0
    while time.time()<endtime:
        time.sleep(0.01)
        mtime=gi.getEntry('Time').value/1000
        if mtime==ltime:
            continue
        ltime=mtime
        X.append(gi.getEntry('Shooter Speed').value)
    allX.append(np.mean(X))
    allY.append(speed/100)

m,b=regretion.regRaw(allX,allY)

time.sleep(0.01)

PID.getEntry("shootkFFm").setDouble(m)
PID.getEntry("shootkFFb").setDouble(b)
PID.getEntry("bypassShooterPID").setBoolean(False)

print("\nm:",m,",b:",b)
input()
