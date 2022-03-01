from networktables import NetworkTables
import time, matplotlib.pyplot as plt, threading

NetworkTables.initialize(server='roboRIO-447-frc.local') #roboRIO-447-frc.local

end=time.time()+30
print("Connecting..")
while not NetworkTables.isConnected():
    time.sleep(0.1)
    if time.time()>=end:
        raise
print("Connected")
gi=NetworkTables.getTable("pidTuningPVs")
st=NetworkTables.getTable('chameleon-vision').getSubTable('Shooter Targeting')

def thread():
    global con
    Y=[]
    X=[]
    ltime=0
    while con:
        time.sleep(0.01)
        mtime=gi.getEntry('timeMS').value/1000
        if mtime==ltime:
            continue
        ltime=mtime
        Y.append(gi.getEntry('Shooter Speed').value)
        X.append(mtime)
    plt.figure(num=1,figsize=[12,6])
    plt.plot(X,Y)
    plt.grid()
    plt.show()

while True:
    con=True
    if input("Enter To Start")=="q":
        break
    t=threading.Thread(target=thread,daemon=True)
    t.start()
    input("Enter To Stop")
    con=False
    time.sleep(0.05)
        
