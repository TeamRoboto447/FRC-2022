from networktables import NetworkTables
import time

NetworkTables.initialize(server='roboRIO-447-frc.local') #roboRIO-447-frc.local
filename='gyroVsTime.csv'

end=time.time()+30
while not NetworkTables.isConnected():
    time.sleep(0.1)
    if time.time()>=end:
        raise 
        
print(NetworkTables.getRemoteAddress())

with open(filename,'w') as file:
    file.write("")

gi=NetworkTables.getTable('gyroInfo')
st=NetworkTables.getTable('chameleon-vision').getSubTable('Shooter Targeting')
PID=NetworkTables.getTable('chameleon-vision').getSubTable('PID')
##PID.getEntry('kP').setDouble(0.0345)
##PID.getEntry('kI').setDouble(0)
##PID.getEntry('kD').setDouble(0)
ltime=0
with open(filename, 'a') as file:
    while True:
        time.sleep(.01)
        ang=st.getEntry('targetYaw').value
        mtime=gi.getEntry('Time').value
        if mtime==ltime:
            continue
        ltime=mtime
        file.write(f"{ang},{mtime}\n" )
        #print(ang,mtime)


#sd = NetworkTables.getTable('SmartDashboard')
#print(sd.getKeys())

#cv = NetworkTables.getTable('chameleon-vision') #/Shooter Targeting
#cvSh=cv.getSubTable('Shooter Targeting')

#print(cvSh.getEntry("latency").value)
