from networktables import NetworkTables as NT
import time, yaml, math

with open("constants.yaml") as file:
    constants=yaml.load(file, Loader=yaml.CLoader)
constants["FFNumbOfSteps"]=int((constants["FFMaxSpeed"]-constants["FFMinSpeed"])/constants["FFStep"])

NT.initialize()

NT.getEntry(constants["NTSetShooterSpeed"]).setDouble(0)
NT.getEntry(constants["NTGetShooterEncoder"]).setDouble(0)
NT.getEntry(constants["NTTime"]).setDouble(time.time())*1000

while True:
    time.sleep(0.1)
    a=NT.getEntry(constants["NTSetShooterSpeed"]).value
    NT.getEntry(constants["NTGetShooterEncoder"]).setDouble(a+math.sin(time.time())*.1)
    NT.getEntry(constants["NTTime"]).setDouble(time.time())*1000
    print(a,end='\r')
