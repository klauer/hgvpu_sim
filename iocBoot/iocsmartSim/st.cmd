#!../../bin/darwin-x86/smartSim

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/smartSim.dbd"
smartSim_registerRecordDeviceDriver pdbbase

epicsEnvSet("EPICS_CAS_INTF_ADDR_LIST", "127.0.0.1")
epicsEnvSet("EPICS_CA_ADDR_LIST", "127.0.0.1")
epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST", "NO")

epicsEnvSet("SIM_IP", "127.0.0.1")

## Load record instances
dbLoadRecords("db/hgvpuMotion.db","U=USEG:SIM")

drvAsynIPPortConfigure "M1_USW" "$(SIM_IP):7001 TCP" 0 0 0
SmartCreateController(S7,M1_USW,4,1,100,1000)

cd "${TOP}/iocBoot/${IOC}"
iocInit()

dbpf "USEG:SIM:USWMotor:Sync" 0

dbpf USEG:SIM:USWMotor.MRES 1e-6
dbpf USEG:SIM:USWMotor.ERES 1e-6
dbpf USEG:SIM:DSWMotor.MRES 1e-6
dbpf USEG:SIM:DSWMotor.ERES 1e-6
dbpf USEG:SIM:DSAMotor.MRES 1e-6
dbpf USEG:SIM:DSAMotor.ERES 1e-6
dbpf USEG:SIM:USAMotor.MRES 1e-6
dbpf USEG:SIM:USAMotor.ERES 1e-6

dbpf USEG:SIM:USWGap.CALC A
dbpf USEG:SIM:USWGap.INPA "USEG:SIM:USWMotor.RBV NPP MS"
dbpf USEG:SIM:DSWGap.CALC A
dbpf USEG:SIM:DSWGap.INPA "USEG:SIM:DSWMotor.RBV NPP MS"
dbpf USEG:SIM:DSAGap.CALC A
dbpf USEG:SIM:DSAGap.INPA "USEG:SIM:DSAMotor.RBV NPP MS"
dbpf USEG:SIM:USAGap.CALC A
dbpf USEG:SIM:USAGap.INPA "USEG:SIM:USAMotor.RBV NPP MS"

dbpf USEG:SIM:USWMotor.VELO 1
dbpf USEG:SIM:DSWMotor.VELO 1
dbpf USEG:SIM:DSAMotor.VELO 1
dbpf USEG:SIM:USAMotor.VELO 1

epicsThreadSleep 1.0

dbpf USEG:SIM:USWMotor.VAL 3.7
dbpf USEG:SIM:DSWMotor.VAL 3.7
dbpf USEG:SIM:DSAMotor.VAL 3.7
dbpf USEG:SIM:USAMotor.VAL 3.7

epicsThreadSleep 2.0

dbpf "USEG:SIM:USWMotor:Sync" 1
