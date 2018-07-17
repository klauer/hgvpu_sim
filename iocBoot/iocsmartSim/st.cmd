#!../../bin/darwin-x86/smartSim

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/smartSim.dbd"
smartSim_registerRecordDeviceDriver pdbbase

# epicsEnvSet("EPICS_CAS_INTF_ADDR_LIST", "127.0.0.1")
# epicsEnvSet("EPICS_CA_ADDR_LIST", "127.0.0.1")
# epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST", "NO")

epicsEnvSet("SIM_IP", "127.0.0.1")

## Load record instances
dbLoadRecords("db/hgvpuMotion.db","U=USEG:SIM")

drvAsynIPPortConfigure "M1_USW" "$(SIM_IP):7001 TCP" 0 0 0
SmartCreateController(S7,M1_USW,4,1,100,1000)

cd "${TOP}/iocBoot/${IOC}"
iocInit()

dbpf "USEG:SIM:USWMotor:Sync" 0

#seq sncxxx,""
