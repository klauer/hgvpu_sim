TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = smartSim
# smartSim.dbd will be created and installed
DBD += smartSim.dbd

# smartSim.dbd will be made up from these files:
smartSim_DBD += base.dbd

# Include dbd files from all support applications:
smartSim_DBD += motorSupport.dbd
smartSim_DBD += devSmartMotorAsynSupport.dbd
smartSim_DBD += drvAsynIPPort.dbd
smartSim_DBD += asyn.dbd

# Add all the support libraries needed by this IOC
smartSim_LIBS += motor SmartMotorAsyn asyn

# smartSim_registerRecordDeviceDriver.cpp derives from smartSim.dbd
smartSim_SRCS += smartSim_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
smartSim_SRCS_DEFAULT += smartSimMain.cpp
smartSim_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#smartSim_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
smartSim_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

