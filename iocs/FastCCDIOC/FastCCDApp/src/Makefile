TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# Note: we must built the application without GraphicsMagick.
USE_GRAPHICSMAGICK=NO

PROD_NAME = FastCCDApp
PROD_IOC_Linux  += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += FastCCDSupport.dbd
$(PROD_NAME)_DBD += NDPluginFastCCDSupport.dbd

PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += FastCCD
PROD_LIBS += NDPluginFastCCD
PROD_LIBS += cin

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

