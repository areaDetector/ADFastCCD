TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for the source files that define template functions.

INC += NDPluginFastCCD.h

LIBRARY_IOC_Linux += NDPluginFastCCD
NDPluginFastCCD_SRCS += NDPluginFastCCD.cpp

include $(ADCORE)/ADApp/commonLibraryMakefile

DBD += NDPluginFastCCDSupport.dbd

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

