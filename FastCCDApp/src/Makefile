TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

INC += FastCCD.h

LIBRARY_IOC_Linux += FastCCD
LIB_SRCS += FastCCD.cpp
LIB_LIBS += cin

include $(ADCORE)/ADApp/commonLibraryMakefile

DBD += FastCCDSupport.dbd

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

