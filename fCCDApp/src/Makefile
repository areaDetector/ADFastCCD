TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#LIBRARY_IOC_WIN32 += fCCD
LIBRARY_IOC_Linux += fCCD
LIB_SRCS += fCCD.cpp
#ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
#LIB_LIBS += ATMCD32M
#else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_LIBS += cin
#endif

DBD += fCCDSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

