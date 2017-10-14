TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC += NDPluginFastCCD
DBD         += NDPluginFastCCD.dbd
INC      += NDPluginFastCCD.h
LIB_SRCS += NDPluginFastCCD.cpp

#NDPluginSupport_DBD += NDPluginFastCCD.dbd

# This tests the problem with forward referencing class sortedListElement if it is
# forwarded referenced in NDPluginDriver.h and defined in NDPluginDriver.cpp
#LIB_SRCS += test_forward_reference.cpp

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for the source files that define template functions.
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
  CODE_CXXFLAGS=
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

