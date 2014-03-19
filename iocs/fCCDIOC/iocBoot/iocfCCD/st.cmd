#!../../../../bin/linux-x86_64/fCCDApp
< envPaths

epicsEnvSet("ADCORE", "$(AREA_DETECTOR)/ADCore")

errlogInit(20000)
epicsEnvSet(EPICS_CA_MAX_ARRAY_BYTES,4432896)

dbLoadDatabase("$(ADFCCD)/dbd/fCCDApp.dbd")
fCCDApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "13ANDOR1:")
epicsEnvSet("PORT",   "ANDOR")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "2048")
epicsEnvSet("YSIZE",  "2048")
epicsEnvSet("NCHANS", "2048")

#The following variables must be set to the correct file paths and file names.
epicsEnvSet("FCCD_CONFIG_DIR", "/home/jfarrington/Documents/cin_config/")
epicsEnvSet("FPGA_CONFIGFILE", "top_frame_fpga-v1019j.bit")
epicsEnvSet("CIN_WAVEFORM", "2013_Nov_30-200MHz_CCD_timing.txt")
epicsEnvSet("CIN_FCRIC", "2013_Nov_25-200MHz_fCRIC_timing.txt")
epicsEnvSet("CIN_BIAS", "2013_Nov_05_Bias_Settings.txt")

# andorCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, 
#                const char *installPath, int priority, int stackSize)
# andorCCDConfig("$(PORT)", 0, 0, "/usr/local/etc/andor/", 0, 100000)

# Set up the paths for CIN config files
# Call FCCD_ConfigDirs before FCCD_cin_power_up
FCCD_ConfigDirs($(FCCD_CONFIG_DIR), $(FPGA_CONFIGFILE), $(CIN_WAVEFORM), $(CIN_FCRIC), $(CIN_BIAS) )


FCCD_cin_power_up("param1")
andorCCDConfig("$(PORT)", 0, 0, "/usr/local/etc/andor/", 0, 100000)

#
#FCCD_cin_power_down("param1")
#
dbLoadRecords("$(ADCORE)/db/ADBase.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/db/NDFile.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(TOP)/db/fCCD.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
# Make NELEMENTS in the following be a little bigger than 2048*2048
# Use the following command for 32-bit images.  This is needed for 32-bit detectors or for 16-bit detectors in acccumulate mode if it would overflow 16 bits
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=4200000")
# Use the following command for 16-bit images.  This can be used for 16-bit detector as long as accumulate mode would not result in 16-bit overflow
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=4200000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

#asynSetTraceMask("$(PORT)",0,3)
#asynSetTraceIOMask("$(PORT)",0,4)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX),D=cam1:")
#asynSetTraceMask($(PORT), 0, 255)
