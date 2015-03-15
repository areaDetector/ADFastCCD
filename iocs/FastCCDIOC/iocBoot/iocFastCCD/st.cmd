#!../../../../bin/linux-x86_64/FastCCDApp
< envPaths

epicsEnvSet("ADCORE", "$(AREA_DETECTOR)/ADCore")

errlogInit(20000)
epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST" , "NO")
epicsEnvSet("EPICS_CA_ADDR_LIST"      , "10.23.0.255")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

dbLoadDatabase("$(ADFASTCCD)/dbd/FastCCDApp.dbd")
FastCCDApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "XF:23ID1-ES{Dif-Cam:FCCD}")
epicsEnvSet("PORT",   "FASTCCD")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "2048")
epicsEnvSet("YSIZE",  "2048")
epicsEnvSet("NCHANS", "2048")

# FastCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, 
#               int priority, int stackSize, int packetBuffer, int imageBuffer,
#				const char *baseIP, const char *fabricIP, const char *fabricMAC))

FastCCDConfig("$(PORT)", 0, 0, 0, 100000, 2000, 200, "", "10.23.5.127", "")

# Load Records

dbLoadRecords("$(ADCORE)/db/ADBase.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADFASTCCD)/db/FastCCD.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/db/NDFile.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
# Make NELEMENTS in the following be a little bigger than 2048*2048
# Use the following command for 16-bit images.  This can be used for 16-bit detector as long as accumulate mode would not result in 16-bit overflow
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=4200000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

set_requestfile_path("$(ADFASTCCD)/FastCCDApp/Db")

#asynSetTraceMask("$(PORT)",0,0x10)
#asynSetTraceIOMask("$(PORT)",0,1)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX),D=cam1:")
#asynSetTraceMask($(PORT), 0, 255)

dbl > /cf-update/xf23id1-ioc2.es-fccd.dbl

