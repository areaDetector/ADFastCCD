#!../../bin/linux-x86_64/FastCCDApp
< envPaths

epicsEnvSet("ADCORE", "$(AREA_DETECTOR)/ADCore")

errlogInit(20000)

epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST" , "NO")
epicsEnvSet("EPICS_CA_ADDR_LIST"      , "10.23.0.255")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

dbLoadDatabase("$(TOP)/dbd/FastCCDApp.dbd")
FastCCDApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "XF:23ID1-ES{FCCD}")
epicsEnvSet("PORT",   "FASTCCD")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "2048")
epicsEnvSet("YSIZE",  "2048")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("CBUFFS", "500")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# FastCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, 
#               int priority, int stackSize, int packetBuffer, int imageBuffer,
#				const char *baseIP, const char *fabricIP, const char *fabricMAC))

FastCCDConfig("$(PORT)", 0, 0, 0, 100000, 2000, 200, "", "10.23.4.127", "")

# Load Records

dbLoadRecords("$(ADFASTCCD)/db/FastCCD.template",   "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "FASTCCD", 0, 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,NDARRAY_PORT=FASTCCD,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=2361600")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

# Load the FastCCD Plugin
NDFastCCDConfigure("FastCCD1", $(QSIZE), 0, "PROC1", 0, 0, 0, 0, 0, $(MAX_THREADS=5))
dbLoadRecords("$(ADFASTCCD)/db/NDFastCCD.template", "P=$(PREFIX),R=FastCCD1:, PORT=FastCCD1, NDARRAY_PORT=FASTCCD, ADDR=0, TIMEOUT=1")

set_requestfile_path("$(ADFASTCCD)/FastCCDApp/Db")
set_requestfile_path("$(ADFASTCCD)/FastCCDPlugin/Db")
set_requestfile_path("$(ADCORE)","ADApp/Db")

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX),D=cam1:")
dbl > $(TOP)/records.dbl
dbl > /cf-update/xf23id1-ioc2.es-fccd.dbl

dbpf $(PREFIX)cam1:NDAttributesFile FastCCDDetectorAttributes.xml
dbpf $(PREFIX)Stats1:NDAttributesFile StatsAttributes.xml
dbpf $(PREFIX)ROIStat1:NDAttributesFile ROIStatAttributes.xml


