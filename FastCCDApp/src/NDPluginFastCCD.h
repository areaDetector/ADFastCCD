#ifndef NDPluginFastCCD_H
#define NDPluginFastCCD_H

#include "NDPluginDriver.h"

#define NDPluginFastCCDOffset0String                 "OFFSET_0"
#define NDPluginFastCCDOffset1String                 "OFFSET_1"
#define NDPluginFastCCDOffset2String                 "OFFSET_2"
#define NDPluginFastCCDDPValString                   "DP_VAL"
#define NDPluginFastCCDEnableString                  "ENABLE"
#define NDPluginFastCCDEnableDataString              "ENABLE_DATA"

class epicsShareClass NDPluginFastCCD : public NDPluginDriver {
public:
    NDPluginFastCCD(const char *portName, int queueSize, int blockingCallbacks, 
                 const char *NDArrayPort, int NDArrayAddr, 
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize);

    void processCallbacks(NDArray *pArray);

protected:
    #define FIRST_NDPLUGIN_FASTCCD_PARAM NDPluginFastCCDOffset0
    int NDPluginFastCCDOffset0;
    int NDPluginFastCCDOffset1;
    int NDPluginFastCCDOffset2;
    int NDPluginFastCCDEnable;
    int NDPluginFastCCDEnableData;
    int NDPluginFastCCDDPVal;
    #define LAST_NDPLUGIN_FASTCCD_PARAM NDPluginFastCCDDPVal
                                
private:
    NDArrayInfo arrayInfo;
};
#define NUM_NDPLUGIN_FASTCCD_PARAMS ((int)(&LAST_NDPLUGIN_FASTCCD_PARAM - &FIRST_NDPLUGIN_FASTCCD_PARAM + 1))
    
#endif
