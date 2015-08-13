#ifndef NDPluginFastCCD_H
#define NDPluginFastCCD_H

#include "NDPluginDriver.h"

#define NDPluginFastCCDGain0String                   "GAIN_0"
#define NDPluginFastCCDGain1String                   "GAIN_1"
#define NDPluginFastCCDGain2String                   "GAIN_2"
#define NDPluginFastCCDDPValString                   "DP_VAL"
#define NDPluginFastCCDBackground0PathString         "BACKGROUND0_PATH"
#define NDPluginFastCCDBackground1PathString         "BACKGROUND1_PATH"
#define NDPluginFastCCDBackground2PathString         "BACKGROUND2_PATH"
#define NDPluginFastCCDBackground0SaveFileString     "BACKGROUND0_SAVE_FILE"
#define NDPluginFastCCDBackground1SaveFileString     "BACKGROUND1_SAVE_FILE"
#define NDPluginFastCCDBackground2SaveFileString     "BACKGROUND2_SAVE_FILE"
#define NDPluginFastCCDBackground0LoadFileString     "BACKGROUND0_LOAD_FILE"
#define NDPluginFastCCDBackground1LoadFileString     "BACKGROUND1_LOAD_FILE"
#define NDPluginFastCCDBackground2LoadFileString     "BACKGROUND2_LOAD_FILE"
#define NDPluginFastCCDEnableBackgroundString        "ENABLE_BACKGROUND"
#define NDPluginFastCCDDataTypeString                "PROCESS_DATA_TYPE"
#define NDPluginFastCCDValidBackgroundString         "VALID_BACKGROUND"
#define NDPluginFastCCDValidBackground0String        "VALID_BACKGROUND_0"
#define NDPluginFastCCDValidBackground1String        "VALID_BACKGROUND_1"
#define NDPluginFastCCDValidBackground2String        "VALID_BACKGROUND_2"
#define NDPluginFastCCDSaveBackground0String         "SAVE_BACKGROUND_0"
#define NDPluginFastCCDSaveBackground1String         "SAVE_BACKGROUND_1"
#define NDPluginFastCCDSaveBackground2String         "SAVE_BACKGROUND_2"

class epicsShareClass NDPluginFastCCD : public NDPluginDriver {
public:
  NDPluginFastCCD(const char *portName, int queueSize, int blockingCallbacks, 
               const char *NDArrayPort, int NDArrayAddr, 
               int maxBuffers, size_t maxMemory,
               int priority, int stackSize);

  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nc, size_t *na);

  void processCallbacks(NDArray *pArray);

protected:
  #define FIRST_NDPLUGIN_FASTCCD_PARAM NDPluginFastCCDGain0
  int NDPluginFastCCDGain0;
  int NDPluginFastCCDGain1;
  int NDPluginFastCCDGain2;

  int NDPluginFastCCDEnableBackground;
  int NDPluginFastCCDDataType;

  int NDPluginFastCCDBackground0Path;
  int NDPluginFastCCDBackground1Path;
  int NDPluginFastCCDBackground2Path;

  int NDPluginFastCCDBackground0SaveFile;
  int NDPluginFastCCDBackground1SaveFile;
  int NDPluginFastCCDBackground2SaveFile;
  int NDPluginFastCCDBackground0LoadFile;
  int NDPluginFastCCDBackground1LoadFile;
  int NDPluginFastCCDBackground2LoadFile;

  int NDPluginFastCCDValidBackground;
  int NDPluginFastCCDValidBackground0;
  int NDPluginFastCCDValidBackground1;
  int NDPluginFastCCDValidBackground2;

  int NDPluginFastCCDSaveBackground0;
  int NDPluginFastCCDSaveBackground1;
  int NDPluginFastCCDSaveBackground2;

  int NDPluginFastCCDDPVal;
  #define LAST_NDPLUGIN_FASTCCD_PARAM NDPluginFastCCDDPVal

private:
  int writeTiffBackground(char* filename, NDArray* array);

  NDArrayInfo arrayInfo;
  NDArray *pBackground0;
  size_t nBackground0Elements;
  NDArray *pBackground1;
  size_t nBackground1Elements;
  NDArray *pBackground2;
  size_t nBackground2Elements;

};
#define NUM_NDPLUGIN_FASTCCD_PARAMS ((int)(&LAST_NDPLUGIN_FASTCCD_PARAM - &FIRST_NDPLUGIN_FASTCCD_PARAM + 1))
    
#endif
