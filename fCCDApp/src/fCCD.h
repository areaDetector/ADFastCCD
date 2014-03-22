/**
 * Area Detector driver for the FCCD CCD.
 *
 * @author Matthew Pearson
 * @date June 2009
 *
 * Updated Dec 2011 for Asyn 4-17 and areaDetector 1-7 
 *
 * Major updates to get callbacks working, etc. by Mark Rivers Feb. 2011
 */

#ifndef FCCD_H
#define FCCD_H

#include "cin.h"

#include "ADDriver.h"

#define MAX_ENUM_STRING_SIZE 26
#define MAX_ADC_SPEEDS 16
#define MAX_PREAMP_GAINS 16

//#define FCCDCoolerParamString             "ANDOR_COOLER"
//#define FCCDTempStatusMessageString       "ANDOR_TEMP_STAT"
//#define FCCDMessageString                 "ANDOR_MESSAGE"
//#define FCCDShutterModeString             "ANDOR_SHUTTER_MODE"
//#define FCCDShutterExTTLString            "ANDOR_SHUTTER_EXTTL"
//#define FCCDPalFileNameString             "ANDOR_PAL_FILE_PATH"
//#define FCCDAccumulatePeriodString        "ANDOR_ACCUMULATE_PERIOD"
//#define FCCDPreAmpGainString              "ANDOR_PREAMP_GAIN"
//#define FCCDAdcSpeedString                "ANDOR_ADC_SPEED"

#define FCCDSetBiasString                  "FCCD_SETBIAS"
#define FCCDSetClocksString                "FCCD_SETCLOCKS"

/*
typedef struct {
  int ADCIndex;
  int AmpIndex;
  int HSSpeedIndex;
  float HSSpeed;
  int BitDepth;
  char *EnumString;
  int EnumValue;
} FCCDADCSpeed_t;

typedef struct {
  float Gain;
  char *EnumString;
  int EnumValue;
} FCCDPreAmpGain_t;
*/

/**
 * Driver class for FCCD CCD. This inherits from ADDriver class in areaDetector.
 *
 */
class FastCCD : public ADDriver {
 public:
  FastCCD(const char *portName, int maxBuffers, size_t maxMemory, 
           const char *installPath, int priority, int stackSize);
  virtual ~FastCCD();

  /* These are the methods that we override from ADDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual void report(FILE *fp, int details);
  virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                              size_t nElements, size_t *nIn);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  void dataTask(void);

 protected:
  //int FCCDCoolerParam;
  //int FCCDTempStatusMessage;
  int FCCDMessage;
  #define FIRST_ANDOR_PARAM FCCDMessage
  //int FCCDShutterMode;
  //int FCCDShutterExTTL;
  //int FCCDPalFileName;
  int FCCDAccumulatePeriod;
  //int FCCDPreAmpGain;
  //int FCCDAdcSpeed;
  int FCCDSetBias;
  int FCCDSetClocks;
  #define LAST_ANDOR_PARAM FCCDSetClocks

 private:

  unsigned int checkStatus(unsigned int returnStatus);
  asynStatus setupAcquisition();
  //asynStatus setupShutter(int command);
  //void saveDataFrame(int frameNumber);
  //void setupADCSpeeds();
  //void setupPreAmpGains();
  /**
   * Additional image mode to those in ADImageMode_t
   */
  // static const epicsInt32 AImageFastKinetics;

  /**
   * List of acquisiton modes.
   */
  static const epicsUInt32 AASingle;
  static const epicsUInt32 AAAccumulate;
  static const epicsUInt32 AARunTillAbort;
  static const epicsUInt32 AATimeDelayedInt;

  /**
   * List of trigger modes.
   */
  static const epicsUInt32 ATInternal;
  static const epicsUInt32 ATExternal1;
  static const epicsUInt32 ATExternal2;
  static const epicsUInt32 ATExternal1or2;

  /**
   * List of detector status states
   */
  static const epicsUInt32 ASIdle;
  //static const epicsUInt32 ASTempCycle;
  static const epicsUInt32 ASAcquiring;
  //static const epicsUInt32 ASAccumTimeNotMet;
  //static const epicsUInt32 ASKineticTimeNotMet;
  //static const epicsUInt32 ASErrorAck;
  //static const epicsUInt32 ASAcqBuffer;
  //static const epicsUInt32 ASSpoolError;

  /**
   * List of detector readout modes.
   */
  //static const epicsInt32 ARFullVerticalBinning;
  //static const epicsInt32 ARMultiTrack;
  //static const epicsInt32 ARRandomTrack;
  //static const epicsInt32 ARSingleTrack;
  //static const epicsInt32 ARImage;

  /**
   * List of shutter modes
   */
  static const epicsInt32 AShutterAuto;
  static const epicsInt32 AShutterOpen;
  static const epicsInt32 AShutterClose;


  epicsEventId statusEvent;
  epicsEventId dataEvent;
  double mPollingPeriod;
  double mFastPollingPeriod;
  unsigned int mAcquiringData;
  unsigned int m_bRequestStop;
  char *mInstallPath;
  
  /**
   * ADC speed parameters
   */
  //int mNumAmps;
  //int mNumADCs;
  //int mNumADCSpeeds;
  //FCCDADCSpeed_t mADCSpeeds[MAX_ADC_SPEEDS];
  //int mTotalPreAmpGains;
  //int mNumPreAmpGains;
  //FCCDPreAmpGain_t mPreAmpGains[MAX_PREAMP_GAINS];

  float mAcquireTime;
  float mAcquirePeriod;
  float mAccumulatePeriod;
  
protected:
   struct cin_port m_port;
   NDArray *m_pArray;

private:
   int FCCD_Init();
   int FCCD_GetImage(); 
   void int_handler(int dummy);
   
};

#define NUM_ANDOR_DET_PARAMS ((int)(&LAST_ANDOR_PARAM - &FIRST_ANDOR_PARAM + 1))

#endif //FCCD_H

