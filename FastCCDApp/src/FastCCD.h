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

#define FCCDSetBiasString                  "FCCD_SETBIAS"
#define FCCDSetClocksString                "FCCD_SETCLOCKS"

/**
 * Driver class for FCCD CCD. This inherits from ADDriver class in areaDetector.
 *
 */
class FastCCD : public ADDriver {
 public:
  FastCCD(const char *portName, int maxBuffers, size_t maxMemory, 
           const char *installPath, int priority, int stackSize);
  virtual ~FastCCD();

  /* Overload the connect and disconnect routines */

  asynStatus connect(asynUser *pasynUser);
  asynStatus disconnect(asynUser *pasynUser);

  /* These are the methods that we override from ADDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual void report(FILE *fp, int details);
  virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], 
                              int values[], int severities[], 
                              size_t nElements, size_t *nIn);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  void dataTask(void);

 protected:
  int FCCDSetBias;
  #define FIRST_ANDOR_PARAM FCCDSetBias
  int FCCDSetClocks;
  #define LAST_ANDOR_PARAM FCCDSetClocks

 private:

  // Connect / Disconnect

  asynStatus disconnectCamera();
  asynStatus connectCamera();

  unsigned int checkStatus(unsigned int returnStatus);
  asynStatus setupAcquisition();

  // List of acquisiton modes.
  static const epicsUInt32 AASingle;
  static const epicsUInt32 AAAccumulate;
  static const epicsUInt32 AARunTillAbort;
  static const epicsUInt32 AATimeDelayedInt;

  // List of trigger modes.
  static const epicsUInt32 ATInternal;
  static const epicsUInt32 ATExternal1;
  static const epicsUInt32 ATExternal2;
  static const epicsUInt32 ATExternal1or2;

  // List of detector status states
  static const epicsUInt32 ASIdle;
  static const epicsUInt32 ASAcquiring;

  // List of shutter modes
  static const epicsInt32 AShutterAuto;
  static const epicsInt32 AShutterOpen;
  static const epicsInt32 AShutterClose;

  epicsEventId statusEvent;
  epicsEventId dataEvent;
  double mPollingPeriod;
  double mFastPollingPeriod;
  unsigned int mAcquiringData;
  unsigned int m_bRequestStop;
  
  float mAcquireTime;
  float mAcquirePeriod;
  float mAccumulatePeriod;
  
  int FCCD_Init();
  int GetImage(); 
  void int_handler(int dummy);
   
protected:
  struct cin_port cin_data_port;
  struct cin_port cin_ctl_port;
  struct cin_port cin_ctl_port_stream;
  NDArray *m_pArray;
};

#define NUM_FCCD_DET_PARAMS ((int)(&LAST_ANDOR_PARAM - &FIRST_ANDOR_PARAM + 1))

#endif //FCCD_H

