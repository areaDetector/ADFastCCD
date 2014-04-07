/**
 * Area Detector driver for the FastCCD CCD.
 *
 * @author Stuart Wilkins and Daron Chabot
 * @date June 2009
 *
 */

#ifndef FastCCD_H
#define FastCCD_H

#include "cin.h"
#include "ADDriver.h"

#define MAX_ENUM_STRING_SIZE 26

#define FastCCDPwrBus12VString              "FastCCD_PwrBus12V"
#define FastCCDPwrMgmt2V3String             "FastCCD_PwrMgmt3V3"

//C Function prototypes to tie in with EPICS
static void FastCCDStatusTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);
static void allocateImageC(cin_data_frame_t *frame);
static void processImageC(cin_data_frame_t *frame);

/**
 * Driver class for FastCCD CCD. This inherits from ADDriver class in areaDetector.
 *
 */
class FastCCD : public ADDriver {
 public:
  FastCCD(const char *portName, int maxBuffers, size_t maxMemory, 
          int priority, int stackSize, int packetBuffer, int imageBuffer);

  ~FastCCD();

  /* Overload the connect and disconnect routines */

  asynStatus connect(asynUser *pasynUser);
  asynStatus disconnect(asynUser *pasynUser);

  /* These are the methods that we override from ADDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  //virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], 
  //                            int values[], int severities[], 
  //                            size_t nElements, size_t *nIn);

  // Filename to report driver info
  void report(FILE *fp, int details);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  
  void allocateImage(cin_data_frame_t *frame);
  void processImage(cin_data_frame_t *frame);  

 protected:
  int FastCCDPwrBus12V;
  #define FIRST_FASTCCD_PARAM FastCCDPwrBus12V
  int FastCCDPwrMgmt2V3;
  #define LAST_FASTCCD_PARAM FastCCDPwrMgmt2V3

 private:

  int cinPacketBuffer;
  int cinImageBuffer;

  // Connect / Disconnect

  asynStatus disconnectCamera();
  asynStatus connectCamera();

  asynStatus setupAcquisition();

  // List of trigger modes.
  static const epicsUInt32 ATInternal;
  static const epicsUInt32 ATExternal1;
  static const epicsUInt32 ATExternal2;
  static const epicsUInt32 ATExternal1or2;

  epicsEventId statusEvent;
  epicsEventId dataEvent;

  double mPollingPeriod;
 
  int framesRemaining;

protected:
  NDArray *pImage;
  struct cin_port cin_data_port;
  struct cin_port cin_ctl_port;
  struct cin_port cin_ctl_port_stream;
};

#define NUM_FastCCD_DET_PARAMS ((int)(&LAST_FASTCCD_PARAM- &FIRST_FASTCCD_PARAM + 1))

#endif //FastCCD_H

