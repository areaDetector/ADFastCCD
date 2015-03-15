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

#define FastCCDMux1String                       "FCCD_MUX1"
#define FastCCDMux2String                       "FCCD_MUX2"

#define FastCCDFirmwarePathString               "FIRMWARE_PATH"
#define FastCCDBiasPathString                   "BIAS_PATH"
#define FastCCDClockPathString                  "CLOCK_PATH"

#define FastCCDFirmwareUploadString             "FIRMWARE_UPLOAD"
#define FastCCDClockUploadString                "CLOCK_UPLOAD"
#define FastCCDBiasUploadString                 "BIAS_UPLOAD"

#define FastCCDPowerString                      "CIN_POWER"
#define FastCCDFPPowerString                    "CIN_FP_POWER"

#define FastCCDFPGAStatusString                 "FPGA_STATUS"
#define FastCCDDCMStatusString                  "DCM_STATUS"

#define FastCCDBiasString						            "BIAS"
#define FastCCDClockString						          "CLOCK"

#define FastCCDLibCinVersionString              "LIBCINVER"
#define FastCCDBoardIDString                    "BOARD_ID"
#define FastCCDSerialNumString                  "SERIAL_NUM"
#define FastCCDFPGAVersionString                "FPGA_VER"

#define FastCCDStatusHBString                   "STATUS_HB"

#define FastCCDBadPckString                     "BAD_PCK"
#define FastCCDDroppedPckString                 "DROPPED_PCK"
#define FastCCDLastFrameString                  "LAST_FRAME"
#define FastCCDResetStatsString                 "RESET_STATS"

#define FastCCDVBus12V0String                   "VBUS_12V0"
#define FastCCDVMgmt3v3String                   "VMGMT_3V3"
#define FastCCDVMgmt2v5String                   "VMGMT_2V5"
#define FastCCDVMgmt1v2String                   "VMGMT_1V2"
#define FastCCDVEnet1v0String                   "VENET_1V0"
#define FastCCDVS3E3v3String                    "VS3E_3V3"
#define FastCCDVGen3v3String                    "VGEN_3V3"
#define FastCCDVGen2v5String                    "VGEN_2V5"
#define FastCCDV60v9String                      "V6_0V9"
#define FastCCDV61v0String                      "V6_1V0"
#define FastCCDV62v5String                      "V6_2V5"
#define FastCCDVFpString                        "VFP"

#define FastCCDIBus12V0String                   "IBUS_12V0"
#define FastCCDIMgmt3v3String                   "IMGMT_3V3"
#define FastCCDIMgmt2v5String                   "IMGMT_2V5"
#define FastCCDIMgmt1v2String                   "IMGMT_1V2"
#define FastCCDIEnet1v0String                   "IENET_1V0"
#define FastCCDIS3E3v3String                    "IS3E_3V3"
#define FastCCDIGen3v3String                    "IGEN_3V3"
#define FastCCDIGen2v5String                    "IGEN_2V5"
#define FastCCDI60v9String                      "I6_0V9"
#define FastCCDI61v0String                      "I6_1V0"
#define FastCCDI62v5String                      "I6_2V5"
#define FastCCDIFpString                        "IFP"

//C Function prototypes to tie in with EPICS
static void FastCCDStatusTaskC(void *drvPvt);
static void FastCCDDataStatsTaskC(void *drvPvt);
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
          int priority, int stackSize, int packetBuffer, int imageBuffer,
		  const char *baseIP, const char *fabricIP, const char *fabricMAC);

  ~FastCCD();

  /* Overload the connect and disconnect routines */

  asynStatus connect(asynUser *pasynUser);
  asynStatus disconnect(asynUser *pasynUser);

  /* These are the methods that we override from ADDriver */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  //asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  //virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], 
  //                            int values[], int severities[], 
  //                            size_t nElements, size_t *nIn);

  // Filename to report driver info
  void report(FILE *fp, int details);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  void dataStatsTask(void);
  
  void allocateImage(cin_data_frame_t *frame);
  void processImage(cin_data_frame_t *frame);  

 protected:
  // Mux Variables
  int FastCCDMux1;
  #define FIRST_FASTCCD_PARAM FastCCDMux1
  int FastCCDMux2;
  
  int FastCCDFirmwarePath;
  int FastCCDBiasPath;
  int FastCCDClockPath;

  // File upload
  int FastCCDFirmwareUpload;
  int FastCCDBiasUpload;
  int FastCCDClockUpload;

  // Power Status
  int FastCCDPower;
  int FastCCDFPPower;

  // Frame FPGA
  int FastCCDFPGAStatus;
  int FastCCDDCMStatus;

  // Bias Power
  int FastCCDBias;

  // Clock Power
  int FastCCDClock;

  // Versioning Info
  int FastCCDLibCinVersion;
  int FastCCDBoardID;
  int FastCCDSerialNum;
  int FastCCDFPGAVersion;

  // Status HB
  int FastCCDStatusHB;

  // Data packet stats

  int FastCCDBadPck;
  int FastCCDDroppedPck;
  int FastCCDLastFrame;
  int FastCCDResetStats;

  // Power monitor Variables
  int FastCCDPwrBus12V;
  int FastCCDPwrMgmt2V3;
  int FastCCDVBus12V0;
  int FastCCDVMgmt3v3;
  int FastCCDVMgmt2v5;
  int FastCCDVMgmt1v2;
  int FastCCDVEnet1v0;
  int FastCCDVS3E3v3;
  int FastCCDVGen3v3;
  int FastCCDVGen2v5;
  int FastCCDV60v9;
  int FastCCDV61v0;
  int FastCCDV62v5;
  int FastCCDVFp;
  int FastCCDIBus12V0;
  int FastCCDIMgmt3v3;
  int FastCCDIMgmt2v5;
  int FastCCDIMgmt1v2;
  int FastCCDIEnet1v0;
  int FastCCDIS3E3v3;
  int FastCCDIGen3v3;
  int FastCCDIGen2v5;
  int FastCCDI60v9;
  int FastCCDI61v0;
  int FastCCDI62v5;
  int FastCCDIFp;
  #define LAST_FASTCCD_PARAM FastCCDIFp

 private:

  int cinPacketBuffer;
  int cinImageBuffer;

  char cinBaseIP[20];
  char cinFabricIP[20];
  char cinFabricMAC[20];

  // Connect / Disconnect

  asynStatus disconnectCamera();
  asynStatus connectCamera();

  asynStatus setupAcquisition();

  epicsEventId statusEvent;
  epicsEventId dataStatsEvent;
  epicsEventId dataEvent;

  double statusPollingPeriod;
  double dataStatsPollingPeriod;
 
  int framesRemaining;

protected:
  NDArray *pImage;
  struct cin_port cin_data_port;
  struct cin_port cin_ctl_port;
  struct cin_port cin_ctl_port_stream;
};

#define NUM_FastCCD_DET_PARAMS ((int)(&LAST_FASTCCD_PARAM- &FIRST_FASTCCD_PARAM + 1))

#endif //FastCCD_H

