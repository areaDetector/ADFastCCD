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

#define FastCCDPollingPeriodString              "POLL_PERIOD"

#define FastCCDFramestoreString                 "FRAMESTORE"

#define FastCCDMux1String                       "FCCD_MUX1"
#define FastCCDMux2String                       "FCCD_MUX2"

#define FastCCDFirmwarePathString               "FIRMWARE_PATH"
#define FastCCDBiasPathString                   "BIAS_PATH"
#define FastCCDClockPathString                  "CLOCK_PATH"
#define FastCCDFCRICPathString                  "FCRIC_PATH"

#define FastCCDFirmwareUploadString             "FIRMWARE_UPLOAD"
#define FastCCDClockUploadString                "CLOCK_UPLOAD"
#define FastCCDBiasUploadString                 "BIAS_UPLOAD"
#define FastCCDFCRICUploadString                "FCRIC_UPLOAD"

#define FastCCDPowerString                      "CIN_POWER"
#define FastCCDFPPowerString                    "CIN_FP_POWER"
#define FastCCDCameraPowerString					      "CAMERA_POWER"
#define FastCCDCameraPowerModeString					  "CAMERA_POWER_MODE"

#define FastCCDBiasString					              "BIAS"
#define FastCCDClocksString					            "CLOCKS"

#define FastCCDFPGAStatusString                 "FPGA_STATUS"
#define FastCCDDCMStatusString                  "DCM_STATUS"

#define FastCCDBaseBoardIDString                "BASE_BOARD_ID"
#define FastCCDBaseSerialNumString              "BASE_SERIAL_NUM"
#define FastCCDBaseFPGAVersionString            "BASE_FPGA_VER"
#define FastCCDFabBoardIDString                 "FAB_BOARD_ID"
#define FastCCDFabSerialNumString               "FAB_SERIAL_NUM"
#define FastCCDFabFPGAVersionString             "FAB_FPGA_VER"

#define FastCCDStatusHBString                   "STATUS_HB"

#define FastCCDBadPckString                     "BAD_PCK"
#define FastCCDDroppedPckString                 "DROPPED_PCK"
#define FastCCDLastFrameString                  "LAST_FRAME"
#define FastCCDResetStatsString                 "RESET_STATS"
#define FastCCDPacketBufferString               "PACKET_BUFFER"
#define FastCCDFrameBufferString                "FRAME_BUFFER"

#define FastCCDOverscanString                   "OVERSCAN"

#define FastCCDFclkString                       "FCLK"

#define FastCCDFCRICGainString                  "FCRIC_GAIN"
#define FastCCDFCRICClampString                 "FCRIC_CLAMP"

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

#define FastCCDBiasPosHString                   "BIAS_POSH"
#define FastCCDBiasNegHString                   "BIAS_NEGH"
#define FastCCDBiasPosRGString                  "BIAS_POSRG"
#define FastCCDBiasNegRGString                  "BIAS_NEGRG"
#define FastCCDBiasPosSWString                  "BIAS_POSSW"
#define FastCCDBiasNegSWString                  "BIAS_NEGSW"
#define FastCCDBiasPosVString                   "BIAS_POSV"
#define FastCCDBiasNegVString                   "BIAS_NEGV"
#define FastCCDBiasPosTGString                  "BIAS_POSTG"
#define FastCCDBiasNegTGString                  "BIAS_NEGTG"
#define FastCCDBiasPosVFString                  "BIAS_POSVF"
#define FastCCDBiasNegVFString                  "BIAS_NEGVF"
#define FastCCDBiasNEDGEString                  "BIAS_NEDGE"
#define FastCCDBiasOTGString                    "BIAS_OTG"
#define FastCCDBiasVDDRString                   "BIAS_VDDR"
#define FastCCDBiasVDDOutString                 "BIAS_VDDOUT"
#define FastCCDBiasBufBaseString                "BIAS_BUFBASE"
#define FastCCDBiasBufDeltaString               "BIAS_BUFDELTA"
#define FastCCDBiasSpare1String                 "BIAS_SPARE1"
#define FastCCDBiasSpare2String                 "BIAS_SPARE2"

#define FastCCDBiasWriteVString                 "BIAS_WRITEV"

#define FastCCDFOTestString                     "FO_TEST"

#define FastCCDBootString                       "BOOT"
#define FastCCDSendBiasString                   "SEND_BIAS"
#define FastCCDSendFCRICString                  "SEND_FCRIC"

#define FastCCDTimingModeString                 "TIMING_MODE"
#define FastCCDSendTimingString                 "SEND_TIMING"
#define FastCCDTimingNameString                 "TIMING_NAME"
#define FastCCDTimingName0String                "TIMING_NAME_0"
#define FastCCDTimingName1String                "TIMING_NAME_1"
#define FastCCDTimingName2String                "TIMING_NAME_2"
#define FastCCDTimingName3String                "TIMING_NAME_3"
#define FastCCDTimingName4String                "TIMING_NAME_4"
#define FastCCDTimingName5String                "TIMING_NAME_5"
#0;define FastCCDTimingName6String                "TIMING_NAME_6"
#define FastCCDTimingName7String                "TIMING_NAME_7"
#define FastCCDTimingName8String                "TIMING_NAME_8"
#define FastCCDTimingName9String                "TIMING_NAME_9"

//C Function prototypes to tie in with EPICS
static void FastCCDStatusTaskC(void *drvPvt);
static void FastCCDDataStatsTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);
static void messageCallbackC(const char *message, int severity, void *ptr);
static void allocateImageC(cin_data_frame_t *frame, void *ptr);
static void processImageC(cin_data_frame_t *frame, void *ptr);

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
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nc, size_t *na);
  
  // Filename to report driver info
  void report(FILE *fp, int details);

  // Should be private, but are called from C so must be public
  void statusTask(void);
  void dataStatsTask(void);
  
  void messageCallback(const char *message, int severity);
  void allocateImage(cin_data_frame_t *frame);
  void processImage(cin_data_frame_t *frame);  

 protected:
  // Mux Variables
  int FastCCDMux1;
  #define FIRST_FASTCCD_PARAM FastCCDMux1
  int FastCCDMux2;

  // Driver Parameters
  int FastCCDPollingPeriod;

  // Framestore Flag
  int FastCCDFramestore;

  // Overscan
  int FastCCDOverscan;

  // FCLK
  int FastCCDFclk;

  // FCRIC
  int FastCCDFCRICGain;
  int FastCCDFCRICClamp;

  // File upload
  int FastCCDFirmwareUpload;
  int FastCCDBiasUpload;
  int FastCCDClockUpload;
  int FastCCDFCRICUpload;
  int FastCCDFirmwarePath;
  int FastCCDBiasPath;
  int FastCCDClockPath;
  int FastCCDFCRICPath;

  // Power Status
  int FastCCDPower;
  int FastCCDFPPower;
  int FastCCDCameraPower; 
  int FastCCDCameraPowerMode; 

  // Bias and Clocks
  int FastCCDBias;
  int FastCCDClocks;

  // Frame FPGA
  int FastCCDFPGAStatus;
  int FastCCDDCMStatus;

  // Versioning Info
  int FastCCDBaseBoardID;
  int FastCCDBaseSerialNum;
  int FastCCDBaseFPGAVersion;
  int FastCCDFabBoardID;
  int FastCCDFabSerialNum;
  int FastCCDFabFPGAVersion;

  // Status HB
  int FastCCDStatusHB;

  // Data packet stats

  int FastCCDBadPck;
  int FastCCDDroppedPck;
  int FastCCDLastFrame;
  int FastCCDResetStats;
  int FastCCDPacketBuffer;
  int FastCCDFrameBuffer;

  // Fiber Optic Test Mode
  int FastCCDFOTest;

  // New boot mode
  int FastCCDBoot;
  int FastCCDSendTiming;
  int FastCCDSendBias;
  int FastCCDSendFCRIC;

  // Timing Name Strings
  int FastCCDTimingName;
  int FastCCDTimingNameN[10];
  int FastCCDTimingMode;

  // Bias Settings
  int FastCCDBiasPosH;       
  int FastCCDBiasNegH;       
  int FastCCDBiasPosRG;      
  int FastCCDBiasNegRG;      
  int FastCCDBiasPosSW;      
  int FastCCDBiasNegSW;      
  int FastCCDBiasPosV;       
  int FastCCDBiasNegV;       
  int FastCCDBiasPosTG;      
  int FastCCDBiasNegTG;      
  int FastCCDBiasPosVF;      
  int FastCCDBiasNegVF;      
  int FastCCDBiasNEDGE;      
  int FastCCDBiasOTG;        
  int FastCCDBiasVDDR;       
  int FastCCDBiasVDDOut;     
  int FastCCDBiasBufBase;    
  int FastCCDBiasBufDelta;
  int FastCCDBiasSpare1;     
  int FastCCDBiasSpare2;     
  int FastCCDBiasWriteV;

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

  char *cinBaseIP;
  char *cinFabricIP;
  char *cinFabricMAC;

  int firstFrameFlag;

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
  int freeRun;

  void getCameraStatus(int first_run);
  int uploadConfig(int status, int path);
  int uploadFirmware(void);

protected:
  NDArray *pImage;
  cin_ctl_t cin_ctl;
  cin_data_t cin_data;
};

#define NUM_FastCCD_DET_PARAMS ((int)(&LAST_FASTCCD_PARAM- &FIRST_FASTCCD_PARAM + 1))

#endif //FastCCD_H

