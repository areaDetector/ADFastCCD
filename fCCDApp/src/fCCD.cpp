/**
 * FastCCD.cpp
 * derived from andorCCD.cpp
 * Area Detector driver for the Andor CCD.
 *
 * Ver
 * 0.1 YF 1/20/14
 * 0.2 YF 3/3/14
 *
 */

#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsExit.h>

#include "fCCD.h"

// Function prototypes for cin_power.c
extern "C" {
int cin_power_up();
int cin_power_down();
int CIN_set_bias(int val);
int CIN_set_clocks(int val);
int CIN_set_trigger(int val);
int CIN_get_trigger_status(); 
int CIN_set_exposure_time(float e_time);
int CIN_set_trigger_delay(float t_time);
int CIN_set_cycle_time(float c_time);
int CIN_set_trigger_mode(int val);
int CIN_trigger_start();
int CIN_trigger_stop();


}


static const char *driverName = "andorCCD";

//Definitions of static class data members

const epicsInt32 AndorCCD::AImageFastKinetics = ADImageContinuous+1;

const epicsUInt32 AndorCCD::AASingle = 1;
const epicsUInt32 AndorCCD::AAAccumulate = 2;
const epicsUInt32 AndorCCD::AARunTillAbort = 5;
const epicsUInt32 AndorCCD::AATimeDelayedInt = 9;

const epicsUInt32 AndorCCD::ATInternal = 0;
const epicsUInt32 AndorCCD::ATExternal1 = 1;
const epicsUInt32 AndorCCD::ATExternal2 = 2;
const epicsUInt32 AndorCCD::ATExternal1or2 = 3;

/*
const epicsUInt32 AndorCCD::ASIdle = DRV_IDLE;
const epicsUInt32 AndorCCD::ASTempCycle = DRV_TEMPCYCLE;
const epicsUInt32 AndorCCD::ASAcquiring = DRV_ACQUIRING;
const epicsUInt32 AndorCCD::ASAccumTimeNotMet = DRV_ACCUM_TIME_NOT_MET;
const epicsUInt32 AndorCCD::ASKineticTimeNotMet = DRV_KINETIC_TIME_NOT_MET;
const epicsUInt32 AndorCCD::ASErrorAck = DRV_ERROR_ACK;
const epicsUInt32 AndorCCD::ASAcqBuffer = DRV_ACQ_BUFFER;
const epicsUInt32 AndorCCD::ASSpoolError = DRV_SPOOLERROR;
*/

const epicsInt32 AndorCCD::ARFullVerticalBinning = 0;
const epicsInt32 AndorCCD::ARMultiTrack = 1;
const epicsInt32 AndorCCD::ARRandomTrack = 2;
const epicsInt32 AndorCCD::ARSingleTrack = 3;
const epicsInt32 AndorCCD::ARImage = 4;

const epicsInt32 AndorCCD::AShutterAuto = 0;
const epicsInt32 AndorCCD::AShutterOpen = 1;
const epicsInt32 AndorCCD::AShutterClose = 2;


//C Function prototypes to tie in with EPICS
static void andorStatusTaskC(void *drvPvt);
static void andorDataTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);

//#define YF_LOCAL_EDITS 1
#undef YF_LOCAL_EDITS 

#ifdef USE_LIBCIN

void AndorCCD::int_handler(int dummy){
  cin_data_stop_threads();
  exit(0);     // YF valid in this context??
}


// YF New code
int AndorCCD::FCCD_Init()
{
   int ret= 0;
   

#ifdef YF_LOCAL_EDITS
  if(cin_init_data_port(&m_port, "192.168.11.112", 49201, "192.168.11.112", 49203, 1000))
#else
  if(cin_init_data_port(&m_port, NULL, 0, "10.23.5.127", 0, 1000))
#endif
   {
      printf("cin_init_data_port returned error \n");
      return (-1);
   }
   else
   {
     printf("port.srvaddr = %s\n", m_port.srvaddr);
   }

   
   /* Start the main routine */
#ifdef YF_LOCAL_EDITS
  if( (ret = cin_data_init(CIN_DATA_MODE_PUSH_PULL, 50, 50)) )
#else
  if( (ret = cin_data_init(CIN_DATA_MODE_PUSH_PULL, 2000, 2000)))
#endif   
   {
      printf("cin_data_init returned error:%d \n",ret);
      return (-1);
   }
   else
   {
     printf("cin_data_init. No error \n");
   }
   // YF Not sure how to implement this in c++.
   // signal(SIGINT, int_handler); // YF: requires C linkage. May not work with class method :-(
   // cin_data_start_monitor_output();
   return (0);   
}

int AndorCCD::FCCD_GetImage() 
{
   size_t dims[2];
   int nDims = 2;
   uint16_t frame_number;
   NDDataType_t dataType;
   
   dims[0] = CIN_DATA_FRAME_WIDTH;
   dims[1] = CIN_DATA_FRAME_HEIGHT;
   dataType = NDUInt16; // kick it.
   
   m_pArray = this->pNDArrayPool->alloc(nDims, dims, dataType, 
      0, NULL);
      
   if (m_pArray)
   {
      // Load the buffer. Pass in memory allocated in NDArrayPool
      cin_data_load_frame((uint16_t *)m_pArray->pData, &frame_number);
      printf("cin_data_load_frame. frame->number %u\n", frame_number);
      return (0);
   }
   else
   {
      printf("********** cin_data_load_frame error ****************\r\n");
      return (-1); // error
   }
}
#endif

/** Constructor for Andor driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] installPath The path to the Andor directory containing the detector INI files, etc.
  *            This can be specified as an empty string ("") for new detectors that don't use the INI
  *            files on Windows, but must be a valid path on Linux.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
AndorCCD::AndorCCD(const char *portName, int maxBuffers, size_t maxMemory, 
                   const char *installPath, int priority, int stackSize)

  : ADDriver(portName, 1, NUM_ANDOR_DET_PARAMS, maxBuffers, maxMemory, 
             asynEnumMask, asynEnumMask,
             ASYN_CANBLOCK, 1, priority, stackSize)
{

  int status = asynSuccess;
  int i;
  int binX=1, binY=1, minX=0, minY=0, sizeX, sizeY;
  
  static const char *functionName = "AndorCCD";

  if (installPath == NULL)
    strcpy(mInstallPath, "");
  else 
    mInstallPath = epicsStrDup(installPath);

  /* Create an EPICS exit handler */
  epicsAtExit(exitHandler, this);

  createParam(AndorCoolerParamString,             asynParamInt32, &AndorCoolerParam);
  createParam(AndorTempStatusMessageString,       asynParamOctet, &AndorTempStatusMessage);
  createParam(AndorMessageString,                 asynParamOctet, &AndorMessage);
  createParam(AndorShutterModeString,             asynParamInt32, &AndorShutterMode);
  createParam(AndorShutterExTTLString,            asynParamInt32, &AndorShutterExTTL);
  createParam(AndorPalFileNameString,             asynParamOctet, &AndorPalFileName);
  createParam(AndorAccumulatePeriodString,      asynParamFloat64, &AndorAccumulatePeriod);
  createParam(AndorPreAmpGainString,              asynParamInt32, &AndorPreAmpGain);
  createParam(AndorAdcSpeedString,                asynParamInt32, &AndorAdcSpeed);

  // YF Custom PV Records
  createParam(FCCDSetBiasString,                  asynParamInt32, &FCCDSetBias);
  createParam(FCCDSetClocksString,                asynParamInt32, &FCCDSetClocks);

  
  
  // Create the epicsEvent for signaling to the status task when parameters should have changed.
  // This will cause it to do a poll immediately, rather than wait for the poll time period.
  this->statusEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->statusEvent) {
    printf("%s:%s epicsEventCreate failure for start event\n", driverName, functionName);
    return;
  }

  // Use this to signal the data acquisition task that acquisition has started.
  this->dataEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->dataEvent) {
    printf("%s:%s epicsEventCreate failure for data event\n", driverName, functionName);
    return;
  }

  if ( (i=FCCD_Init()) )
  {
     printf("FCCD_Init() error:%d\n", i);
  }
  // Initialize camera
  try {
    printf("%s:%s: initializing camera\n",
      driverName, functionName);

    setStringParam(AndorMessage, "Camera successfully initialized.");
    // YF  checkStatus(GetDetector(&sizeX, &sizeY));
    // YF checkStatus(GetHeadModel(model));
    // YF checkStatus(SetReadMode(ARImage));
    // YF checkStatus(SetImage(binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY));
    sizeX = CIN_DATA_FRAME_WIDTH;
    sizeY = CIN_DATA_FRAME_HEIGHT;
    
   
    callParamCallbacks();
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    return;
  }

#if 0  
  // Initialize ADC enums
  for (i=0; i<MAX_ADC_SPEEDS; i++) {
    mADCSpeeds[i].EnumValue = i;
    mADCSpeeds[i].EnumString = (char *)calloc(MAX_ENUM_STRING_SIZE, sizeof(char));
  } 

  // Initialize Pre-Amp enums
  for (i=0; i<MAX_PREAMP_GAINS; i++) {
    mPreAmpGains[i].EnumValue = i;
    mPreAmpGains[i].EnumString = (char *)calloc(MAX_ENUM_STRING_SIZE, sizeof(char));
  } 
  
#endif

  /* Set some default values for parameters */
  status =  setStringParam(ADManufacturer, "FCCD");
  status |= setStringParam(ADModel, ""); // Model ?
  status |= setIntegerParam(ADSizeX, sizeX);
  status |= setIntegerParam(ADSizeY, sizeY);
  status |= setIntegerParam(ADBinX, 1);
  status |= setIntegerParam(ADBinY, 1);
  status |= setIntegerParam(ADMinX, 0);
  status |= setIntegerParam(ADMinY, 0);
  status |= setIntegerParam(ADMaxSizeX, sizeX);
  status |= setIntegerParam(ADMaxSizeY, sizeY);  
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, AndorCCD::ATInternal);
  mAcquireTime = 1.0;
  status |= setDoubleParam (ADAcquireTime, mAcquireTime);
  mAcquirePeriod = 5.0;
  status |= setDoubleParam (ADAcquirePeriod, mAcquirePeriod);
  status |= setIntegerParam(ADNumImages, 1);
  status |= setIntegerParam(ADNumExposures, 1);
  status |= setIntegerParam(NDArraySizeX, sizeX);
  status |= setIntegerParam(NDArraySizeY, sizeY);
  status |= setIntegerParam(NDDataType, NDUInt16);
  status |= setIntegerParam(NDArraySize, sizeX*sizeY*sizeof(epicsUInt16)); 
  mAccumulatePeriod = 2.0;
  status |= setDoubleParam(AndorAccumulatePeriod, mAccumulatePeriod); 
  //status |= setIntegerParam(AndorAdcSpeed, 0);
  //status |= setIntegerParam(AndorShutterExTTL, 1);
  status |= setIntegerParam(AndorShutterMode, AShutterAuto);
  status |= setDoubleParam(ADShutterOpenDelay, 0.);
  status |= setDoubleParam(ADShutterCloseDelay, 0.);
  
  // YF Set Default Bias off.
  status |= setIntegerParam(FCCDSetBias, 0);
  

  /// setupADCSpeeds();
  /// setupPreAmpGains();
  status |= setupShutter(-1);

  // YF Set default trigger mode 1 = Single
  checkStatus(CIN_set_trigger_mode( 1 ));

  setStringParam(AndorMessage, "Defaults Set.");
  callParamCallbacks();

  /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
  epicsEventSignal(statusEvent);

//  if (status) {
//    printf("%s:%s: unable to set camera parameters\n", driverName, functionName);
//    return;
//  }

  //Define the polling periods for the status thread.
  mPollingPeriod = 0.2; //seconds
  mFastPollingPeriod = 0.05; //seconds

  mAcquiringData = 0;
  
  if (stackSize == 0) stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("AndorStatusTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)andorStatusTaskC,
                              this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure for status task\n",
           driverName, functionName);
    return;
  }

  
  /* Create the thread that does data readout */
  status = (epicsThreadCreate("AndorDataTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)andorDataTaskC,
                              this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure for data task\n",
           driverName, functionName);
    return;
  }
  printf("CCD initialized OK!\n");
}

/**
 * Destructor.  Free resources and closes the Andor library
 */
AndorCCD::~AndorCCD() 
{
  static const char *functionName = "~AndorCCD";

  try {
    printf("Shutdown and freeing up memory...\n");
    this->lock();
    printf("Camera shutting down as part of IOC exit.\n");
    cin_data_stop_threads();
    this->unlock();
    sleep(2);
    // TODO cin_data_wait_for_threads();
    

  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
  }
  printf("Destructor done.");
}


/**
 * Exit handler, delete the AndorCCD object.
 */

static void exitHandler(void *drvPvt)
{
  AndorCCD *pAndorCCD = (AndorCCD *)drvPvt;
  delete pAndorCCD;
}


/// removed void AndorCCD::setupPreAmpGains()

asynStatus AndorCCD::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                              size_t nElements, size_t *nIn)
{
  int function = pasynUser->reason;
  int i;

  // YF debug
  printf("***DEBUG: readEnum function %d\n", function);
  
#if 0
  if (function == AndorAdcSpeed) {
    for (i=0; ((i<mNumADCSpeeds) && (i<(int)nElements)); i++) {
      if (strings[i]) free(strings[i]);
      strings[i] = epicsStrDup(mADCSpeeds[i].EnumString);
      values[i] = mADCSpeeds[i].EnumValue;
      severities[i] = 0;
    }
  }
  else if (function == AndorPreAmpGain) {
    for (i=0; ((i<mNumPreAmpGains) && (i<(int)nElements)); i++) {
      if (strings[i]) free(strings[i]);
      strings[i] = epicsStrDup(mPreAmpGains[i].EnumString);
      values[i] = mPreAmpGains[i].EnumValue;
      severities[i] = 0;
    }
  }
  else {
    *nIn = 0;
    return asynError;
  }
  
#endif
  
   return ADDriver::readEnum(pasynUser, strings, values, severities,nElements, nIn );
  
}



/// YF removed void AndorCCD::setupADCSpeeds()




/** Report status of the driver.
  * Prints details about the detector in us if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details Controls the level of detail in the report. */
void AndorCCD::report(FILE *fp, int details)
{
  int param1;
  float fParam1;
  int xsize, ysize;
  int i;
  char sParam[256];
  unsigned int uIntParam1;
  unsigned int uIntParam2;
  unsigned int uIntParam3;
  unsigned int uIntParam4;
  unsigned int uIntParam5;
  unsigned int uIntParam6;
  ///AndorADCSpeed_t *pSpeed;
  static const char *functionName = "report";

  fprintf(fp, "Andor CCD port=%s\n", this->portName);
  if (details > 0) {
    try {
      //checkStatus(GetHeadModel(sParam));
      //fprintf(fp, "  Model: %s\n", sParam);
      //checkStatus(GetCameraSerialNumber(&param1));
      //fprintf(fp, "  Serial number: %d\n", param1); 
      //checkStatus(GetHardwareVersion(&uIntParam1, &uIntParam2, &uIntParam3, 
      //                               &uIntParam4, &uIntParam5, &uIntParam6));
      //fprintf(fp, "  PCB version: %d\n", uIntParam1);
      //fprintf(fp, "  Flex file version: %d\n", uIntParam2);
      //fprintf(fp, "  Firmware version: %d\n", uIntParam5);
      //fprintf(fp, "  Firmware build: %d\n", uIntParam6);
      //checkStatus(GetVersionInfo(AT_SDKVersion, sParam, sizeof(sParam)));
      //fprintf(fp, "  SDK version: %s\n", sParam);
      //checkStatus(GetVersionInfo(AT_DeviceDriverVersion, sParam, sizeof(sParam)));
      //fprintf(fp, "  Device driver version: %s\n", sParam);
      getIntegerParam(ADMaxSizeX, &xsize);
      getIntegerParam(ADMaxSizeY, &ysize);
      fprintf(fp, "  X pixels: %d\n", xsize);
      fprintf(fp, "  Y pixels: %d\n", ysize);
      //fprintf(fp, "  Number of amplifier channels: %d\n", mNumAmps);
      fprintf(fp, "  Number of ADC channels: %d\n", mNumADCs);
      //fprintf(fp, "  Number of pre-amp gains (total): %d\n", mTotalPreAmpGains);
      //for (i=0; i<mTotalPreAmpGains; i++) {
      //  checkStatus(GetPreAmpGain(i, &fParam1));
      //  fprintf(fp, "    Gain[%d]: %f\n", i, fParam1);
      //}
      //fprintf(fp, "  Total ADC speeds: %d\n", mNumADCSpeeds);
      //for (i=0; i<mNumADCSpeeds; i++) {
      //  pSpeed = &mADCSpeeds[i];
      //  fprintf(fp, "    Amp=%d, ADC=%d, bitDepth=%d, HSSpeedIndex=%d, HSSpeed=%f\n",
      //          pSpeed->AmpIndex, pSpeed->ADCIndex, pSpeed->BitDepth, pSpeed->HSSpeedIndex, //pSpeed->HSSpeed);
      //}
      //fprintf(fp, "  Pre-amp gains available: %d\n", mNumPreAmpGains);
      //for (i=0; i<mNumPreAmpGains; i++) {
      //  fprintf(fp, "    Index=%d, Gain=%f\n",
      //          mPreAmpGains[i].EnumValue, mPreAmpGains[i].Gain);
      //}
      //capabilities.ulSize = sizeof(capabilities);
      //checkStatus(GetCapabilities(&capabilities));
      //fprintf(fp, "  Capabilities\n");
      //fprintf(fp, "        AcqModes=0x%X\n", (int)capabilities.ulAcqModes);
      //fprintf(fp, "       ReadModes=0x%X\n", (int)capabilities.ulReadModes);
      //fprintf(fp, "     FTReadModes=0x%X\n", (int)capabilities.ulFTReadModes);
      //fprintf(fp, "    TriggerModes=0x%X\n", (int)capabilities.ulTriggerModes);
      //fprintf(fp, "      CameraType=%d\n",   (int)capabilities.ulCameraType);
      //fprintf(fp, "      PixelModes=0x%X\n", (int)capabilities.ulPixelMode);
      //fprintf(fp, "    SetFunctions=0x%X\n", (int)capabilities.ulSetFunctions);
      //fprintf(fp, "    GetFunctions=0x%X\n", (int)capabilities.ulGetFunctions);
      //fprintf(fp, "        Features=0x%X\n", (int)capabilities.ulFeatures);
      //fprintf(fp, "         PCI MHz=%d\n",   (int)capabilities.ulPCICard);
      //fprintf(fp, "          EMGain=0x%X\n", (int)capabilities.ulEMGainCapability);

    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
    }
  }
  // Call the base class method
  ADDriver::report(fp, details);
}




/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus AndorCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus = 0;
    epicsInt32 oldValue;

    asynStatus status = asynSuccess;
    static const char *functionName = "writeInt32";

    //Set in param lib so the user sees a readback straight away. Save a backup in case of errors.
    getIntegerParam(function, &oldValue);
    status = setIntegerParam(function, value);

    if (function == ADAcquire) 
    {
    
      if (value)  // User clicked 'Start' button
      {
         // Send the hardware a start trigger command
         CIN_trigger_start();
          mAcquiringData = 1;
      }
      else     // User clicked 'Stop' Button
      {
         // Send the hardware a stop trigger command
         CIN_trigger_stop();
      }
      //getIntegerParam(ADStatus, &adstatus);
//      if (value && (adstatus == ADStatusIdle)) {
//        try {
//          mAcquiringData = 1;
//          //We send an event at the bottom of this function.
//        } catch (const std::string &e) {
//          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
//            "%s:%s: %s\n",
//            driverName, functionName, e.c_str());
//          status = asynError;
//        }
//      }
//      if (!value && (adstatus != ADStatusIdle)) {
//        try {
//          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
//            "%s:%s:, AbortAcquisition()\n", 
//            driverName, functionName);
//          // YF TODO  checkStatus(AbortAcquisition());
//          mAcquiringData = 0;
//          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
//            "%s:%s:, FreeInternalMemory()\n", 
//            driverName, functionName);
//          // YF TODO  checkStatus(FreeInternalMemory());
//          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
//            "%s:%s:, CancelWait()\n", 
//            driverName, functionName);
//          // YF TODO  checkStatus(CancelWait());
//       } catch (const std::string &e) {
//          asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
//            "%s:%s: %s\n",
//            driverName, functionName, e.c_str());
//          status = asynError;
//        } 
//    }
    }
    
    
    else if ((function == ADNumExposures) || (function == ADNumImages) ||
             (function == ADImageMode)                                 ||
             (function == ADBinX)         || (function == ADBinY)      ||
             (function == ADMinX)         || (function == ADMinY)      ||
             (function == ADSizeX)        || (function == ADSizeY)  )
             {
      status = setupAcquisition();

      if (status != asynSuccess) setIntegerParam(function, oldValue);
    }
    else if (function == AndorCoolerParam) {
      try {
        if (value == 0) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, CoolerOFF()\n", 
            driverName, functionName);
          // YF TODO  checkStatus(CoolerOFF());
        } else if (value == 1) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, CoolerON()\n", 
            driverName, functionName);
          // YF TODO  checkStatus(CoolerON());
        }
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    else if (function == ADShutterControl) {
      status = setupShutter(value);
    }
    else if (function == AndorShutterMode) {
      status = setupShutter(-1);
    }
    
    else if (function == FCCDSetBias) {
         CIN_set_bias(value);
    }
    else if (function == FCCDSetClocks) {
         CIN_set_clocks(value);
    }
    else if (function == ADTriggerMode) {
         // After set value, read back value
         int gts;
         CIN_set_trigger(value);
         
         gts =  CIN_get_trigger_status();
         setIntegerParam(ADTriggerMode, gts);
         // callParamCallbacks();
    }
    

    
    else {
      status = ADDriver::writeInt32(pasynUser, value);
    }

    //For a successful write, clear the error message.
    setStringParam(AndorMessage, " ");

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
    epicsEventSignal(statusEvent);

    if (mAcquiringData) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, Sending dataEvent to dataTask ...\n", 
        driverName, functionName);
      //Also signal the data readout thread
      epicsEventSignal(dataEvent);
    }

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus AndorCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64";

    int minTemp = 0;
    int maxTemp = 0;

    /* Set the parameter and readback in the parameter library.  */
    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
      mAcquireTime = (float)value; 

      CIN_set_exposure_time(mAcquireTime);
      status = asynSuccess;
      // status = setupAcquisition();
    }
    else if (function == ADAcquirePeriod) {
      mAcquirePeriod = (float)value;  
      CIN_set_cycle_time(mAcquirePeriod);
      status = asynSuccess;
    }
    else if (function == ADGain) {
      try {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetPreAmpGain(%d)\n", 
          driverName, functionName, (int)value);
        // YF TODO  checkStatus(SetPreAmpGain((int)value));
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    else if (function == AndorAccumulatePeriod) {
      mAccumulatePeriod = (float)value;  
      status = setupAcquisition();
    }
    else if (function == ADTemperature) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s:%s:, Setting temperature value %f\n", 
        driverName, functionName, value);
      try {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, CCD Min Temp: %d, Max Temp %d\n", 
          driverName, functionName, minTemp, maxTemp);
        // YF TODO  checkStatus(GetTemperatureRange(&minTemp, &maxTemp));
        if ((static_cast<int>(value) > minTemp) & (static_cast<int>(value) < maxTemp)) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetTemperature(%d)\n", 
            driverName, functionName, static_cast<int>(value));
          // YF TODO  checkStatus(SetTemperature(static_cast<int>(value)));
        } else {
          setStringParam(AndorMessage, "Temperature is out of range.");
          callParamCallbacks();
          status = asynError;
        }
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    
    else if (function == ADShutterOpenDelay) 
    {
      float fVal = (float)value;
      CIN_set_trigger_delay(fVal);
    }
    else if (function == ADShutterCloseDelay) {             
      status = setupShutter(-1);
    }
      
    else {
      status = ADDriver::writeFloat64(pasynUser, value);
    }

    //For a successful write, clear the error message.
    setStringParam(AndorMessage, " ");

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
    return status;
}


/** Controls shutter
 * @param[in] command 0=close, 1=open, -1=no change, only set other parameters */
asynStatus AndorCCD::setupShutter(int command)
{
  double dTemp;
  int openTime, closeTime;
  int shutterMode;
  asynStatus status=asynSuccess;
  static const char *functionName = "setupShutter";
  
  getDoubleParam(ADShutterOpenDelay, &dTemp);
  // Convert to ms
  openTime = (int)(dTemp * 1000.);
  getDoubleParam(ADShutterCloseDelay, &dTemp);
  closeTime = (int)(dTemp * 1000.);
  
  if (command == ADShutterClosed) {
    shutterMode = AShutterClose;
    setIntegerParam(ADShutterStatus, ADShutterClosed);
  }
  else if (command == ADShutterOpen) {
    if (shutterMode == AShutterOpen) {
      setIntegerParam(ADShutterStatus, ADShutterOpen);
    }
    // No need to change shutterMode, we leave it alone and it shutter
    // will do correct thing, i.e. auto or open depending shutterMode
  }

  return status;
}



/**
 * Expect returnStatus to be zero, otherwise display error message.
 */
unsigned int AndorCCD::checkStatus(unsigned int returnStatus)
{
  char message[256];
  if (returnStatus == 0) 
  {
    return 0;
  } 
  else 
  {
    printf("checkStatus failed\n.");
    return (-1);
  } 
}


/**
 * Update status of detector. Meant to be run in own thread.
 */
void AndorCCD::statusTask(void)
{
  int value = 0;
  // float temperature;
  // unsigned int uvalue = 0;
  unsigned int status = 0;
  double timeout = 0.0;
  unsigned int forcedFastPolls = 0;
  static const char *functionName = "statusTask";

  printf("%s:%s: Status thread started...\n", driverName, functionName);
  while(1) {

    //Read timeout for polling freq.
    this->lock();
    if (forcedFastPolls > 0) {
      timeout = mFastPollingPeriod;
      forcedFastPolls--;
    } else {
      timeout = mPollingPeriod;
    }
    this->unlock();

    if (timeout != 0.0) {
      status = epicsEventWaitWithTimeout(statusEvent, timeout);
    } else {
      status = epicsEventWait(statusEvent);
    }              
    if (status == epicsEventWaitOK) {
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Got status event\n",
        driverName, functionName);
      //We got an event, rather than a timeout.  This is because other software
      //knows that data has arrived, or device should have changed state (parameters changed, etc.).
      //Force a minimum number of fast polls, because the device status
      //might not have changed in the first few polls
      forcedFastPolls = 5;
    }

    this->lock();

#if 0
    try {
      //Only read these if we are not acquiring data
      if (!mAcquiringData) {
        //Read cooler status
        checkStatus(IsCoolerOn(&value));
        status = setIntegerParam(AndorCoolerParam, value);
        //Read temperature of CCD
        checkStatus(GetTemperatureF(&temperature));
        status = setDoubleParam(ADTemperatureActual, temperature);
      }

      //Read detector status (idle, acquiring, error, etc.)
      // YF TODO checkStatus(GetStatus(&value));
      uvalue = static_cast<unsigned int>(value);
      if (uvalue == ASIdle) {
        setIntegerParam(ADStatus, ADStatusIdle);
        setStringParam(ADStatusMessage, "IDLE. Waiting on instructions.");
      } else if (uvalue == ASTempCycle) {
        setIntegerParam(ADStatus, ADStatusWaiting);
        setStringParam(ADStatusMessage, "Executing temperature cycle.");
      } else if (uvalue == ASAcquiring) {
        setIntegerParam(ADStatus, ADStatusAcquire);
        setStringParam(ADStatusMessage, "Data acquisition in progress.");
      } else if (uvalue == ASAccumTimeNotMet) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Unable to meet accumulate time.");
      } else if (uvalue == ASKineticTimeNotMet) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Unable to meet kinetic cycle time.");
      } else if (uvalue == ASErrorAck) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Unable to communicate with device.");
      } else if (uvalue == ASAcqBuffer) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Computer unable to read data from device at required rate.");
      } else if (uvalue == ASSpoolError) {
        setIntegerParam(ADStatus, ADStatusError);
        setStringParam(ADStatusMessage, "Overflow of the spool buffer.");
      }
    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
      setStringParam(AndorMessage, e.c_str());
    }
#endif    

    /* Call the callbacks to update any changes */
    callParamCallbacks();
    this->unlock();
        
  } //End of loop

}

/** Set up acquisition parameters */
asynStatus AndorCCD::setupAcquisition()
{
   int numExposures;
   int numImages;
   int imageMode;
   ///int adcSpeed;
   int triggerMode;
   int binX, binY, minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
   //float acquireTimeAct, acquirePeriodAct, accumulatePeriodAct;
   // int FKmode = 4;
   //int FKOffset;
   // AndorADCSpeed_t *pSpeed;
   static const char *functionName = "setupAcquisition";

   getIntegerParam(ADImageMode, &imageMode);
   getIntegerParam(ADNumExposures, &numExposures);
   if (numExposures <= 0) {
    numExposures = 1;
    setIntegerParam(ADNumExposures, numExposures);
   }
   getIntegerParam(ADNumImages, &numImages);
   if (numImages <= 0) {
    numImages = 1;
    setIntegerParam(ADNumImages, numImages);
   }
   getIntegerParam(ADBinX, &binX);
   if (binX <= 0) {
    binX = 1;
    setIntegerParam(ADBinX, binX);
   }
   getIntegerParam(ADBinY, &binY);
   if (binY <= 0) {
    binY = 1;
    setIntegerParam(ADBinY, binY);
   }
   getIntegerParam(ADMinX, &minX);
   getIntegerParam(ADMinY, &minY);
   getIntegerParam(ADSizeX, &sizeX);
   getIntegerParam(ADSizeY, &sizeY);
   getIntegerParam(ADMaxSizeX, &maxSizeX);
   getIntegerParam(ADMaxSizeY, &maxSizeY);
   if (minX > (maxSizeX - 2*binX)) {
    minX = maxSizeX - 2*binX;
    setIntegerParam(ADMinX, minX);
   }
   if (minY > (maxSizeY - 2*binY)) {
    minY = maxSizeY - 2*binY;
    setIntegerParam(ADMinY, minY);
   }
   if ((minX + sizeX) > maxSizeX) {
    sizeX = maxSizeX - minX;
    setIntegerParam(ADSizeX, sizeX);
   }
   if ((minY + sizeY) > maxSizeY) {
    sizeY = maxSizeY - minY;
    setIntegerParam(ADSizeY, sizeY);
   }
  
   // Note: we do triggerMode and adcChannel in this function because they change
   // the computed actual AcquirePeriod and AccumulatePeriod
   getIntegerParam(ADTriggerMode, &triggerMode);
   ///getIntegerParam(AndorAdcSpeed, &adcSpeed);
   ///pSpeed = &mADCSpeeds[adcSpeed];

   // Unfortunately there does not seem to be a way to query the Andor SDK 
   // for the actual size of the image, so we must compute it.
   setIntegerParam(NDArraySizeX, sizeX/binX);
   setIntegerParam(NDArraySizeY, sizeY/binY);
  
   //
   // YF New code to support setting the trigger mode
   //
   try
   {

      switch (imageMode) 
      {
         case ADImageSingle:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
               "%s:%s:, CIN_set_trigger_mode(1)\n", driverName, functionName);
             // Set Hardware to single trigger mode.
             // This also sets number of exposures = 1
            checkStatus(CIN_set_trigger_mode( 1 ) ); // Single Image mode
            break;

         case ADImageMultiple:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
               "%s:%s:, CIN_set_trigger_mode(n)\n", 
               driverName, functionName);
            // YF Assume mode should be continuous mode - not sure.
            checkStatus(CIN_set_trigger_mode( numImages ) );  // Multiple Image Mode
            
            break;

         case ADImageContinuous:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
               "%s:%s:, CIN_set_trigger_mode(0)\n", 
               driverName, functionName);
            checkStatus(CIN_set_trigger_mode( 0 ) );  // Continuous mode    
            break;

      } // switch
   } 
   catch (const std::string &e) 
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,  "%s:%s: %s\n",
         driverName, functionName, e.c_str());
      return asynError;
   }
    
#if 0 // Andor specfic stuff  
  try {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetTriggerMode(%d)\n", 
      driverName, functionName, triggerMode);
    checkStatus(SetTriggerMode(triggerMode));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetADChannel(%d)\n", 
      driverName, functionName, pSpeed->ADCIndex);
    checkStatus(SetADChannel(pSpeed->ADCIndex));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetOutputAmplifier(%d)\n", 
      driverName, functionName, pSpeed->AmpIndex);
    checkStatus(SetOutputAmplifier(pSpeed->AmpIndex));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetHSSpeed(%d, %d)\n", 
      driverName, functionName, pSpeed->AmpIndex, pSpeed->HSSpeedIndex);
    checkStatus(SetHSSpeed(pSpeed->AmpIndex, pSpeed->HSSpeedIndex));

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetImage(%d,%d,%d,%d,%d,%d)\n", 
      driverName, functionName, binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY);
    checkStatus(SetImage(binX, binY, minX+1, minX+sizeX, minY+1, minY+sizeY));


    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      "%s:%s:, SetExposureTime(%f)\n", 
      driverName, functionName, mAcquireTime);
    checkStatus(SetExposureTime(mAcquireTime));
    

   switch (imageMode) {
      case ADImageSingle:
        if (numExposures == 1) {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetAcquisitionMode(AASingle)\n", 
            driverName, functionName);
          checkStatus(SetAcquisitionMode(AASingle));
        } else {
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetAcquisitionMode(AAAccumulate)\n", 
            driverName, functionName);
          checkStatus(SetAcquisitionMode(AAAccumulate));
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetNumberAccumulations(%d)\n", 
            driverName, functionName, numExposures);
          checkStatus(SetNumberAccumulations(numExposures));
          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s:, SetAccumulationCycleTime(%f)\n", 
            driverName, functionName, mAccumulatePeriod);
          checkStatus(SetAccumulationCycleTime(mAccumulatePeriod));
        }
        break;

      case ADImageMultiple:
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAcquisitionMode(AAKinetics)\n", 
          driverName, functionName);
        checkStatus(SetAcquisitionMode(AAKinetics));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetNumberAccumulations(%d)\n", 
          driverName, functionName, numExposures);
        checkStatus(SetNumberAccumulations(numExposures));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAccumulationCycleTime(%f)\n", 
          driverName, functionName, mAccumulatePeriod);
        checkStatus(SetAccumulationCycleTime(mAccumulatePeriod));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetNumberKinetics(%d)\n", 
          driverName, functionName, numImages);
        checkStatus(SetNumberKinetics(numImages));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetKineticCycleTime(%f)\n", 
          driverName, functionName, mAcquirePeriod);
        checkStatus(SetKineticCycleTime(mAcquirePeriod));
        break;

      case ADImageContinuous:
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAcquisitionMode(AARunTillAbort)\n", 
          driverName, functionName);
        checkStatus(SetAcquisitionMode(AARunTillAbort));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetKineticCycleTime(%f)\n", 
          driverName, functionName, mAcquirePeriod);
        checkStatus(SetKineticCycleTime(mAcquirePeriod));
        break;

      case AImageFastKinetics:
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetAcquisitionMode(AAFastKinetics)\n", 
          driverName, functionName);
        checkStatus(SetAcquisitionMode(AAFastKinetics));
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetImage(%d,%d,%d,%d,%d,%d)\n", 
          driverName, functionName, binX, binY, 1, maxSizeX, 1, maxSizeY);
        checkStatus(SetImage(binX, binY, 1, maxSizeX, 1, maxSizeY));
        FKOffset = maxSizeY - sizeY - minY;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, SetFastKineticsEx(%d,%d,%f,%d,%d,%d,%d)\n", 
          driverName, functionName, sizeY, numImages, mAcquireTime, FKmode, binX, binY, FKOffset);
        checkStatus(SetFastKineticsEx(sizeY, numImages, mAcquireTime, FKmode, binX, binY, FKOffset));
        setIntegerParam(NDArraySizeX, maxSizeX/binX);
        setIntegerParam(NDArraySizeY, sizeY/binY);
        break;
    }
    // Read the actual times
    if (imageMode == AImageFastKinetics) {
      checkStatus(GetFKExposureTime(&acquireTimeAct));
    } else {
      checkStatus(GetAcquisitionTimings(&acquireTimeAct, &accumulatePeriodAct, &acquirePeriodAct));
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s:, GetAcquisitionTimings(exposure=%f, accumulate=%f, kinetic=%f)\n",
        driverName, functionName, acquireTimeAct, accumulatePeriodAct, acquirePeriodAct);
    }
    setDoubleParam(ADAcquireTime, acquireTimeAct);
    setDoubleParam(ADAcquirePeriod, acquirePeriodAct);
    setDoubleParam(AndorAccumulatePeriod, accumulatePeriodAct);

  
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    return asynError;
  }
  
 #endif
  
   return asynSuccess;
}


/**
 * Do data readout from the detector. Meant to be run in own thread.
 */
void AndorCCD::dataTask(void)
{
  epicsUInt32 status = 0;
  // int acquireStatus;
  char *errorString = NULL;
  int acquiring = 0;
  int imageMode;
  epicsInt32 numImages;
  epicsInt32 numImagesCounter;
  epicsInt32 numExposuresCounter;
  epicsInt32 imageCounter;
  epicsInt32 arrayCallbacks;
  epicsInt32 sizeX, sizeY;
  NDDataType_t dataType;
  int itemp;
  //at_32 firstImage, lastImage;
  //at_32 validFirst, validLast;
  //size_t dims[2];
  //int nDims = 2;
  // int i;
  epicsTimeStamp startTime;
  
  int autoSave;
  static const char *functionName = "dataTask";

  printf("%s:%s: Data thread started...\n", driverName, functionName);
  
  this->lock();

  while(1) {
    
    errorString = NULL;

    //Wait for event from main thread to signal that data acquisition has started.
    this->unlock();
    status = epicsEventWait(dataEvent);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s:, got data event\n", 
      driverName, functionName);
    this->lock();

    //Sanity check that main thread thinks we are acquiring data
    if (mAcquiringData) {
      try {
        status = setupAcquisition();
        ///if (status != asynSuccess) continue;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, StartAcquisition()\n", 
          driverName, functionName);
        /// YF TODO checkStatus(StartAcquisition());
        // YF Need to do anything to start acquisition?
        acquiring = 1;
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        continue;
      }
      //Read some parameters
      getIntegerParam(NDDataType, &itemp); dataType = (NDDataType_t)itemp;
      getIntegerParam(NDAutoSave, &autoSave);
      getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
      getIntegerParam(NDArraySizeX, &sizeX);
      getIntegerParam(NDArraySizeY, &sizeY);
      getIntegerParam(ADNumImages, &numImages);
      // Reset the counters
      setIntegerParam(ADNumImagesCounter, 0);
      setIntegerParam(ADNumExposuresCounter, 0);
      getIntegerParam(ADImageMode, &imageMode);
      setIntegerParam(ADStatus, 1 /*"Acquire", def in ADBase.template */); 
      
      callParamCallbacks();
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
        "%s:%s:, Data thread is running but main thread thinks we are not acquiring.\n", 
        driverName, functionName);
      acquiring = 0;
    }

    while (acquiring && mAcquiringData) 
    {
      try {
         
         this->unlock();
            status = FCCD_GetImage(); 
         this->lock();
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s:%s:, Got an image.\n", driverName, functionName);
         getIntegerParam(ADNumExposuresCounter, &numExposuresCounter);
         numExposuresCounter++;
         setIntegerParam(ADNumExposuresCounter, numExposuresCounter);
         callParamCallbacks();

         getIntegerParam(NDArrayCounter, &imageCounter);
         imageCounter++;
         setIntegerParam(NDArrayCounter, imageCounter);;
         getIntegerParam(ADNumImagesCounter, &numImagesCounter);
         numImagesCounter++;
         setIntegerParam(ADNumImagesCounter, numImagesCounter);
         // If array callbacks are enabled then read data into NDArray, do callbacks
         // YF
         arrayCallbacks = 1; // kick it.
          
          if (arrayCallbacks && (status == 0) ) {
            epicsTimeGetCurrent(&startTime);
            
            if (m_pArray)
            {
                           
               setIntegerParam(NDArraySize, sizeX * sizeY * sizeof(uint16_t));
         
               /* Put the frame number and time stamp into the buffer */
               m_pArray->uniqueId = imageCounter;
               m_pArray->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
               /* Get any attributes that have been defined for this driver */        
               this->getAttributes(m_pArray->pAttributeList);
               /* Call the NDArray callback */
               /* Must release the lock here, or we can get into a deadlock, because we can
                * block on the plugin lock, and the plugin can be calling us */
               this->unlock();
               asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                    "%s:%s:, calling array callbacks\n", 
                    driverName, functionName);
                    
               // printf("***3***\n"); // DEBUG
                 
                 
               doCallbacksGenericPointer(m_pArray, NDArrayData, 0);
               this->lock();
               m_pArray->release();
               // Calling release does not free any memory, but allows
               // NDPoolArray to reuse object
               // 
            
            }
            else
            {
               // allocate error!
               printf("Out of memory in data task!\n");
            }
          } // if (arrayCallbacks) {

          callParamCallbacks();

      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        errorString = const_cast<char *>(e.c_str());
        setStringParam(AndorMessage, errorString);
      }
      
      
      // printf("***5***\n"); // DEBUG
       
       // Never exit the acquire loop.
       // Instead rely on hardware to switch from single trigger to continuous mode
      /* See if acquisition is done */
//      if ((imageMode == ADImageSingle) ||
//         ((imageMode == ADImageMultiple) &&
//          (numImagesCounter >= numImages))) {
//          
//            acquiring = 0;
//      }
    } // while acquiring
      
    //Now clear main thread flag
    mAcquiringData = 0;
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, 0); 

    /* Call the callbacks to update any changes */
    callParamCallbacks();
  } //End of loop

}


/**
 * Save a data frame using the Andor SDK file writing functions.
 */
/// removed void AndorCCD::saveDataFrame(int frameNumber) 


//C utility functions to tie in with EPICS

static void andorStatusTaskC(void *drvPvt)
{
  AndorCCD *pPvt = (AndorCCD *)drvPvt;

  pPvt->statusTask();
}


static void andorDataTaskC(void *drvPvt)
{
  AndorCCD *pPvt = (AndorCCD *)drvPvt;

  pPvt->dataTask();
}

/** IOC shell configuration command for Andor driver
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] installPath The path to the Andor directory containing the detector INI files, etc.
  *            This can be specified as an empty string ("") for new detectors that don't use the INI
  *            files on Windows, but must be a valid path on Linux.
  * \param[in] priority The thread priority for the asyn port driver thread
  * \param[in] stackSize The stack size for the asyn port driver thread
  */
extern "C" {
int andorCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, 
                   const char *installPath, int priority, int stackSize)
{
  /*Instantiate class.*/
  new AndorCCD(portName, maxBuffers, maxMemory, installPath, priority, stackSize);
  return(asynSuccess);
}


// 
// IOC shell configuration command for cin power up
//  
int FCCD_cin_power_up(const char *strParam)
{
   printf("cin_power_up: %s\n", strParam);
   cin_power_up(); // defined in cin_power.c
   return (0);
}

// 
// IOC shell configuration command for cin power down
//  
int FCCD_cin_power_down(const char *strParam)
{
   cin_power_down(); // defined in cin_power.c
   return (0);
}

/* Code for iocsh registration */

/* andorCCDConfig */
static const iocshArg andorCCDConfigArg0 = {"Port name", iocshArgString};
static const iocshArg andorCCDConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg andorCCDConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg andorCCDConfigArg3 = {"installPath", iocshArgString};
static const iocshArg andorCCDConfigArg4 = {"priority", iocshArgInt};
static const iocshArg andorCCDConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const andorCCDConfigArgs[] =  {&andorCCDConfigArg0,
                                                       &andorCCDConfigArg1,
                                                       &andorCCDConfigArg2,
                                                       &andorCCDConfigArg3,
                                                       &andorCCDConfigArg4,
                                                       &andorCCDConfigArg5};

static const iocshFuncDef configAndorCCD = {"andorCCDConfig", 6, andorCCDConfigArgs};
static void configAndorCCDCallFunc(const iocshArgBuf *args)
{
    andorCCDConfig(args[0].sval, args[1].ival, args[2].ival, args[3].sval, 
                   args[4].ival, args[5].ival);
}

static void fCCDRegister(void)
{

    iocshRegister(&configAndorCCD, configAndorCCDCallFunc);
}




/* Information needed by iocsh */
static const iocshArg     cin_power_upArg0 = {"strParam", iocshArgString};
static const iocshArg    *cin_power_upArgs[] = {&cin_power_upArg0};
static const iocshFuncDef cin_power_upFuncDef = {"FCCD_cin_power_up", 1, cin_power_upArgs};
static const iocshFuncDef cin_power_downFuncDef = {"FCCD_cin_power_down", 1, cin_power_upArgs};


static void cin_power_upCallFunc(const iocshArgBuf *args)
{
    FCCD_cin_power_up(args[0].sval);
}

static void cin_power_downCallFunc(const iocshArgBuf *args)
{
    FCCD_cin_power_down(args[0].sval);
}


static void cin_power_upRegister(void)
{
    iocshRegister(&cin_power_upFuncDef, cin_power_upCallFunc);
}

static void cin_power_downRegister(void)
{
    iocshRegister(&cin_power_downFuncDef, cin_power_downCallFunc);
}

epicsExportRegistrar(fCCDRegister);
epicsExportRegistrar(cin_power_upRegister);
epicsExportRegistrar(cin_power_downRegister);

} // extern "C"

