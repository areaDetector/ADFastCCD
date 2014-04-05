/**
 * Area Detector driver for the LBNL FastCCD
 * Modifed by S. Wilkins
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

#include "cin.h"
#include "fCCD.h"

static const char *driverName = "fCCD";

//Definitions of static class data members

const epicsUInt32 FastCCD::ATInternal = 0;
const epicsUInt32 FastCCD::ATExternal1 = 1;
const epicsUInt32 FastCCD::ATExternal2 = 2;
const epicsUInt32 FastCCD::ATExternal1or2 = 3;

const epicsInt32 FastCCD::AShutterAuto = 0;
const epicsInt32 FastCCD::AShutterOpen = 1;
const epicsInt32 FastCCD::AShutterClose = 2;


//C Function prototypes to tie in with EPICS
static void FCCDStatusTaskC(void *drvPvt);
static void FCCDDataTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);

asynStatus FastCCD::connect(asynUser *pasynUser){
  return connectCamera();
}

asynStatus FastCCD::connectCamera(){

  int ret= 0;

  if(cin_init_data_port(&cin_data_port, NULL, 0, "10.23.5.127", 0, 1000))
    return asynError;

  if(cin_ctl_init_port(&cin_ctl_port, NULL, 0, 0))
    return asynError;
  if(cin_ctl_init_port(&cin_ctl_port_stream, NULL, 49202, 50202))
    return asynError;
   
  if( (ret = cin_data_init(CIN_DATA_MODE_PUSH_PULL, 2000, 2000)))
    return asynError;
 
  printf("Connected\n");

  return asynSuccess;
}

asynStatus FastCCD::disconnect(asynUser *pasynUser){
  return disconnectCamera();
}

asynStatus FastCCD::disconnectCamera(){
  cin_ctl_close_port(&cin_ctl_port);
  cin_ctl_close_port(&cin_ctl_port_stream);
  return asynSuccess; 
}

int FastCCD::GetImage() 
{
   size_t dims[2];
   int nDims = 2;
   uint16_t frame_number;
   NDDataType_t dataType;
   
   dims[0] = CIN_DATA_FRAME_WIDTH;
   dims[1] = CIN_DATA_FRAME_HEIGHT;
   dataType = NDUInt16; 
   
   m_pArray = this->pNDArrayPool->alloc(nDims, dims, dataType, 
                                        0, NULL);
      
   if(m_pArray)
   {
      // Load the buffer. Pass in memory allocated in NDArrayPool
      cin_data_load_frame((uint16_t *)m_pArray->pData, &frame_number);
      printf("Got frame %d\n", frame_number);
      return (0);
   }

   return (-1); // error
}

/** Constructor for FCCD driver; most parameters are simply passed to ADDriver::ADDriver.
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
FastCCD::FastCCD(const char *portName, int maxBuffers, size_t maxMemory, 
                   const char *installPath, int priority, int stackSize)

  : ADDriver(portName, 1, NUM_FCCD_DET_PARAMS, maxBuffers, maxMemory, 
             asynEnumMask, asynEnumMask,
             ASYN_CANBLOCK, 1, priority, stackSize)
{

  int status = asynSuccess;
  int i;
  int sizeX, sizeY;
  
  static const char *functionName = "FastCCD";

  /* Create an EPICS exit handler */
  epicsAtExit(exitHandler, this);

  //createParam(ADStatusMessageString,                 asynParamOctet, &ADStatusMessage);
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

  try {
    printf("%s:%s: initializing camera\n",
           driverName, functionName);
  
    this->lock();
    connectCamera();
 
    // Set some parameters as default

    cin_ctl_set_focus(&cin_ctl_port, 1);
 
    this->unlock();

    setStringParam(ADStatusMessage, "Camera successfully initialized.");
    callParamCallbacks();
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    return;
  }

  sizeX = CIN_DATA_FRAME_WIDTH;
  sizeY = CIN_DATA_FRAME_HEIGHT;

  /* Set some default values for parameters */
  status =  setStringParam(ADManufacturer, "LB National Lab.");
  status |= setStringParam(ADModel, "1k x 2k FastCCD"); // Model ?
  status |= setIntegerParam(ADSizeX, sizeX);
  status |= setIntegerParam(ADSizeY, sizeY);
  status |= setIntegerParam(ADBinX, 1);
  status |= setIntegerParam(ADBinY, 1);
  status |= setIntegerParam(ADMinX, 0);
  status |= setIntegerParam(ADMinY, 0);
  status |= setIntegerParam(ADMaxSizeX, sizeX);
  status |= setIntegerParam(ADMaxSizeY, sizeY);  
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, FastCCD::ATInternal);
  mAcquireTime = 0.005;
  status |= setDoubleParam (ADAcquireTime, mAcquireTime);
  mAcquirePeriod = 0.5;
  status |= setDoubleParam (ADAcquirePeriod, mAcquirePeriod);
  status |= setIntegerParam(ADNumImages, 1);
  status |= setIntegerParam(ADNumExposures, 1);
  status |= setIntegerParam(NDArraySizeX, sizeX);
  status |= setIntegerParam(NDArraySizeY, sizeY);
  status |= setIntegerParam(NDDataType, NDUInt16);
  status |= setIntegerParam(NDArraySize, sizeX*sizeY*sizeof(epicsUInt16)); 
  status |= setDoubleParam(ADShutterOpenDelay, 0.);
  status |= setDoubleParam(ADShutterCloseDelay, 0.);
  status |= setIntegerParam(FCCDSetBias, 0);
  

  setStringParam(ADStatusMessage, "Defaults Set.");
  callParamCallbacks();

  /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
  epicsEventSignal(statusEvent);

  //Define the polling periods for the status thread.
  mPollingPeriod = 0.2; //seconds
  mFastPollingPeriod = 0.05; //seconds

  mAcquiringData = 0;
  
  if (stackSize == 0) stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("FCCDStatusTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)FCCDStatusTaskC,
                              this) == NULL);
  if(status) {
    printf("%s:%s: epicsThreadCreate failure for status task\n",
           driverName, functionName);
    return;
  }

  /* Create the thread that does data readout */
  status = (epicsThreadCreate("FCCDDataTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)FCCDDataTaskC,
                              this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure for data task\n",
           driverName, functionName);
    return;
  }
}

/**
 * Destructor.  Free resources and closes the FCCD library
 */
FastCCD::~FastCCD() 
{
  static const char *functionName = "~FastCCD";

  try {
    printf("Shutdown and freeing up memory...\n");
    this->lock();
    printf("Camera shutting down as part of IOC exit.\n");
    cin_data_stop_threads();
    cin_data_wait_for_threads();
    this->unlock();
    sleep(2);

  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
  }
}


/**
 * Exit handler, delete the FastCCD object.
 */

static void exitHandler(void *drvPvt)
{
  FastCCD *pFastCCD = (FastCCD *)drvPvt;
  delete pFastCCD;
}


asynStatus FastCCD::readEnum(asynUser *pasynUser, char *strings[], int values[], 
                             int severities[], size_t nElements, size_t *nIn)
{
  int function = pasynUser->reason;
  int i;

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


/** Report status of the driver.
  * Prints details about the detector in us if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details Controls the level of detail in the report. */
void FastCCD::report(FILE *fp, int details)
{
  int xsize, ysize;
  static const char *functionName = "report";

  fprintf(fp, "FastCCD CCD port = %s\n", this->portName);
  if (details > 0) {
    try {
      getIntegerParam(ADMaxSizeX, &xsize);
      getIntegerParam(ADMaxSizeY, &ysize);
      fprintf(fp, "  X pixels: %d\n", xsize);
      fprintf(fp, "  Y pixels: %d\n", ysize);

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
asynStatus FastCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
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
      if (value) // User clicked 'Start' button
      {
         // Send the hardware a start trigger command
         int n_images, t_mode, i_mode;
         getIntegerParam(ADTriggerMode, &t_mode);
         getIntegerParam(ADNumImages, &n_images);
         getIntegerParam(ADImageMode, &i_mode);

         if(i_mode == ADImageSingle){
           n_images = 1;
         } else if(i_mode == ADImageContinuous){
           n_images = 0;
         }

         if(t_mode == FastCCD::ATInternal){
           if(!cin_ctl_int_trigger_start(&cin_ctl_port, n_images)){
             mAcquiringData = 1;
             setIntegerParam(ADStatus, ADStatusAcquire);
           }
         } else {
           if(!cin_ctl_ext_trigger_start(&cin_ctl_port, t_mode)){
             mAcquiringData = 0;
             setIntegerParam(ADStatus, ADStatusAcquire);
           }
         }
      }
      else // User clicked 'Stop' Button
      {
         // Send the hardware a stop trigger command
         if(!cin_ctl_int_trigger_stop(&cin_ctl_port)){
           setIntegerParam(ADStatus, ADStatusIdle);
         }
         if(!cin_ctl_ext_trigger_stop(&cin_ctl_port)){
           setIntegerParam(ADStatus, ADStatusIdle);
         }
      }
      //getIntegerParam(ADStatus, &adstatus);
    }
    else if ((function == ADNumExposures) || (function == ADNumImages)   ||
             (function == ADImageMode)    ||                             
             (function == ADBinX)         || (function == ADBinY)        ||
             (function == ADMinX)         || (function == ADMinY)        ||
             (function == ADSizeX)        || (function == ADSizeY)  )
             {
      status = setupAcquisition();

      if (status != asynSuccess) setIntegerParam(function, oldValue);
    } 
    // else if (function == FCCDSetBias) {
    //      cin_ctl_set_bias(cin_ctl_port, value);
    // }
    // else if (function == FCCDSetClocks) {
    //      cin_ctl_set_clocks(cin_ctl_port, value);
    // }
    // else if (function == ADTriggerMode) {
    //      // Cache this as it is needed by
    //      // the start triggers
    //      mTriggerMode = value;
    //      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    //                "%s:%s:, trigger mode set to %d\n", 
    //                driverName, functionName, value);
    // }
    else {
      status = ADDriver::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    /* Send a signal to the poller task which will make it 
       do a poll, and switch to the fast poll rate */
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
asynStatus FastCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64";

    /* Set the parameter and readback in the parameter library.  */
    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
      mAcquireTime = (float)value; 
      cin_ctl_set_exposure_time(&cin_ctl_port, mAcquireTime);
      status = asynSuccess;
      // status = setupAcquisition();
    }
    else if (function == ADAcquirePeriod) {
      mAcquirePeriod = (float)value;  
      cin_ctl_set_cycle_time(&cin_ctl_port, mAcquirePeriod);
      status = asynSuccess;
    }
    // else if (function == ADGain) {
    //   try {
    //     asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    //       "%s:%s:, SetPreAmpGain(%d)\n", 
    //       driverName, functionName, (int)value);
    //   } catch (const std::string &e) {
    //     asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
    //       "%s:%s: %s\n",
    //       driverName, functionName, e.c_str());
    //     status = asynError;
    //   }
    // }
    // else if (function == AndorAccumulatePeriod) {
    //   mAccumulatePeriod = (float)value;  
    //   status = setupAcquisition();
    // }
    // else if (function == ADTemperature) {
    //   asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    //     "%s:%s:, Setting temperature value %f\n", 
    //     driverName, functionName, value);
    //   try {
    //     asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    //       "%s:%s:, CCD Min Temp: %d, Max Temp %d\n", 
    //       driverName, functionName, minTemp, maxTemp);
    //     // YF TODO  checkStatus(GetTemperatureRange(&minTemp, &maxTemp));
    //     if ((static_cast<int>(value) > minTemp) & (static_cast<int>(value) < maxTemp)) {
    //       asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
    //         "%s:%s:, SetTemperature(%d)\n", 
    //         driverName, functionName, static_cast<int>(value));
    //       // YF TODO  checkStatus(SetTemperature(static_cast<int>(value)));
    //     } else {
    //       setStringParam(ADStatusMessage, "Temperature is out of range.");
    //       callParamCallbacks();
    //       status = asynError;
    //     }
    //   } catch (const std::string &e) {
    //     asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
    //       "%s:%s: %s\n",
    //       driverName, functionName, e.c_str());
    //     status = asynError;
    //   }
    // }
    // 
    //else if (function == ADShutterOpenDelay) 
    //{
    //  float fVal = (float)value;
    //  CIN_set_trigger_delay(fVal);
    //}
    else {
      status = ADDriver::writeFloat64(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status)
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
// asynStatus FastCCD::setupShutter(int command)
// {
//   double dTemp;
//   int openTime, closeTime;
//   int shutterMode;
//   asynStatus status=asynSuccess;
//   static const char *functionName = "setupShutter";
//   
//   getDoubleParam(ADShutterOpenDelay, &dTemp);
//   // Convert to ms
//   openTime = (int)(dTemp * 1000.);
//   getDoubleParam(ADShutterCloseDelay, &dTemp);
//   closeTime = (int)(dTemp * 1000.);
//   
//   if (command == ADShutterClosed) {
//     shutterMode = AShutterClose;
//     setIntegerParam(ADShutterStatus, ADShutterClosed);
//   }
//   else if (command == ADShutterOpen) {
//     if (shutterMode == AShutterOpen) {
//       setIntegerParam(ADShutterStatus, ADShutterOpen);
//     }
//     // No need to change shutterMode, we leave it alone and it shutter
//     // will do correct thing, i.e. auto or open depending shutterMode
//   }
// 
//   return status;
// }



/**
 * Expect returnStatus to be zero, otherwise display error message.
 */
unsigned int FastCCD::checkStatus(unsigned int returnStatus)
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
void FastCCD::statusTask(void)
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
      setStringParam(ADStatusMessage, e.c_str());
    }
#endif    

    /* Call the callbacks to update any changes */
    callParamCallbacks();
    this->unlock();
        
  } //End of loop

}

/** Set up acquisition parameters */
asynStatus FastCCD::setupAcquisition()
{
   int numExposures;
   int numImages;
   int imageMode;
   int triggerMode;
   int binX, binY, minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;
   static const char *functionName = "setupAcquisition";

   getIntegerParam(ADImageMode, &imageMode);
   getIntegerParam(ADNumExposures, &numExposures);
   if(numExposures <= 0) {
    numExposures = 1;
    setIntegerParam(ADNumExposures, numExposures);
   }
   getIntegerParam(ADNumImages, &numImages);
   if(numImages <= 0) {
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
  
   getIntegerParam(ADTriggerMode, &triggerMode);

   setIntegerParam(NDArraySizeX, sizeX/binX);
   setIntegerParam(NDArraySizeY, sizeY/binY);
  
   // try
   // {
   //    switch (imageMode) 
   //    {
   //       case ADImageSingle:
   //          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
   //             "%s:%s:, CIN_set_trigger_mode(1)\n", driverName, functionName);
   //           // Set Hardware to single trigger mode.
   //           // This also sets number of exposures = 1
   //          checkStatus(CIN_set_trigger_mode(1)); // Single Image mode
   //          break;

   //       case ADImageMultiple:
   //          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
   //             "%s:%s:, CIN_set_trigger_mode(n)\n", 
   //             driverName, functionName);
   //          checkStatus(CIN_set_trigger_mode(numImages) );  // Multiple Image Mode
   //          break;

   //       case ADImageContinuous:
   //          asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
   //             "%s:%s:, CIN_set_trigger_mode(0)\n", 
   //             driverName, functionName);
   //          checkStatus(CIN_set_trigger_mode(0) );  // Continuous mode    
   //          break;

   //    } // switch
   // } 
   //  catch (const std::string &e) 
   //  {
   //     asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,  "%s:%s: %s\n",
   //        driverName, functionName, e.c_str());
   //     return asynError;
   //  }
   // 
   return asynSuccess;
}


/**
 * Do data readout from the detector. Meant to be run in own thread.
 */
void FastCCD::dataTask(void)
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
  epicsTimeStamp startTime;
  
  int autoSave;
  static const char *functionName = "dataTask";

  printf("%s:%s: Data thread started...\n", driverName, functionName);
  
  this->lock();

  while(1) {
    
    errorString = NULL;

    // Wait for event from main thread to signal that data acquisition has started.
    this->unlock();
    status = epicsEventWait(dataEvent);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, got data event\n", 
              driverName, functionName);
    this->lock();

    //Sanity check that main thread thinks we are acquiring data
    // if (mAcquiringData) {
      // try {
      //   status = setupAcquisition();
      //   ///if (status != asynSuccess) continue;
      //   asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
      //     "%s:%s:, StartAcquisition()\n", 
      //     driverName, functionName);
      //   /// YF TODO checkStatus(StartAcquisition());
      //   // YF Need to do anything to start acquisition?
      //   acquiring = 1;
      // } catch (const std::string &e) {
      //   asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      //     "%s:%s: %s\n",
      //     driverName, functionName, e.c_str());
      //   continue;
      // }
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
    // } else {
    //   asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
    //     "%s:%s:, Data thread is running but main thread thinks we are not acquiring.\n", 
    //     driverName, functionName);
    //   acquiring = 0;
    // }
  
    acquiring = 1;
    while (acquiring)
    {
      try {
         // Get the image, unlocking the mutex as we block here.
         this->unlock();
         printf("Waiting for data .....\n");
         status = GetImage(); 
         this->lock();
         asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                   "%s:%s:, Got an image.\n", driverName, functionName);
         
         // Increment the number of exposures counter
         getIntegerParam(ADNumExposuresCounter, &numExposuresCounter);
         numExposuresCounter++;
         setIntegerParam(ADNumExposuresCounter, numExposuresCounter);
         callParamCallbacks();

         // Increment the array counter
         getIntegerParam(NDArrayCounter, &imageCounter);
         imageCounter++;
         setIntegerParam(NDArrayCounter, imageCounter);;
         getIntegerParam(ADNumImagesCounter, &numImagesCounter);
         numImagesCounter++;
         setIntegerParam(ADNumImagesCounter, numImagesCounter);
          
         if (arrayCallbacks && (status == 0) ) {
            epicsTimeGetCurrent(&startTime);
            
            if (m_pArray)
            {
               setIntegerParam(NDArraySize, sizeX * sizeY * sizeof(uint16_t));
         
               /* Put the frame number and time stamp into the buffer */
               m_pArray->uniqueId = imageCounter; // SBW: Should this come from the CIN?
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
                    
               doCallbacksGenericPointer(m_pArray, NDArrayData, 0);
               this->lock();
               m_pArray->release();
            }
            else
            {
               printf("Out of memory in data task!\n");
            }
          } 

          callParamCallbacks();

      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: %s\n",
          driverName, functionName, e.c_str());
        errorString = const_cast<char *>(e.c_str());
        setStringParam(ADStatusMessage, errorString);
      }
      
      // Never exit the acquire loop.
      // See here if the acquisition is done 
      /* See if acquisition is done */
      printf("numImages = %d\n", numImagesCounter);
      if ((imageMode == ADImageSingle) ||
         ((imageMode == ADImageMultiple) &&
          (numImagesCounter >= numImages))) {
            acquiring = 0;
      }
    } // while acquiring
      
    //Now clear main thread flag
    mAcquiringData = 0;
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, 0); 

    /* Call the callbacks to update any changes */
    callParamCallbacks();
  } //End of loop

}


//C utility functions to tie in with EPICS

static void FCCDStatusTaskC(void *drvPvt)
{
  FastCCD *pPvt = (FastCCD *)drvPvt;

  pPvt->statusTask();
}


static void FCCDDataTaskC(void *drvPvt)
{
  FastCCD *pPvt = (FastCCD *)drvPvt;

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
int FCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, 
                   const char *installPath, int priority, int stackSize)
{
  /*Instantiate class.*/
  new FastCCD(portName, maxBuffers, maxMemory, installPath, priority, stackSize);
  return(asynSuccess);
}


// 
// IOC shell configuration command for cin power up
//  
int FCCD_cin_power_up(const char *strParam)
{
   printf("cin_power_up: %s\n", strParam);
   //cin_power_up(); // defined in cin_power.c
   return (0);
}

// 
// IOC shell configuration command for cin power down
//  
int FCCD_cin_power_down(const char *strParam)
{
   //cin_power_down(); // defined in cin_power.c
   return (0);
}

/* Code for iocsh registration */

/* FCCDConfig */
static const iocshArg FCCDConfigArg0 = {"Port name", iocshArgString};
static const iocshArg FCCDConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg FCCDConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg FCCDConfigArg3 = {"installPath", iocshArgString};
static const iocshArg FCCDConfigArg4 = {"priority", iocshArgInt};
static const iocshArg FCCDConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const FCCDConfigArgs[] =  {&FCCDConfigArg0,
                                                       &FCCDConfigArg1,
                                                       &FCCDConfigArg2,
                                                       &FCCDConfigArg3,
                                                       &FCCDConfigArg4,
                                                       &FCCDConfigArg5};

static const iocshFuncDef configFastCCD = {"FCCDConfig", 6, FCCDConfigArgs};
static void configFastCCDCallFunc(const iocshArgBuf *args)
{
    FCCDConfig(args[0].sval, args[1].ival, args[2].ival, args[3].sval, 
                   args[4].ival, args[5].ival);
}

static void fCCDRegister(void)
{

    iocshRegister(&configFastCCD, configFastCCDCallFunc);
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

