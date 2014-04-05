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
#include "FastCCD.h"

static const char *driverName = "FastCCD";

//Definitions of static class data members

const epicsUInt32 FastCCD::ATInternal = 0;
const epicsUInt32 FastCCD::ATExternal1 = 1;
const epicsUInt32 FastCCD::ATExternal2 = 2;
const epicsUInt32 FastCCD::ATExternal1or2 = 3;

const epicsInt32 FastCCD::AShutterAuto = 0;
const epicsInt32 FastCCD::AShutterOpen = 1;
const epicsInt32 FastCCD::AShutterClose = 2;


//C Function prototypes to tie in with EPICS
static void FastCCDStatusTaskC(void *drvPvt);
static void FastCCDDataTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);

asynStatus FastCCD::connect(asynUser *pasynUser){
  return connectCamera();
}

asynStatus FastCCD::connectCamera(){

  int ret= 0;

  if(cin_data_init_port(&cin_data_port, NULL, 0, "10.23.5.127", 0, 1000))
    return asynError;

  if(cin_ctl_init_port(&cin_ctl_port, NULL, 0, 0))
    return asynError;
  if(cin_ctl_init_port(&cin_ctl_port_stream, NULL, 49202, 50202))
    return asynError;
   
  if( (ret = cin_data_init(CIN_DATA_MODE_PUSH_PULL, cinPacketBuffer, cinImageBuffer)))
    return asynError;
 
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
      return (0);
   }

   return (-1); // error
}

/** Constructor for FastCCD driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
FastCCD::FastCCD(const char *portName, int maxBuffers, size_t maxMemory, 
                 int priority, int stackSize, int packetBuffer, int imageBuffer)

  : ADDriver(portName, 1, NUM_FastCCD_DET_PARAMS, maxBuffers, maxMemory, 
             asynEnumMask, asynEnumMask,
             ASYN_CANBLOCK, 1, priority, stackSize)
{

  int status = asynSuccess;
  int sizeX, sizeY;
  
  static const char *functionName = "FastCCD";

  /* Write the packet and frame buffer sizes */
  cinPacketBuffer = packetBuffer;
  cinImageBuffer = imageBuffer;

  fprintf(stderr, "BUFFERS %d %d\n", cinPacketBuffer, cinImageBuffer);

  /* Create an EPICS exit handler */
  epicsAtExit(exitHandler, this);

  //createParam(FastCCDSetBiasString,                  asynParamInt32, &FastCCDSetBias);
  //createParam(FastCCDSetClocksString,                asynParamInt32, &FastCCDSetClocks);

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

  setStringParam(ADStatusMessage, "Defaults Set.");
  callParamCallbacks();

  /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
  epicsEventSignal(statusEvent);

  //Define the polling periods for the status thread.
  mPollingPeriod = 10.0; //seconds

  mAcquiringData = 0;
  
  if (stackSize == 0) stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("FastCCDStatusTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)FastCCDStatusTaskC,
                              this) == NULL);
  if(status) {
    printf("%s:%s: epicsThreadCreate failure for status task\n",
           driverName, functionName);
    return;
  }

  /* Create the thread that does data readout */
  status = (epicsThreadCreate("FastCCDDataTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)FastCCDDataTaskC,
                              this) == NULL);
  if (status) {
    printf("%s:%s: epicsThreadCreate failure for data task\n",
           driverName, functionName);
    return;
  }
}

/**
 * Destructor.  Free resources and closes the FastCCD library
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
  //int function = pasynUser->reason;

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
    // else if (function == FastCCDSetBias) {
    //      cin_ctl_set_bias(cin_ctl_port, value);
    // }
    // else if (function == FastCCDSetClocks) {
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


/**
 * Update status of detector. Meant to be run in own thread.
 */
void FastCCD::statusTask(void)
{
  // float temperature;
  // unsigned int uvalue = 0;
  unsigned int status = 0;
  double timeout = 0.0;
  static const char *functionName = "statusTask";

  printf("%s:%s: Status thread started...\n", driverName, functionName);
  while(1) {

    //Read timeout for polling freq.
    this->lock();
    timeout = mPollingPeriod;
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
    }

    this->lock();

    try {
      //Only read these if we are not acquiring data
      //if (!mAcquiringData) {
      //  
      //}

      int cin_status;
      cin_ctl_pwr_mon_t pwr_value;
      cin_ctl_fpga_status fpga_status;
      int pwr;
      cin_status  = cin_ctl_get_power_status(&cin_ctl_port, &pwr, &pwr_value);
      cin_status |= cin_ctl_get_cfg_fpga_status(&cin_ctl_port, &fpga_status);
      if(!cin_status){
        fprintf(stderr,"OK\n");
      }
    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
      setStringParam(ADStatusMessage, e.c_str());
    }

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
   //static const char *functionName = "setupAcquisition";

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
      //getIntegerParam(NDDataType, &itemp); dataType = (NDDataType_t)itemp;
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

static void FastCCDStatusTaskC(void *drvPvt)
{
  FastCCD *pPvt = (FastCCD *)drvPvt;

  pPvt->statusTask();
}


static void FastCCDDataTaskC(void *drvPvt)
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
  * \param[in] priority The thread priority for the asyn port driver thread
  * \param[in] stackSize The stack size for the asyn port driver thread
  * \param[in] packetBuffer The CINDATA packet buffer size
  * \param[in] imageBuffer The CINDATA image buffer size
  */
extern "C" {
int FastCCDConfig(const char *portName, int maxBuffers, size_t maxMemory, 
                  int priority, int stackSize, int packetBuffer, int imageBuffer)
{
  new FastCCD(portName, maxBuffers, maxMemory, priority, stackSize, packetBuffer, imageBuffer);
  return(asynSuccess);
}

/* Code for iocsh registration */

/* FastCCDConfig */
static const iocshArg FastCCDConfigArg0 = {"Port name", iocshArgString};
static const iocshArg FastCCDConfigArg1 = {"maxBuffers", iocshArgInt};
static const iocshArg FastCCDConfigArg2 = {"maxMemory", iocshArgInt};
static const iocshArg FastCCDConfigArg3 = {"priority", iocshArgInt};
static const iocshArg FastCCDConfigArg4 = {"stackSize", iocshArgInt};
static const iocshArg FastCCDConfigArg5 = {"packetBuffer", iocshArgInt};
static const iocshArg FastCCDConfigArg6 = {"imageBuffer", iocshArgInt};
static const iocshArg * const FastCCDConfigArgs[] =  {&FastCCDConfigArg0,
                                                       &FastCCDConfigArg1,
                                                       &FastCCDConfigArg2,
                                                       &FastCCDConfigArg3,
                                                       &FastCCDConfigArg4,
                                                       &FastCCDConfigArg5,
                                                       &FastCCDConfigArg6};

static const iocshFuncDef configFastCCD = {"FastCCDConfig", 7, FastCCDConfigArgs};
static void configFastCCDCallFunc(const iocshArgBuf *args)
{
    FastCCDConfig(args[0].sval, args[1].ival, args[2].ival,
                  args[3].ival, args[4].ival, args[5].ival,
                  args[6].ival);
}

static void FastCCDRegister(void)
{
    iocshRegister(&configFastCCD, configFastCCDCallFunc);
}

epicsExportRegistrar(FastCCDRegister);

} // extern "C"

