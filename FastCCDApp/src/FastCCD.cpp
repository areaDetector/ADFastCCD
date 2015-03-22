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

asynStatus FastCCD::connect(asynUser *pasynUser){
  return connectCamera();
}
  
asynStatus FastCCD::connectCamera(){

  if(cin_data_init_port(&cin_data_port, NULL, 0, (char *)cinFabricIP, 0, 500)) {
    return asynError;
  }
  if(cin_ctl_init_port(&cin_ctl_port, NULL, 0, 0)) {
    return asynError;
  }
  if(cin_ctl_init_port(&cin_ctl_port_stream, NULL, 49202, 50202)) {
    return asynError;
  }
  if(cin_data_init(CIN_DATA_MODE_CALLBACK, cinPacketBuffer, cinImageBuffer,
                   allocateImageC, processImageC, this)) {
    return asynError;
  }

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

static void allocateImageC(cin_data_frame_t *frame){
  FastCCD *ptr = (FastCCD*)frame->usr_ptr;
  ptr->allocateImage(frame);
}

void FastCCD::allocateImage(cin_data_frame_t *frame)
{
  size_t dims[2];
  int nDims = 2;
   
  dims[0] = CIN_DATA_MAX_FRAME_X;
  dims[1] = CIN_DATA_MAX_FRAME_Y;
  NDDataType_t dataType = NDUInt16; 
  
  while(!(pImage = this->pNDArrayPool->alloc(nDims, dims, dataType, 
                                             0, NULL))) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "Unable to allocate array from pool....\n");
    sleep(1);
  }
      
  frame->data = (uint16_t*)pImage->pData;

  return;
}

static void processImageC(cin_data_frame_t *frame)
{
  FastCCD *ptr = (FastCCD*)frame->usr_ptr;
  ptr->processImage(frame);
}

void FastCCD::processImage(cin_data_frame_t *frame)
{
  const char* functionName = "processImage";
  this->lock();

  // Set the unique ID
  pImage->uniqueId = frame->number;
  pImage->dims[0].size = frame->size_x;
  pImage->dims[0].offset = 0;
  pImage->dims[0].binning = 1;
  pImage->dims[1].size = frame->size_y;
  pImage->dims[1].offset = 0;
  pImage->dims[1].binning = 1;

  // Process Timestamps.
  
  // Frame timestamp is a timespec always trust driver becuase it is correct!
  pImage->timeStamp = frame->timestamp.tv_sec + 1e-9 * frame->timestamp.tv_nsec;
  pImage->epicsTS.secPastEpoch = frame->timestamp.tv_sec;
  pImage->epicsTS.nsec = frame->timestamp.tv_nsec;
  updateTimeStamp(&pImage->epicsTS);
  
  // Get any attributes for the driver
  this->getAttributes(pImage->pAttributeList);
       
  int arrayCallbacks;
  getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

  if (arrayCallbacks) {
    /* Call the NDArray callback */
    /* Must release the lock here, or we can get into a deadlock, because we can
     * block on the plugin lock, and the plugin can be calling us */
    this->unlock();
    doCallbacksGenericPointer(pImage, NDArrayData, 0);
    this->lock();
  }

  if (this->framesRemaining > 0) this->framesRemaining--;
  if (this->framesRemaining == 0) {
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusIdle);
  }

  /* Update the frame counter */
  int imageCounter;
  getIntegerParam(NDArrayCounter, &imageCounter);
  imageCounter++;
  setIntegerParam(NDArrayCounter, imageCounter);
 
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "%s:%s: frameId=%d\n",
              driverName, functionName, frame->number);

  // Release the frame as we are done with it!
  pImage->release();

  /* Update any changed parameters */
  callParamCallbacks();

  this->unlock();
  return;
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
                 int priority, int stackSize, int packetBuffer, int imageBuffer,
		         const char *baseIP, const char *fabricIP, const char *fabricMAC)

  : ADDriver(portName, 1, NUM_FastCCD_DET_PARAMS, maxBuffers, maxMemory, 
             asynUInt32DigitalMask, asynUInt32DigitalMask,
             ASYN_CANBLOCK, 1, priority, stackSize)
{

  int status = asynSuccess;
  int sizeX, sizeY;
  
  static const char *functionName = "FastCCD";

  /* Write the packet and frame buffer sizes */
  cinPacketBuffer = packetBuffer;
  cinImageBuffer = imageBuffer;

  /* Store the network information */

  strncpy(cinBaseIP, baseIP, 20);
  strncpy(cinFabricIP, fabricIP, 20);
  strncpy(cinFabricMAC, fabricMAC, 20);

  //Define the polling periods for the status thread.
  statusPollingPeriod = 10; //seconds
  dataStatsPollingPeriod = 0.2; //seconds
  
  /* Create an EPICS exit handler */
  epicsAtExit(exitHandler, this);

  createParam(FastCCDMux1String,                asynParamInt32,    &FastCCDMux1);
  createParam(FastCCDMux2String,                asynParamInt32,    &FastCCDMux2);

  createParam(FastCCDFirmwarePathString,        asynParamOctet,    &FastCCDFirmwarePath);
  createParam(FastCCDBiasPathString,            asynParamOctet,    &FastCCDBiasPath);
  createParam(FastCCDClockPathString,           asynParamOctet,    &FastCCDClockPath);

  createParam(FastCCDFirmwareUploadString,      asynParamInt32,    &FastCCDFirmwareUpload);
  createParam(FastCCDBiasUploadString,          asynParamInt32,    &FastCCDBiasUpload);
  createParam(FastCCDClockUploadString,         asynParamInt32,    &FastCCDClockUpload);

  createParam(FastCCDPowerString,               asynParamInt32,    &FastCCDPower);
  createParam(FastCCDFPPowerString,             asynParamInt32,    &FastCCDFPPower);

  createParam(FastCCDFPGAStatusString,          asynParamUInt32Digital, &FastCCDFPGAStatus);
  createParam(FastCCDDCMStatusString,           asynParamUInt32Digital, &FastCCDDCMStatus);

  createParam(FastCCDBiasString,                asynParamInt32,    &FastCCDBias);
  createParam(FastCCDClockString,               asynParamInt32,    &FastCCDClock);

  createParam(FastCCDOverscanString,            asynParamInt32,    &FastCCDOverscan);

  createParam(FastCCDFclkString,                asynParamInt32,    &FastCCDFclk);

  createParam(FastCCDVBus12V0String,            asynParamFloat64,  &FastCCDVBus12V0);
  createParam(FastCCDVMgmt3v3String,            asynParamFloat64,  &FastCCDVMgmt3v3);
  createParam(FastCCDVMgmt2v5String,            asynParamFloat64,  &FastCCDVMgmt2v5);
  createParam(FastCCDVMgmt1v2String,            asynParamFloat64,  &FastCCDVMgmt1v2);
  createParam(FastCCDVEnet1v0String,            asynParamFloat64,  &FastCCDVEnet1v0);
  createParam(FastCCDVS3E3v3String,             asynParamFloat64,  &FastCCDVS3E3v3);
  createParam(FastCCDVGen3v3String,             asynParamFloat64,  &FastCCDVGen3v3);
  createParam(FastCCDVGen2v5String,             asynParamFloat64,  &FastCCDVGen2v5);
  createParam(FastCCDV60v9String,               asynParamFloat64,  &FastCCDV60v9);
  createParam(FastCCDV61v0String,               asynParamFloat64,  &FastCCDV61v0);
  createParam(FastCCDV62v5String,               asynParamFloat64,  &FastCCDV62v5);
  createParam(FastCCDVFpString,                 asynParamFloat64,  &FastCCDVFp);

  createParam(FastCCDIBus12V0String,            asynParamFloat64,  &FastCCDIBus12V0);
  createParam(FastCCDIMgmt3v3String,            asynParamFloat64,  &FastCCDIMgmt3v3);
  createParam(FastCCDIMgmt2v5String,            asynParamFloat64,  &FastCCDIMgmt2v5);
  createParam(FastCCDIMgmt1v2String,            asynParamFloat64,  &FastCCDIMgmt1v2);
  createParam(FastCCDIEnet1v0String,            asynParamFloat64,  &FastCCDIEnet1v0);
  createParam(FastCCDIS3E3v3String,             asynParamFloat64,  &FastCCDIS3E3v3);
  createParam(FastCCDIGen3v3String,             asynParamFloat64,  &FastCCDIGen3v3);
  createParam(FastCCDIGen2v5String,             asynParamFloat64,  &FastCCDIGen2v5);
  createParam(FastCCDI60v9String,               asynParamFloat64,  &FastCCDI60v9);
  createParam(FastCCDI61v0String,               asynParamFloat64,  &FastCCDI61v0);
  createParam(FastCCDI62v5String,               asynParamFloat64,  &FastCCDI62v5);
  createParam(FastCCDIFpString,                 asynParamFloat64,  &FastCCDIFp);

  createParam(FastCCDLibCinVersionString,       asynParamOctet,    &FastCCDLibCinVersion);
  createParam(FastCCDBoardIDString,             asynParamInt32,    &FastCCDBoardID);
  createParam(FastCCDSerialNumString,           asynParamInt32,    &FastCCDSerialNum);
  createParam(FastCCDFPGAVersionString,         asynParamInt32,    &FastCCDFPGAVersion);

  createParam(FastCCDStatusHBString,            asynParamInt32,    &FastCCDStatusHB);

  createParam(FastCCDBadPckString,              asynParamInt32,    &FastCCDBadPck);
  createParam(FastCCDDroppedPckString,          asynParamInt32,    &FastCCDDroppedPck);
  createParam(FastCCDLastFrameString,           asynParamInt32,    &FastCCDLastFrame);
  createParam(FastCCDResetStatsString,          asynParamInt32,    &FastCCDResetStats);

  // Create the epicsEvent for signaling to the status task when parameters should have changed.
  // This will cause it to do a poll immediately, rather than wait for the poll time period.
  this->statusEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->statusEvent) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Failed to create event for status task.\n",
              driverName, functionName);
    return;
  }

  this->dataStatsEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->dataStatsEvent) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Failed to create event for data stats task.\n",
              driverName, functionName);
    return;
  }

  try {
    this->lock();
    connectCamera();
    this->unlock();
    setStringParam(ADStatusMessage, "Initialized");
    callParamCallbacks();
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: %s\n",
      driverName, functionName, e.c_str());
    return;
  }

  sizeX = CIN_DATA_MAX_FRAME_X;
  sizeY = CIN_DATA_MAX_FRAME_Y;

  /* Set some default values for parameters */
  status =  setStringParam(ADManufacturer, "Berkeley Laboratory");
  status |= setStringParam(ADModel, "1k x 2k FastCCD");
  status |= setIntegerParam(ADSizeX, sizeX);
  status |= setIntegerParam(ADSizeY, sizeY);
  status |= setIntegerParam(ADBinX, 1);
  status |= setIntegerParam(ADBinY, 1);
  status |= setIntegerParam(ADMinX, 0);
  status |= setIntegerParam(ADMinY, 0);
  status |= setIntegerParam(ADMaxSizeX, sizeX);
  status |= setIntegerParam(ADMaxSizeY, sizeY);  
  status |= setIntegerParam(ADReverseX, 0);  
  status |= setIntegerParam(ADReverseY, 0);  
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, 1);
  status |= setDoubleParam(ADAcquireTime, 0.005);
  status |= setDoubleParam(ADAcquirePeriod, 1.0);
  status |= setIntegerParam(ADNumImages, 1);
  status |= setIntegerParam(ADNumExposures, 1);
  status |= setIntegerParam(NDArraySizeX, sizeX);
  status |= setIntegerParam(NDArraySizeY, sizeY);
  status |= setIntegerParam(NDDataType, NDUInt16);
  status |= setIntegerParam(NDArraySize, sizeX*sizeY*sizeof(epicsUInt16)); 
  status |= setDoubleParam(ADShutterOpenDelay, 0.);
  status |= setDoubleParam(ADShutterCloseDelay, 0.);

  status |= setIntegerParam(FastCCDFirmwareUpload, 0);
  status |= setIntegerParam(FastCCDClockUpload, 0);
  status |= setIntegerParam(FastCCDBiasUpload, 0);

  status |= setIntegerParam(FastCCDPower, 0);
  status |= setIntegerParam(FastCCDFPPower, 0);
  status |= setUIntDigitalParam(FastCCDFPGAStatus, 0x0, 0xFFFF);
  status |= setUIntDigitalParam(FastCCDDCMStatus, 0x0, 0xFFFF);
  status |= setIntegerParam(FastCCDBias, 0);
  status |= setIntegerParam(FastCCDClock, 0);

  status |= setIntegerParam(FastCCDMux1, 0);
  status |= setIntegerParam(FastCCDMux2, 0);

  status |= setStringParam(FastCCDFirmwarePath, "");
  status |= setStringParam(FastCCDBiasPath, "");
  status |= setStringParam(FastCCDClockPath, "");

  status |= setIntegerParam(FastCCDOverscan, 2);

  status |= setIntegerParam(FastCCDFclk, 0);

  status |= setDoubleParam(FastCCDVBus12V0, 0);
  status |= setDoubleParam(FastCCDVMgmt3v3, 0);
  status |= setDoubleParam(FastCCDVMgmt2v5, 0);
  status |= setDoubleParam(FastCCDVMgmt1v2, 0);
  status |= setDoubleParam(FastCCDVEnet1v0, 0);
  status |= setDoubleParam(FastCCDVS3E3v3, 0);
  status |= setDoubleParam(FastCCDVGen3v3, 0);
  status |= setDoubleParam(FastCCDVGen2v5, 0);
  status |= setDoubleParam(FastCCDV60v9, 0);
  status |= setDoubleParam(FastCCDV61v0, 0);
  status |= setDoubleParam(FastCCDV62v5, 0);
  status |= setDoubleParam(FastCCDVFp, 0);

  status |= setDoubleParam(FastCCDIBus12V0, 0);
  status |= setDoubleParam(FastCCDIMgmt3v3, 0);
  status |= setDoubleParam(FastCCDIMgmt2v5, 0);
  status |= setDoubleParam(FastCCDIMgmt1v2, 0);
  status |= setDoubleParam(FastCCDIEnet1v0, 0);
  status |= setDoubleParam(FastCCDIS3E3v3, 0);
  status |= setDoubleParam(FastCCDIGen3v3, 0);
  status |= setDoubleParam(FastCCDIGen2v5, 0);
  status |= setDoubleParam(FastCCDI60v9, 0);
  status |= setDoubleParam(FastCCDI61v0, 0);
  status |= setDoubleParam(FastCCDI62v5, 0);
  status |= setDoubleParam(FastCCDIFp, 0);

  status |= setStringParam(FastCCDLibCinVersion, (char *)cin_build_version);

  callParamCallbacks();

  // Signal the status thread to poll the detector
  epicsEventSignal(statusEvent);
  epicsEventSignal(dataStatsEvent);
  
  if (stackSize == 0) {
    stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
  }

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("FastCCDStatusTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)FastCCDStatusTaskC,
                              this) == NULL);
  if(status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to create status task.\n",
      driverName, functionName);
    return;
  }

  /* Create the thread that updates the data stats */
  status = (epicsThreadCreate("FastCCDDataStatsTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)FastCCDDataStatsTaskC,
                              this) == NULL);
  if(status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Failed to create data stats task.\n",
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
    this->lock();
    cin_data_stop_threads();
    cin_data_wait_for_threads();
    this->unlock();
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


// asynStatus FastCCD::readEnum(asynUser *pasynUser, char *strings[], int values[], 
//                              int severities[], size_t nElements, size_t *nIn)
// {
//   int function = pasynUser->reason;
// 
//   return ADDriver::readEnum(pasynUser, strings, values, severities,nElements, nIn );
//   
// }

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

asynStatus FastCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value){
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    static const char *functionName = "writeFloat64";

    getParamName(function, &paramName);

    status = setDoubleParam(function, value);

    if(function == ADAcquireTime){
      if(cin_ctl_set_exposure_time(&cin_ctl_port, (float)value)){
        status = asynError;
      }
    } else if (function == ADAcquirePeriod) {
      if(cin_ctl_set_cycle_time(&cin_ctl_port, (float)value)){
        status = asynError;
      }
    }

    if(status){
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
    }
    return status;

}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus FastCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;

    asynStatus status = asynSuccess;
    static const char *functionName = "writeInt32";

    //Set in param lib so the user sees a readback straight away. Save a backup in case of errors.
    setIntegerParam(function, value);

    int _status = 0; // For return of cin functions

    if (function == ADAcquire) 
    {
      if (value) // User clicked 'Start' button
      {
         // Send the hardware a start trigger command
         int n_images, t_mode, i_mode;

         getIntegerParam(ADTriggerMode, &t_mode);
         getIntegerParam(ADImageMode, &i_mode);
         getIntegerParam(ADNumImages, &n_images);

         switch(i_mode) {
           case ADImageSingle:
             this->framesRemaining = 1;
             n_images = 1;
             break;
           case ADImageMultiple:
             this->framesRemaining = n_images;
             break;
           case ADImageContinuous:
             this->framesRemaining = -1;
             n_images = 0;
             break;
         }

         if(t_mode == 0){
           if(!cin_ctl_int_trigger_start(&cin_ctl_port, n_images)){
             setIntegerParam(ADAcquire, 1);
             setIntegerParam(ADStatus, ADStatusAcquire);
           }
         } else {
           if(!cin_ctl_ext_trigger_start(&cin_ctl_port, t_mode)){
             setIntegerParam(ADStatus, ADStatusAcquire);
             setIntegerParam(ADAcquire, 1);
           }
         }
      }
      else // User clicked 'Stop' Button
      {
         // Send the hardware a stop trigger command
         if(!cin_ctl_int_trigger_stop(&cin_ctl_port)){
           setIntegerParam(ADStatus, ADStatusIdle);
           setIntegerParam(ADAcquire, 0);
         }
         if(!cin_ctl_ext_trigger_stop(&cin_ctl_port)){
           setIntegerParam(ADStatus, ADStatusIdle);
           setIntegerParam(ADAcquire, 0);
         }
      }
    }
    else if (function == FastCCDFirmwareUpload)
    {
      char path[256];
      getStringParam(FastCCDFirmwarePath, sizeof(path), path);
      setIntegerParam(FastCCDFirmwareUpload, 1);

      // Power off the cin
	  
      setStringParam(ADStatusMessage, "Powering CIN OFF");
      callParamCallbacks();
      _status = cin_ctl_pwr(&cin_ctl_port, 0);
      sleep(2);
	  
      // Power on the cin
	  
      setStringParam(ADStatusMessage, "Powering CIN ON");
      callParamCallbacks();
      _status = cin_ctl_pwr(&cin_ctl_port, 1);
      sleep(2);

      setStringParam(ADStatusMessage, "Uploading Firmware");
      callParamCallbacks();
      _status = cin_ctl_load_firmware(&cin_ctl_port, 
                                      &cin_ctl_port_stream, path);
     
      if(!_status){
        _status |= cin_ctl_set_fabric_address(&cin_ctl_port, (char *)cinFabricIP);
        _status |= cin_data_send_magic();
      }
      setIntegerParam(FastCCDFirmwareUpload, 0);

      char msg[256];
      sprintf(msg, "Set Fabric Ip to %s", cinFabricIP);
      setStringParam(ADStatusMessage, msg);
      callParamCallbacks();
    }
    else if (function == FastCCDClockUpload)
    {
      char path[256];
      getStringParam(FastCCDClockPath, sizeof(path), path);
      setIntegerParam(FastCCDClockUpload, 1);
      setStringParam(ADStatusMessage, "Uploading Clock File");
      callParamCallbacks();

      _status = cin_ctl_load_config(&cin_ctl_port, path);

      setIntegerParam(FastCCDClockUpload, 0);
      setStringParam(ADStatusMessage, "Done");
      callParamCallbacks();
    }
    else if (function == FastCCDBiasUpload)
    {
      char path[256];
      getStringParam(FastCCDBiasPath, sizeof(path), path);
      setIntegerParam(FastCCDBiasUpload, 1);
      setStringParam(ADStatusMessage, "Uploading Bias File");
      callParamCallbacks();

      _status = cin_ctl_load_config(&cin_ctl_port, path);

      setIntegerParam(FastCCDBiasUpload, 0);
      setStringParam(ADStatusMessage, "Done");
      callParamCallbacks();
    }
    else if (function == FastCCDPower)
    {
      if(value) {
        _status = cin_ctl_pwr(&cin_ctl_port, 1);
      } else {
        _status = cin_ctl_pwr(&cin_ctl_port, 0);
      }
    }
    else if (function == FastCCDFPPower)
    {
      if(value) {
        _status = cin_ctl_fp_pwr(&cin_ctl_port, 1);
      } else {
        _status = cin_ctl_fp_pwr(&cin_ctl_port, 0);
      }
    }
    else if (function == FastCCDBias)
    {
      if(value){
        _status = cin_ctl_set_bias(&cin_ctl_port, 1);
      } else {
        _status = cin_ctl_set_bias(&cin_ctl_port, 0);
      }
    }
    else if (function == FastCCDClock)
    {
      if(value){
        _status = cin_ctl_set_clocks(&cin_ctl_port, 1);
      } else {
        _status = cin_ctl_set_clocks(&cin_ctl_port, 0);
      }
    }
    else if ((function == ADNumExposures) || (function == ADNumImages)   ||
             (function == ADImageMode))
    {
      status = ADDriver::writeInt32(pasynUser, value);
    }
    else if ((function == FastCCDMux1) || (function == FastCCDMux2))
    {
      int _val, _val1, _val2;
      getIntegerParam(FastCCDMux1, &_val1);
      getIntegerParam(FastCCDMux2, &_val2);
      _val = (_val2 << 4) | _val1;
      _status = cin_ctl_set_mux(&cin_ctl_port, _val);   
    }
    else if (function == FastCCDResetStats)
    {
      cin_data_reset_stats();
    }
    else if ((function == ADSizeY) || (function == FastCCDOverscan))
    {
      // The Y size changed, change the descramble routine
      int _val1, _val2;
      getIntegerParam(ADSizeY, &_val1);
      getIntegerParam(FastCCDOverscan, &_val2);
      _status = cin_data_set_descramble_params(_val1, _val2);

      // Read back to check all OK

      int _x, _y;
      cin_data_get_descramble_params(&_val1, &_val2, &_x, &_y);
      setIntegerParam(ADSizeX, _x);
      setIntegerParam(ADSizeY, _y);

    }
    else if (function == FastCCDFclk)
    {
      _status = cin_ctl_set_fclk(&cin_ctl_port, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(_status){
      status = asynError;
    }

    if (status)
    {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    }
    return status;
}

void FastCCD::dataStatsTask(void)
{
  unsigned int status = 0;
  double timeout = 0.0;
  static const char *functionName = "dataStatsTask";

  while(1) {

    //Read timeout for polling freq.
    this->lock();
    timeout = dataStatsPollingPeriod;
    this->unlock();

    if (timeout != 0.0) {
      status = epicsEventWaitWithTimeout(dataStatsEvent, timeout);
    } else {
      status = epicsEventWait(dataStatsEvent);
    }              
  
    if (status == epicsEventWaitOK) {
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Got status event\n",
        driverName, functionName);
    }

    cin_data_stats_t stats;
    cin_data_compute_stats(&stats);
    setIntegerParam(FastCCDDroppedPck, (int)stats.dropped_packets);
    setIntegerParam(FastCCDBadPck, (int)stats.mallformed_packets);
    setIntegerParam(FastCCDLastFrame, stats.last_frame);
    //setDoubleParam(FastCCDDataRate, stats.datarate);
    
    this->lock();
    callParamCallbacks();
    this->unlock();
  }
}

/**
 * Update status of detector. Meant to be run in own thread.
 */
void FastCCD::statusTask(void)
{
  unsigned int status = 0;
  double timeout = 0.0;
  static const char *functionName = "statusTask";

  cin_ctl_pwr_mon_t pwr_value;
  int pwr;
  int cin_status;
  int ticktock = 0;
  int run_once = 0;

  while(1) {

    //Read timeout for polling freq.
    this->lock();
    timeout = statusPollingPeriod;
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

    // Update the ticktock
    
    ticktock += 1;
    setIntegerParam(FastCCDStatusHB, ticktock);

    // Run once certain reads
    
    if(!run_once){
      cin_ctl_id_t id;
      cin_status = 0;
      cin_status |= cin_ctl_get_id(&cin_ctl_port, &id);
      if(!cin_status){
        setIntegerParam(FastCCDBoardID, id.board_id);
        setIntegerParam(FastCCDSerialNum, id.serial_no);
        setIntegerParam(FastCCDFPGAVersion, id.fpga_ver);
        setParamStatus(FastCCDBoardID, asynSuccess);
        setParamStatus(FastCCDSerialNum, asynSuccess);
        setParamStatus(FastCCDFPGAVersion, asynSuccess);
      }

      // If sucsessful, don't run again

      if(!cin_status){
        run_once = 1;
      } else {
        setParamStatus(FastCCDBoardID, asynDisconnected);
        setParamStatus(FastCCDSerialNum, asynDisconnected);
        setParamStatus(FastCCDFPGAVersion, asynDisconnected);
      }
    }

    cin_status = cin_ctl_get_power_status(&cin_ctl_port, &pwr, &pwr_value);
    if(!cin_status){
      // Power Status
      if(pwr){
        setIntegerParam(FastCCDPower, 1);
        if(pwr == 2){
          setIntegerParam(FastCCDFPPower, 1);
        } else {
          setIntegerParam(FastCCDFPPower, 0);
        }
       
        // Voltage Values

        setDoubleParam(FastCCDVBus12V0, pwr_value.bus_12v0.v);
        setDoubleParam(FastCCDVMgmt3v3, pwr_value.mgmt_3v3.v);
        setDoubleParam(FastCCDVMgmt2v5, pwr_value.mgmt_2v5.v);
        setDoubleParam(FastCCDVMgmt1v2, pwr_value.mgmt_1v2.v);
        setDoubleParam(FastCCDVEnet1v0, pwr_value.enet_1v0.v);
        setDoubleParam(FastCCDVS3E3v3,  pwr_value.s3e_3v3.v);
        setDoubleParam(FastCCDVGen3v3,  pwr_value.gen_3v3.v);
        setDoubleParam(FastCCDVGen2v5,  pwr_value.gen_2v5.v);
        setDoubleParam(FastCCDV60v9,    pwr_value.v6_0v9.v);
        setDoubleParam(FastCCDV61v0,    pwr_value.v6_1v0.v);
        setDoubleParam(FastCCDV62v5,    pwr_value.v6_2v5.v);
        setDoubleParam(FastCCDVFp,      pwr_value.fp.v);
        setParamStatus(FastCCDVBus12V0, asynSuccess);
        setParamStatus(FastCCDVMgmt3v3, asynSuccess);
        setParamStatus(FastCCDVMgmt2v5, asynSuccess);
        setParamStatus(FastCCDVMgmt1v2, asynSuccess);
        setParamStatus(FastCCDVEnet1v0, asynSuccess);
        setParamStatus(FastCCDVS3E3v3,  asynSuccess);
        setParamStatus(FastCCDVGen3v3,  asynSuccess);
        setParamStatus(FastCCDVGen2v5,  asynSuccess);
        setParamStatus(FastCCDV60v9,    asynSuccess);
        setParamStatus(FastCCDV61v0,    asynSuccess);
        setParamStatus(FastCCDV62v5,    asynSuccess);
        setParamStatus(FastCCDVFp,      asynSuccess);

        // Current Values

        setDoubleParam(FastCCDIBus12V0, pwr_value.bus_12v0.i);
        setDoubleParam(FastCCDIMgmt3v3, pwr_value.mgmt_3v3.i);
        setDoubleParam(FastCCDIMgmt2v5, pwr_value.mgmt_2v5.i);
        setDoubleParam(FastCCDIMgmt1v2, pwr_value.mgmt_1v2.i);
        setDoubleParam(FastCCDIEnet1v0, pwr_value.enet_1v0.i);
        setDoubleParam(FastCCDIS3E3v3,  pwr_value.s3e_3v3.i);
        setDoubleParam(FastCCDIGen3v3,  pwr_value.gen_3v3.i);
        setDoubleParam(FastCCDIGen2v5,  pwr_value.gen_2v5.i);
        setDoubleParam(FastCCDI60v9,    pwr_value.v6_0v9.i);
        setDoubleParam(FastCCDI61v0,    pwr_value.v6_1v0.i);
        setDoubleParam(FastCCDI62v5,    pwr_value.v6_2v5.i);
        setDoubleParam(FastCCDIFp,      pwr_value.fp.i);
        setParamStatus(FastCCDIBus12V0, asynSuccess);
        setParamStatus(FastCCDIMgmt3v3, asynSuccess);
        setParamStatus(FastCCDIMgmt2v5, asynSuccess);
        setParamStatus(FastCCDIMgmt1v2, asynSuccess);
        setParamStatus(FastCCDIEnet1v0, asynSuccess);
        setParamStatus(FastCCDIS3E3v3,  asynSuccess);
        setParamStatus(FastCCDIGen3v3,  asynSuccess);
        setParamStatus(FastCCDIGen2v5,  asynSuccess);
        setParamStatus(FastCCDI60v9,    asynSuccess);
        setParamStatus(FastCCDI61v0,    asynSuccess);
        setParamStatus(FastCCDI62v5,    asynSuccess);
        setParamStatus(FastCCDIFp,      asynSuccess);

      } else {
        setIntegerParam(FastCCDPower, 0);
        setIntegerParam(FastCCDFPPower, 0);
      }

      setParamStatus(FastCCDPower, asynSuccess);
      setParamStatus(FastCCDFPPower, asynSuccess);
    }

    if(cin_status || !pwr){
      // Voltage Values

      setParamStatus(FastCCDVBus12V0, asynDisconnected);
      setParamStatus(FastCCDVMgmt3v3, asynDisconnected);
      setParamStatus(FastCCDVMgmt2v5, asynDisconnected);
      setParamStatus(FastCCDVMgmt1v2, asynDisconnected);
      setParamStatus(FastCCDVEnet1v0, asynDisconnected);
      setParamStatus(FastCCDVS3E3v3,  asynDisconnected);
      setParamStatus(FastCCDVGen3v3,  asynDisconnected);
      setParamStatus(FastCCDVGen2v5,  asynDisconnected);
      setParamStatus(FastCCDV60v9,    asynDisconnected);
      setParamStatus(FastCCDV61v0,    asynDisconnected);
      setParamStatus(FastCCDV62v5,    asynDisconnected);
      setParamStatus(FastCCDVFp,      asynDisconnected);

      // Current Values

      setParamStatus(FastCCDIBus12V0, asynDisconnected);
      setParamStatus(FastCCDIMgmt3v3, asynDisconnected);
      setParamStatus(FastCCDIMgmt2v5, asynDisconnected);
      setParamStatus(FastCCDIMgmt1v2, asynDisconnected);
      setParamStatus(FastCCDIEnet1v0, asynDisconnected);
      setParamStatus(FastCCDIS3E3v3,  asynDisconnected);
      setParamStatus(FastCCDIGen3v3,  asynDisconnected);
      setParamStatus(FastCCDIGen2v5,  asynDisconnected);
      setParamStatus(FastCCDI60v9,    asynDisconnected);
      setParamStatus(FastCCDI61v0,    asynDisconnected);
      setParamStatus(FastCCDI62v5,    asynDisconnected);
      setParamStatus(FastCCDIFp,      asynDisconnected);
    }

    // Status
	
    uint16_t fpga_status, dcm_status;
    
	  cin_status = cin_ctl_get_cfg_fpga_status(&cin_ctl_port, &fpga_status);
    if(!cin_status){
      setUIntDigitalParam(FastCCDFPGAStatus, fpga_status, 0xFFFF);
      setParamStatus(FastCCDFPGAStatus, asynSuccess);
    } else {
      setParamStatus(FastCCDFPGAStatus, asynDisconnected);
    }
    
	  cin_status = cin_ctl_get_dcm_status(&cin_ctl_port, &dcm_status);
      if(!cin_status){
	    setUIntDigitalParam(FastCCDDCMStatus, dcm_status, 0xFFFF);
      setParamStatus(FastCCDDCMStatus, asynSuccess);
	  } else {
      setParamStatus(FastCCDDCMStatus, asynDisconnected);
    }

    /* Are we powered up and configured? */

    if(fpga_status & CIN_CTL_FPGA_STS_CFG){

      // Clock and Bias status
    
      int bias, clock;
      cin_status = cin_ctl_get_bias(&cin_ctl_port, &bias);
      if(!cin_status){
        setIntegerParam(FastCCDBias, bias);
        setParamStatus(FastCCDBias, asynSuccess);
      } else {
        setParamStatus(FastCCDBias, asynDisconnected);
      }
   
      cin_status = cin_ctl_get_clocks(&cin_ctl_port, &clock);
      if(!cin_status){
        setIntegerParam(FastCCDClock, clock);
        setParamStatus(FastCCDClock, asynSuccess);
      } else {
        setParamStatus(FastCCDClock, asynDisconnected);
      }

      // Get Mux Settings
      
      int mux;
      cin_status = cin_ctl_get_mux(&cin_ctl_port, &mux);
      if(!cin_status){
        setIntegerParam(FastCCDMux1, (mux & 0x000F));
        setIntegerParam(FastCCDMux2, (mux & 0x00F0) >> 4);
        setParamStatus(FastCCDMux1, asynSuccess);
        setParamStatus(FastCCDMux2, asynSuccess);
      } else {
        setParamStatus(FastCCDMux1, asynDisconnected);
        setParamStatus(FastCCDMux2, asynDisconnected);
      }


      // Get FCLK Settings

      int fclk;
      cin_status = cin_ctl_get_fclk(&cin_ctl_port, &fclk);
      if(!cin_status){
        switch(fclk){
          case CIN_CTL_FCLK_125:
            setIntegerParam(FastCCDFclk, 0);
            break;
          case CIN_CTL_FCLK_200:
            setIntegerParam(FastCCDFclk, 1);
            break;
          case CIN_CTL_FCLK_250:
            setIntegerParam(FastCCDFclk, 2);
            break;
        }

        setParamStatus(FastCCDFclk, asynSuccess);

      } else {
        setParamStatus(FastCCDFclk, asynDisconnected);
      }


    } else {
      // Set the Comm Error 
      
      setParamStatus(FastCCDBias, asynDisconnected);
      setParamStatus(FastCCDClock, asynDisconnected);
      setParamStatus(FastCCDMux1, asynDisconnected);
      setParamStatus(FastCCDMux2, asynDisconnected);
      setParamStatus(FastCCDFclk, asynDisconnected);

    }

    /* Call the callbacks to update any changes */
    this->lock();
    callParamCallbacks();
    this->unlock();
        
  } //End of while loop

}

//C utility functions to tie in with EPICS

static void FastCCDStatusTaskC(void *drvPvt)
{
  FastCCD *pPvt = (FastCCD *)drvPvt;

  pPvt->statusTask();
}


static void FastCCDDataStatsTaskC(void *drvPvt)
{
  FastCCD *pPvt = (FastCCD *)drvPvt;

  pPvt->dataStatsTask();
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
                  int priority, int stackSize, int packetBuffer, int imageBuffer,
				  const char *baseIP, const char *fabricIP, const char *fabricMAC)
{
  new FastCCD(portName, maxBuffers, maxMemory, priority, stackSize, packetBuffer, imageBuffer,
		      baseIP, fabricIP, fabricMAC);
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
static const iocshArg FastCCDConfigArg7 = {"baseIP", iocshArgString};
static const iocshArg FastCCDConfigArg8 = {"fabricIP", iocshArgString};
static const iocshArg FastCCDConfigArg9 = {"fabricMAC", iocshArgString};
static const iocshArg * const FastCCDConfigArgs[] =  {&FastCCDConfigArg0,
                                                       &FastCCDConfigArg1,
                                                       &FastCCDConfigArg2,
                                                       &FastCCDConfigArg3,
                                                       &FastCCDConfigArg4,
                                                       &FastCCDConfigArg5,
                                                       &FastCCDConfigArg6,
                                                       &FastCCDConfigArg7,
                                                       &FastCCDConfigArg8,
                                                       &FastCCDConfigArg9};

static const iocshFuncDef configFastCCD = {"FastCCDConfig", 10, FastCCDConfigArgs};
static void configFastCCDCallFunc(const iocshArgBuf *args)
{
    FastCCDConfig(args[0].sval, args[1].ival, args[2].ival,
                  args[3].ival, args[4].ival, args[5].ival,
                  args[6].ival, args[7].sval, args[8].sval,
				  args[9].sval);
}

static void FastCCDRegister(void)
{
    iocshRegister(&configFastCCD, configFastCCDCallFunc);
}

epicsExportRegistrar(FastCCDRegister);

} // extern "C"

