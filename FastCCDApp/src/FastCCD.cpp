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

  if(cin_data_init_port(&cin_data_port, NULL, 0, "10.23.5.127", 0, 1000))
  {
    return asynError;
  }
  if(cin_ctl_init_port(&cin_ctl_port, NULL, 0, 0))
  {
    return asynError;
  }
  if(cin_ctl_init_port(&cin_ctl_port_stream, NULL, 49202, 50202))
  {
    return asynError;
  }
  if(cin_data_init(CIN_DATA_MODE_CALLBACK, cinPacketBuffer, cinImageBuffer,
                   allocateImageC, processImageC, this))
  {
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
   
  dims[0] = CIN_DATA_FRAME_WIDTH;
  dims[1] = CIN_DATA_FRAME_HEIGHT;
  NDDataType_t dataType = NDUInt16; 
  
  while(!(pImage = this->pNDArrayPool->alloc(nDims, dims, dataType, 
                                        0, NULL)))
  {
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

  // TODO : Do timestamp processing using cin_data_frame
  
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
  // SBW Not sure we should do this!!!!!!
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

  //Define the polling periods for the status thread.
  mPollingPeriod = 5.0; //seconds
  
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

  createParam(FastCCDFrameIPAddrString,         asynParamOctet,    &FastCCDFrameIPAddr);
  createParam(FastCCDFrameMACAddrString,        asynParamOctet,    &FastCCDFrameMACAddr);
  createParam(FastCCDDataIPAddrString,          asynParamOctet,    &FastCCDDataIPAddr);
  createParam(FastCCDDataMACAddrString,         asynParamOctet,    &FastCCDDataMACAddr);

  createParam(FastCCDFPGAStatusString,          asynParamInt32,    &FastCCDFPGAStatus);
  createParam(FastCCDFPPowerStatusString,       asynParamInt32,    &FastCCDFPPowerStatus);

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

  // Create the epicsEvent for signaling to the status task when parameters should have changed.
  // This will cause it to do a poll immediately, rather than wait for the poll time period.
  this->statusEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->statusEvent) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Failed to create event for status task.\n",
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

  sizeX = CIN_DATA_FRAME_WIDTH;
  sizeY = CIN_DATA_FRAME_HEIGHT;

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
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, 1);
  status |= setDoubleParam (ADAcquireTime, 0.005);
  status |= setDoubleParam (ADAcquirePeriod, 1.0);
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

  //status |= setIntegerParam(FastCCDPower, 0);
  status |= setIntegerParam(FastCCDFPGAStatus, 0);
  status |= setIntegerParam(FastCCDFPPowerStatus, 0);

  status |= setIntegerParam(FastCCDMux1, 0);
  status |= setIntegerParam(FastCCDMux2, 0);

  status |= setStringParam(FastCCDFirmwarePath, "");
  status |= setStringParam(FastCCDBiasPath, "");
  status |= setStringParam(FastCCDClockPath, "");

  status |= setStringParam(FastCCDFrameIPAddr, "");
  status |= setStringParam(FastCCDFrameMACAddr, "");

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

  callParamCallbacks();

  // Signal the status thread to poll the detector
  epicsEventSignal(statusEvent);
  
  if (stackSize == 0)
  {
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

    int _status ; // For return of cin functions

    if (function == ADAcquire) 
    {
      if (value) // User clicked 'Start' button
      {
         // Send the hardware a start trigger command
         int n_images, t_mode, i_mode;
         double t_exp, t_cycle; 
         getIntegerParam(ADTriggerMode, &t_mode);
         getIntegerParam(ADNumImages, &n_images);
         getIntegerParam(ADImageMode, &i_mode);
         getDoubleParam(ADAcquireTime, &t_exp);
         getDoubleParam(ADAcquirePeriod, &t_cycle);

         cin_ctl_set_exposure_time(&cin_ctl_port, (float)t_exp);
         cin_ctl_set_cycle_time(&cin_ctl_port, (float)t_cycle);

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
      setStringParam(ADStatusMessage, "Uploading Firmware.");
      callParamCallbacks();

      _status = cin_ctl_load_firmware(&cin_ctl_port, 
                                      &cin_ctl_port_stream, path);
     
      if(!_status){
        char ip[256];
        getStringParam(FastCCDFrameIPAddr, sizeof(ip), ip);
        _status |= cin_ctl_set_fabric_address(&cin_ctl_port, ip);
        _status |= cin_data_send_magic();
      }
      setIntegerParam(FastCCDFirmwareUpload, 0);
      setStringParam(ADStatusMessage, "");
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
      setStringParam(ADStatusMessage, "");
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
      setStringParam(ADStatusMessage, "");
    }
    else if (function == FastCCDPower)
    {
      if(value)
      {
        _status = cin_ctl_pwr(&cin_ctl_port, 1);
        sleep(1);
        _status = cin_ctl_fp_pwr(&cin_ctl_port, 1);
      } else {
        _status = cin_ctl_pwr(&cin_ctl_port, 0);
      }
    }
    else if ((function == ADNumExposures) || (function == ADNumImages)   ||
             (function == ADImageMode))
    {
      status = ADDriver::writeInt32(pasynUser, value);
    }
    else if (function == FastCCDMux1)
    {
      int _val;
      getIntegerParam(FastCCDMux2, &_val);
      _val = (_val << 4) | value;
      cin_ctl_set_mux(&cin_ctl_port, value);   
    }
    else if (function == FastCCDMux2)
    {
      int _val;
      getIntegerParam(FastCCDMux1, &_val);
      _val = _val | (value << 4);
      cin_ctl_set_mux(&cin_ctl_port, _val);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

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

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
//asynStatus FastCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
//{
//    int function = pasynUser->reason;
//    asynStatus status = asynSuccess;
//    static const char *functionName = "writeFloat64";
//
//    /* Set the parameter and readback in the parameter library.  */
//    status = setDoubleParam(function, value);
//
//    {
//      status = ADDriver::writeFloat64(pasynUser, value);
//    }
//
//    /* Do callbacks so higher layers see any changes */
//    callParamCallbacks();
//
//    if(status)
//    {
//        asynPrint(pasynUser, ASYN_TRACE_ERROR,
//              "%s:%s: error, status=%d function=%d, value=%f\n",
//              driverName, functionName, status, function, value);
//    }
//    else
//    {
//        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
//              "%s:%s: function=%d, value=%f\n",
//              driverName, functionName, function, value);
//    }
//    return status;
//}


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
  uint16_t fpga_status;
  cin_data_stats_t stats;

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

    cin_data_compute_stats(&stats);

    cin_status  = cin_ctl_get_power_status(&cin_ctl_port, &pwr, &pwr_value);

    fprintf(stderr, "cin_status = %d, pwr = %d\n", cin_status, pwr);

    if(!cin_status){
      // Power Status
      if(pwr){
        setIntegerParam(FastCCDPower, 1);

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

      } else {
        setIntegerParam(FastCCDPower, 0);

        // Voltage Values

        setDoubleParam(FastCCDVBus12V0, 0);
        setDoubleParam(FastCCDVMgmt3v3, 0);
        setDoubleParam(FastCCDVMgmt2v5, 0);
        setDoubleParam(FastCCDVMgmt1v2, 0);
        setDoubleParam(FastCCDVEnet1v0, 0);
        setDoubleParam(FastCCDVS3E3v3,  0);
        setDoubleParam(FastCCDVGen3v3,  0);
        setDoubleParam(FastCCDVGen2v5,  0);
        setDoubleParam(FastCCDV60v9,    0);
        setDoubleParam(FastCCDV61v0,    0);
        setDoubleParam(FastCCDV62v5,    0);
        setDoubleParam(FastCCDVFp,      0);


        // Current Values

        setDoubleParam(FastCCDIBus12V0, 0);
        setDoubleParam(FastCCDIMgmt3v3, 0);
        setDoubleParam(FastCCDIMgmt2v5, 0);
        setDoubleParam(FastCCDIMgmt1v2, 0);
        setDoubleParam(FastCCDIEnet1v0, 0);
        setDoubleParam(FastCCDIS3E3v3,  0);
        setDoubleParam(FastCCDIGen3v3,  0);
        setDoubleParam(FastCCDIGen2v5,  0);
        setDoubleParam(FastCCDI60v9,    0);
        setDoubleParam(FastCCDI61v0,    0);
        setDoubleParam(FastCCDI62v5,    0);
        setDoubleParam(FastCCDIFp,      0);

      }
    }

    cin_status = cin_ctl_get_cfg_fpga_status(&cin_ctl_port, &fpga_status);
    if(!cin_status){
      fprintf(stderr, "fpga_status = %x\n", fpga_status);
      setIntegerParam(FastCCDFPGAStatus, (fpga_status & CIN_CTL_FPGA_STS_CFG));
      setIntegerParam(FastCCDFPPowerStatus, (fpga_status & CIN_CTL_FPGA_STS_FP_PWR));
    }
    
    // Status

    //cin_data_show_stats(stats);


    /* Call the callbacks to update any changes */
    this->lock();
    callParamCallbacks();
    this->unlock();
        
  } //End of loop

}

//C utility functions to tie in with EPICS

static void FastCCDStatusTaskC(void *drvPvt)
{
  FastCCD *pPvt = (FastCCD *)drvPvt;

  pPvt->statusTask();
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

