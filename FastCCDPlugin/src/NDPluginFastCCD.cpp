/*
 * NDPluginFastCCD.cpp
 *
 * Fast CCD Processing plugin
 * Author: Stuart Wilkins
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>

#include <epicsString.h>
#include <epicsMutex.h>
#include <iocsh.h>

#include <asynDriver.h>

#include <epicsExport.h>
#include "NDPluginDriver.h"
#include "NDPluginFastCCD.h"

#include "tiffio.h"

#include "cin.h"

static const char *driverName="NDPluginFastCCD";

/** Callback function that is called by the NDArray driver with new NDArray data.
  * \param[in] pArray  The NDArray from the callback.
  */
void NDPluginFastCCD::processCallbacks(NDArray *pArray)
{
    /* This function does the actual operations
     * It is called with the mutex already locked.  
     * It unlocks it during long calculations when private
     * structures don't need to be protected.
     */

    NDArray *pArrayOut = NULL;
    NDArray *pScratch = NULL;

    /* Call the base class method */
    NDPluginDriver::processCallbacks(pArray);

    /* Get paraemeters which we need for processing the image */
    double gain0, gain1, gain2, dpval;
    int dataType, enableBackground;

    getIntegerParam(NDPluginFastCCDDataType,            &dataType);
    getIntegerParam(NDPluginFastCCDEnableBackground,    &enableBackground);

    getDoubleParam(NDPluginFastCCDGain0, &gain0);
    getDoubleParam(NDPluginFastCCDGain1, &gain1);
    getDoubleParam(NDPluginFastCCDGain2, &gain2);
    getDoubleParam(NDPluginFastCCDDPVal, &dpval);

    /* This function is called with the lock taken, and it must be set when we exit.
     * The following code can be exected without the mutex because we are not accessing memory
     * that other threads can access. */
    this->unlock();

    pArray->getInfo(&arrayInfo);
    size_t nElements = arrayInfo.nElements;

    /* Make a copy of the array converted to NDFloat64 (Double) */
    
    int validBackground0 = 0;
    int validBackground1 = 0;
    int validBackground2 = 0;

    this->pNDArrayPool->convert(pArray, &pScratch, NDFloat64);
    if (NULL == pScratch) {
       asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
                 "NDPluginFastCCD::processCallbacks() : Processing aborted; cannot allocate an NDArray for storage of temporary data.\n");
       goto doCallbacks;
    }

    if (this->pBackground0 && (nElements == this->nBackground0Elements)){
      validBackground0 = 1;
    }
    setIntegerParam(NDPluginFastCCDValidBackground0, validBackground0);

    if (this->pBackground1 && (nElements == this->nBackground1Elements)){
      validBackground1 = 1;
    }
    setIntegerParam(NDPluginFastCCDValidBackground1, validBackground1);

    if (this->pBackground2 && (nElements == this->nBackground2Elements)){
      validBackground2 = 1;
    }
    setIntegerParam(NDPluginFastCCDValidBackground2, validBackground2);

    if (validBackground0 && validBackground1 && validBackground2 && enableBackground){
      setIntegerParam(NDPluginFastCCDValidBackground, 1);

      double *background0 = (double *)this->pBackground0->pData;
      double *background1 = (double *)this->pBackground1->pData;
      double *background2 = (double *)this->pBackground2->pData;
      double *data = (double *)pScratch->pData;
      epicsInt16 ctrl;

      for(size_t i=0; i<nElements; i++){
        ctrl = ((epicsInt16)data[i]) & CIN_DATA_CTRL_MASK;
        data[i] = (epicsInt16)data[i] & CIN_DATA_DATA_MASK;
        if((ctrl & CIN_DATA_GAIN_8) == CIN_DATA_GAIN_8){
          // Unity gain
          data[i] = (data[i] - background2[i]) * gain2;
        } else if ((ctrl & CIN_DATA_GAIN_4) == CIN_DATA_GAIN_4) {
          // Gain
          data[i] = (data[i] - background1[i]) * gain1;
        } else if ((ctrl & CIN_DATA_DROPPED_PACKET_VAL) == CIN_DATA_DROPPED_PACKET_VAL) {
          // Dropped Packet
          data[i] = dpval;
        } else {
          // Maximum gain
          data[i] = (data[i] - background0[i]) * gain0;
        }
      }
    }

doCallbacks:

    this->pNDArrayPool->convert(pScratch, &pArrayOut, (NDDataType_t)dataType);

    this->lock();
    if (pArrayOut != NULL) {
        /* Get the attributes from this driver */
        this->getAttributes(pArrayOut->pAttributeList);
        /* Call any clients who have registered for NDArray callbacks */
        this->unlock();
        doCallbacksGenericPointer( pArrayOut, NDArrayData, 0);
        this->lock();
        if (this->pArrays[0] != NULL){
          this->pArrays[0]->release();
        }
        this->pArrays[0] = pArrayOut;
    }

    if (NULL != pScratch) pScratch->release();

    callParamCallbacks();
}

int NDPluginFastCCD::readTiffBackground(char *filename, NDArray *array){

  static const char *functionName = "readTiffBackground";
  TIFF *tif;

  if(array == NULL){
    return -1;
  }

  if((tif = TIFFOpen(filename, "r")) == NULL){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
              "%s:%s error opening file %s\n",
              driverName, functionName, filename);
    return -1;
  }

  

  TIFFClose(tif);
}

int NDPluginFastCCD::writeTiffBackground(char *filename, NDArray *array){

  static const char *functionName = "writeTiffBackground";
  TIFF *tif;

  if(array == NULL){
    return -1;
  }

  if((tif = TIFFOpen(filename, "w")) == NULL){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
              "%s:%s error opening file %s\n",
              driverName, functionName, filename);
    return -1;
  }

  if((array->ndims != 2) || (array->dataType != NDFloat64)){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
              "%s:%s NDArray is not of correct type\n",
              driverName, functionName);
    return -1;
  }

  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE,   64);
  TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT,    SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG,    PLANARCONFIG_CONTIG);
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH,      (epicsUInt32)array->dims[0].size);
  IFFSetField(tif, TIFFTAG_IMAGELENGTH,     (epicsUInt32)array->dims[1].size);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP,    (epicsUInt32)array->dims[1].size);

  tsize_t stripSize = TIFFStripSize(tif);
  size_t nwrite = TIFFWriteEncodedStrip(tif, 0, array->pData, stripSize);

  TIFFClose(tif);

}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus NDPluginFastCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int addr=0;
    NDArrayInfo arrayInfo;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeInt32";

    status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(addr, function, value);

    if (function == NDPluginFastCCDSaveBackground0){
        setIntegerParam(NDPluginFastCCDSaveBackground0, 0);
        if (this->pBackground0) {
          this->pBackground0->release();
        }
        this->pBackground0 = NULL;
        if (this->pArrays[0]) {
            /* Make a copy of the current array, converted to double type */
            this->pNDArrayPool->convert(this->pArrays[0], &this->pBackground0, NDFloat64);
            this->pBackground0->getInfo(&arrayInfo);
            nBackground0Elements= arrayInfo.nElements;
            setIntegerParam(NDPluginFastCCDValidBackground0, 1);
        }
    } else if (function == NDPluginFastCCDSaveBackground1){
        setIntegerParam(NDPluginFastCCDSaveBackground1, 0);
        if (this->pBackground1) {
          this->pBackground1->release();
        }
        this->pBackground1 = NULL;

        if (this->pArrays[0]) {
            /* Make a copy of the current array, converted to double type */
            this->pNDArrayPool->convert(this->pArrays[0], &this->pBackground1, NDFloat64);
            this->pBackground1->getInfo(&arrayInfo);
            nBackground1Elements= arrayInfo.nElements;
            setIntegerParam(NDPluginFastCCDValidBackground1, 1);
        }
    } else if (function == NDPluginFastCCDSaveBackground2){
        setIntegerParam(NDPluginFastCCDSaveBackground2, 0);
        if (this->pBackground2) {
          this->pBackground2->release();
        }
        this->pBackground2 = NULL;

        if (this->pArrays[0]) {
            /* Make a copy of the current array, converted to double type */
            this->pNDArrayPool->convert(this->pArrays[0], &this->pBackground2, NDFloat64);
            this->pBackground2->getInfo(&arrayInfo);
            nBackground2Elements= arrayInfo.nElements;
            setIntegerParam(NDPluginFastCCDValidBackground2, 1);
        }
    } else if(function == NDPluginFastCCDBackground0SaveFile){
      setIntegerParam(NDPluginFastCCDBackground0SaveFile, 0);
      char fn[256];
      getStringParam(NDPluginFastCCDBackground0Path, sizeof(fn), fn);
      if(writeTiffBackground(fn, this->pBackground0)){
        status = asynError;
      }
    } else if(function == NDPluginFastCCDBackground1SaveFile){
      setIntegerParam(NDPluginFastCCDBackground1SaveFile, 0);
      char fn[256];
      getStringParam(NDPluginFastCCDBackground1Path, sizeof(fn), fn);
      if(writeTiffBackground(fn, this->pBackground1)){
        status = asynError;
      }
    } else if(function == NDPluginFastCCDBackground2SaveFile){
      setIntegerParam(NDPluginFastCCDBackground2SaveFile, 0);
      char fn[256];
      getStringParam(NDPluginFastCCDBackground2Path, sizeof(fn), fn);
      if(writeTiffBackground(fn, this->pBackground2)){
        status = asynError;
      }
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_NDPLUGIN_FASTCCD_PARAM) {
            status = NDPluginDriver::writeInt32(pasynUser, value);
        }
    }
    
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks(addr);
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, value=%d", 
                  driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}

asynStatus NDPluginFastCCD::writeOctet(asynUser *pasynUser, const char *value, size_t nc, size_t *na){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  static const char *functionName = "writeOctet";

  status = getAddress(pasynUser, &addr);
  if(status != asynSuccess){
    return status;
  }
  status = (asynStatus)setStringParam(addr, function, (char *)value);
  if(status != asynSuccess){
    return status;
  }

  if((function == NDPluginFastCCDBackground0Path) ||
     (function == NDPluginFastCCDBackground1Path) ||
     (function == NDPluginFastCCDBackground2Path)) {
    struct stat s;
    int _status = stat(value, &s);
    if(_status){
      setParamStatus(function, asynError);
    } else {
      setParamStatus(function, asynSuccess);
    }
  } else {
    // Call base class to handle strings
    status = NDPluginDriver::writeOctet(pasynUser, value, nc, na);
  }

  if (status) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
          "%s:%s: error, status=%d function=%d, value=%s\n",
          driverName, functionName, status, function, value);
  } else {
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
          "%s:%s: function=%d, value=%s\n",
          driverName, functionName, function, value);
  }


  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();
 
  // Set the number of characters written
  *na = nc;
  return status;
}

/** Constructor for NDPluginFastCCD; most parameters are simply passed to NDPluginDriver::NDPluginDriver.

  * After calling the base class constructor this method sets reasonable default values for all of the
  * ROI parameters.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] queueSize The number of NDArrays that the input queue for this plugin can hold when
  *            NDPluginDriverBlockingCallbacks=0.  Larger queues can decrease the number of dropped arrays,
  *            at the expense of more NDArray buffers being allocated from the underlying driver's NDArrayPool.
  * \param[in] blockingCallbacks Initial setting for the NDPluginDriverBlockingCallbacks flag.
  *            0=callbacks are queued and executed by the callback thread; 1 callbacks execute in the thread
  *            of the driver doing the callbacks.
  * \param[in] NDArrayPort Name of asyn port driver for initial source of NDArray callbacks.
  * \param[in] NDArrayAddr asyn port driver address for initial source of NDArray callbacks.
  * \param[in] maxOverlays The maximum number ofoverlays this plugin supports. 1 is minimum.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
NDPluginFastCCD::NDPluginFastCCD(const char *portName, int queueSize, int blockingCallbacks,
                         const char *NDArrayPort, int NDArrayAddr, 
                         int maxBuffers, size_t maxMemory,
                         int priority, int stackSize)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks,
                   NDArrayPort, NDArrayAddr, 1, NUM_NDPLUGIN_FASTCCD_PARAMS, 
                   maxBuffers, maxMemory,
                   asynGenericPointerMask,
                   asynGenericPointerMask,
                   ASYN_MULTIDEVICE, 1, priority, stackSize)
{
    //static const char *functionName = "NDPluginFastCCD";

    createParam(NDPluginFastCCDGain0String,      asynParamFloat64,  &NDPluginFastCCDGain0);
    createParam(NDPluginFastCCDGain1String,      asynParamFloat64,  &NDPluginFastCCDGain1);
    createParam(NDPluginFastCCDGain2String,      asynParamFloat64,  &NDPluginFastCCDGain2);
    createParam(NDPluginFastCCDDPValString,      asynParamFloat64,  &NDPluginFastCCDDPVal);
    createParam(NDPluginFastCCDEnableBackgroundString, asynParamInt32,  
                &NDPluginFastCCDEnableBackground);
    createParam(NDPluginFastCCDDataTypeString,   asynParamInt32,    &NDPluginFastCCDDataType);
    createParam(NDPluginFastCCDValidBackgroundString,  asynParamInt32,    
                &NDPluginFastCCDValidBackground);
    createParam(NDPluginFastCCDValidBackground0String,  asynParamInt32,    
                &NDPluginFastCCDValidBackground0);
    createParam(NDPluginFastCCDValidBackground1String,  asynParamInt32,    
                &NDPluginFastCCDValidBackground1);
    createParam(NDPluginFastCCDValidBackground2String,  asynParamInt32,    
                &NDPluginFastCCDValidBackground2);
    createParam(NDPluginFastCCDSaveBackground0String,   asynParamInt32,    
                &NDPluginFastCCDSaveBackground0);
    createParam(NDPluginFastCCDSaveBackground1String,   asynParamInt32,    
                &NDPluginFastCCDSaveBackground1);
    createParam(NDPluginFastCCDSaveBackground2String,   asynParamInt32,    
                &NDPluginFastCCDSaveBackground2);
    createParam(NDPluginFastCCDBackground0PathString,   asynParamOctet,
                &NDPluginFastCCDBackground0Path);
    createParam(NDPluginFastCCDBackground1PathString,   asynParamOctet,
                &NDPluginFastCCDBackground1Path);
    createParam(NDPluginFastCCDBackground2PathString,   asynParamOctet,
                &NDPluginFastCCDBackground2Path);
    createParam(NDPluginFastCCDBackground0SaveFileString,   asynParamInt32,
                &NDPluginFastCCDBackground0SaveFile);
    createParam(NDPluginFastCCDBackground1SaveFileString,   asynParamInt32,
                &NDPluginFastCCDBackground1SaveFile);
    createParam(NDPluginFastCCDBackground2SaveFileString,   asynParamInt32,
                &NDPluginFastCCDBackground2SaveFile);
    createParam(NDPluginFastCCDBackground0LoadFileString,   asynParamInt32,
                &NDPluginFastCCDBackground0LoadFile);
    createParam(NDPluginFastCCDBackground1LoadFileString,   asynParamInt32,
                &NDPluginFastCCDBackground1LoadFile);
    createParam(NDPluginFastCCDBackground2LoadFileString,   asynParamInt32,
                &NDPluginFastCCDBackground2LoadFile);

    setIntegerParam(NDPluginFastCCDGain0, 1);
    setIntegerParam(NDPluginFastCCDGain1, 1);
    setIntegerParam(NDPluginFastCCDGain2, 1);
    setIntegerParam(NDPluginFastCCDDPVal, 0);

    setIntegerParam(NDPluginFastCCDValidBackground, 0);
    setIntegerParam(NDPluginFastCCDValidBackground0, 0);
    setIntegerParam(NDPluginFastCCDValidBackground1, 0);
    setIntegerParam(NDPluginFastCCDValidBackground2, 0);

    setIntegerParam(NDPluginFastCCDEnableBackground, 0);
    setIntegerParam(NDPluginFastCCDDataType, NDFloat64);

    setStringParam(NDPluginFastCCDBackground0Path, "");
    setStringParam(NDPluginFastCCDBackground1Path, "");
    setStringParam(NDPluginFastCCDBackground2Path, "");

    pBackground0 = NULL;
    pBackground1 = NULL;
    pBackground2 = NULL;
    nBackground0Elements = 0;
    nBackground1Elements = 0;
    nBackground2Elements = 0;

    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "NDPluginFastCCD");

    /* Call Parameter Callbacks */
    callParamCallbacks();

    /* Try to connect to the array port */
    connectToArrayPort();
}

/** Configuration command */
extern "C" int NDFastCCDConfigure(const char *portName, int queueSize, int blockingCallbacks,
                                 const char *NDArrayPort, int NDArrayAddr, 
                                 int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize)
{
    new NDPluginFastCCD(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, 
                        maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "frame queue size",iocshArgInt};
static const iocshArg initArg2 = { "blocking callbacks",iocshArgInt};
static const iocshArg initArg3 = { "NDArrayPort",iocshArgString};
static const iocshArg initArg4 = { "NDArrayAddr",iocshArgInt};
static const iocshArg initArg5 = { "maxBuffers",iocshArgInt};
static const iocshArg initArg6 = { "maxMemory",iocshArgInt};
static const iocshArg initArg7 = { "priority",iocshArgInt};
static const iocshArg initArg8 = { "stackSize",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3,
                                            &initArg4,
                                            &initArg5,
                                            &initArg6,
                                            &initArg7,
                                            &initArg8};
static const iocshFuncDef initFuncDef = {"NDFastCCDConfigure",9,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    NDFastCCDConfigure(args[0].sval, args[1].ival, args[2].ival,
                       args[3].sval, args[4].ival, args[5].ival,
                       args[6].ival, args[7].ival, args[8].ival);
}

extern "C" void NDFastCCDRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
epicsExportRegistrar(NDFastCCDRegister);
}
