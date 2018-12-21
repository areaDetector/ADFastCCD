/*
 * NDPluginFastCCD.cpp
 *
 * FastCCD Plugin
 * Author: Stuart Wilkins
 *
 * Created October 14th 2017
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsMessageQueue.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <iocsh.h>

#include <asynDriver.h>

#include <epicsExport.h>
#include "NDPluginDriver.h"
#include "NDPluginFastCCD.h"

#include <unistd.h>

static const char *driverName="NDPluginFastCCD";


/** Callback function that is called by the NDArray driver with new NDArray data.
  * \param[in] pArray  The NDArray from the callback.
  */
void NDPluginFastCCD::processCallbacks(NDArray *pArray)
{
  //static const char* functionName = "processCallbacks";
  NDArray *pOutput;
  NDAttribute *pAttribute;
  size_t dims[2];
  int nDims = 2;
  int rowOffset;
  int rows, overscanCols; 
  int correctSize, correctGain, correctBgnd;
  int attrOver, captureBgnd, validBgnd;
  int bgndSubtract = 0;
    
  /* Get all parameters while we have the mutex */
  getIntegerParam(NDPluginFastCCDRowOffset,      &rowOffset);
  getIntegerParam(NDPluginFastCCDRows,           &rows);
  getIntegerParam(NDPluginFastCCDOverscanCols,   &overscanCols);
  getIntegerParam(NDPluginFastCCDEnableSize,     &correctSize);
  getIntegerParam(NDPluginFastCCDEnableGain,     &correctGain);
  getIntegerParam(NDPluginFastCCDEnableBgnd,     &correctBgnd);
  getIntegerParam(NDPluginFastCCDAttrOver,       &attrOver);
  getIntegerParam(NDPluginFastCCDCaptureBgnd,    &captureBgnd);

  setIntegerParam(NDPluginFastCCDValidImage, 0);
  setIntegerParam(NDPluginFastCCDBgndSubtr, 0);

  /* Call the base class method */
  NDPluginDriver::beginProcessCallbacks(pArray);

  /* we only work on 2d arrays */

  if(pArray->ndims != 2)
  {
    NDPluginDriver::endProcessCallbacks(pArray, true, true);
    callParamCallbacks();
    return;
  }

  if(captureBgnd)
  {
    if(pBackground != NULL){
      pBackground->release();
      pBackground = NULL;
    }
    this->pNDArrayPool->convert(pArray, &pBackground, NDInt16);
    setIntegerParam(NDPluginFastCCDCaptureBgnd, 0);
    callParamCallbacks();
    return;
  }

  validBgnd = 0;
  if(pBackground != NULL)
  {
    if(pBackground->ndims == 2)
    {
      if((pBackground->dims[0].size == pArray->dims[0].size) && 
         (pBackground->dims[1].size == pArray->dims[1].size))
      {
        validBgnd = 1;
      }
    }
  }

  setIntegerParam(NDPluginFastCCDValidBgnd,      validBgnd);

  if(correctSize)
  {
    if(attrOver)
    {
      pAttribute = pArray->pAttributeList->find("OverscanColumns");
      if(pAttribute) pAttribute->getValue(NDAttrInt32, &overscanCols);
      pAttribute = pArray->pAttributeList->find("SensorRows");
      if(pAttribute) pAttribute->getValue(NDAttrInt32, &rows);
      pAttribute = pArray->pAttributeList->find("SensorRowOffset");
      if(pAttribute) pAttribute->getValue(NDAttrInt32, &rowOffset);
      setIntegerParam(NDPluginFastCCDRowOffset,      rowOffset);
      setIntegerParam(NDPluginFastCCDRows,           rows);
      setIntegerParam(NDPluginFastCCDOverscanCols,   overscanCols);
    }

    // rows is the number of rows in the CCD for *half* of the image.
    // The dims[1] size is reduced by 10+overscan / 10
    //
    // Do some bounds checking here
    int newCols = FCCD_SCOL_N * pArray->dims[0].size / (FCCD_SCOL_N + overscanCols);

    if(((unsigned int)(rows + rowOffset) > (pArray->dims[1].size / 2)) ||
       ((unsigned int)newCols > pArray->dims[0].size))
    {
      // Invalid dimensions, just pass array
      NDPluginDriver::endProcessCallbacks(pArray, true, true);
      callParamCallbacks();
      return;
    }

    dims[0] = newCols;
    dims[1] = rows * 2;

  } else {
    // This is just a passthrough, not efficient but works
    dims[1] = pArray->dims[1].size;
    dims[0] = pArray->dims[0].size;
    rowOffset = 0;
    overscanCols = 0;
    rows = dims[1];
  }

  pOutput = this->pNDArrayPool->alloc(nDims, dims, NDFloat32, 0, NULL);

  bgndSubtract = validBgnd && correctBgnd;
  setIntegerParam(NDPluginFastCCDBgndSubtr, bgndSubtract);

  /* This function is called with the lock taken, and it must be set when we exit.
   * The following code can be exected without the mutex because we are not accessing memory
   * that other threads can access. */
  this->unlock();
  asynStatus stat = processImage(pArray, pOutput, rowOffset, overscanCols, correctGain, bgndSubtract);
  this->lock();

  if(stat != asynSuccess)
  {
    pOutput->release();
    setIntegerParam(NDPluginFastCCDValidImage, 0);
    NDPluginDriver::endProcessCallbacks(pArray, true, true);
    callParamCallbacks();
    return;
  }

  setIntegerParam(NDPluginFastCCDValidImage, 1);
  NDPluginDriver::endProcessCallbacks(pOutput, false, true);
  callParamCallbacks();
  return;
}

asynStatus NDPluginFastCCD::processImage(NDArray *pIn, NDArray *pOut, 
                                         int rowOffset, int overscanCols, int correctGain, int bgndSubtract)
{
  asynStatus status;

  switch(pIn->dataType)
  {
    case NDInt32:
      status = processImageT<epicsInt32>(pIn, pOut, rowOffset, overscanCols, correctGain, bgndSubtract);
      break;
    case NDInt16:
      status = processImageT<epicsInt16>(pIn, pOut, rowOffset, overscanCols, correctGain, bgndSubtract);
      break;
    case NDUInt16:
      status = processImageT<epicsInt16>(pIn, pOut, rowOffset, overscanCols, correctGain, bgndSubtract);
      break;
    default:
      status = asynError;
      break;
  }

  return status;
}

template <typename epicsType>
asynStatus NDPluginFastCCD::processImageT(NDArray *pIn, NDArray *pOut, 
                                          int rowOffset, int overscanCols, int correctGain,
                                          int bgndSubtract)
{
  epicsType *pInData = (epicsType *)pIn->pData;
  epicsFloat32 *pOutData = (epicsFloat32 *)pOut->pData;
  epicsInt16 *pBackData = NULL;

  if(bgndSubtract)
  {
    pBackData = (epicsInt16 *)pBackground->pData;
  }

  int oldCols = pIn->dims[0].size;
  int oldRows = pIn->dims[1].size;
  int newCols = pOut->dims[0].size;
  int newRows = pOut->dims[1].size;

  for(int row=0;row<(newRows/2) ;row++)
  {
    int newCol = 0;
    for(int col=0;col<oldCols;col++)
    {
      if(col % (FCCD_SCOL_N + overscanCols) >= overscanCols)
      {
        int nOut = newCol + (newCols * row);
        int nIn = col + (oldCols * (row + rowOffset));
        if(bgndSubtract)
        {
          pOutData[nOut] = correctPixel<epicsType>(pInData[nIn], pBackData[nIn], correctGain);
        } else {
          pOutData[nOut] = correctPixel<epicsType>(pInData[nIn], 0, correctGain);
        }
        newCol++;
      }
    }
  }

  for(int row=0;row<(newRows/2) ;row++)
  {
    int newCol = 0;
    for(int col=0;col<oldCols;col++)
    {
      if(col % (FCCD_SCOL_N + overscanCols) < FCCD_SCOL_N)
      {
        int nOut = newCol + (newCols * (newRows - 1 - row));
        int nIn = col + (oldCols * (oldRows - 1 - row - rowOffset));
        if(bgndSubtract)
        {
          pOutData[nOut] = correctPixel<epicsType>(pInData[nIn], pBackData[nIn], correctGain);
        } else {
          pOutData[nOut] = correctPixel<epicsType>(pInData[nIn], 0, correctGain);
        }
        newCol++;
      }
    }
  }

  return asynSuccess;
}

template <typename epicsType>
epicsFloat32 NDPluginFastCCD::correctPixel(epicsType inp, epicsInt16 bgnd, int correctGain)
{
  epicsFloat32 outp; 
  double gain8, gain2, gain1;

  getDoubleParam(NDPluginFastCCDGain8, &gain8);
  getDoubleParam(NDPluginFastCCDGain2, &gain2);
  getDoubleParam(NDPluginFastCCDGain1, &gain1);

  if(correctGain){
    if((inp & FCCD_GAIN_1) == FCCD_GAIN_1)
    {
      outp = ((inp & FCCD_MASK) - bgnd) * gain1;
    } else if ((inp & FCCD_GAIN_2) == FCCD_GAIN_2) {
      outp = ((inp & FCCD_MASK) - bgnd) * gain2;
    } else {
      // Gain 8
      outp = ((inp & FCCD_MASK) - bgnd) * gain8;
    }
  } else {
    outp = inp - bgnd;
  }
  return outp;
}
/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including NDPluginDriverEnableCallbacks and
  * NDPluginDriverArrayAddr.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus NDPluginFastCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char* functionName = "writeInt32";

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    if (function == NDPluginFastCCDTest){
      int i;
      getIntegerParam(NDPluginFastCCDTest, &i);
      sleep(10);
      if(i == 0)
      {
        status = asynError;
      } else {
        setIntegerParam(NDPluginFastCCDTest, i + 1);
      }
        
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_NDPLUGIN_ROI_PARAM) 
            status = NDPluginDriver::writeInt32(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}


/** Constructor for NDPluginFastCCD; most parameters are simply passed to NDPluginDriver::NDPluginDriver.
  * After calling the base class constructor this method sets reasonable default values for all of the
  * plugin parameters.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] queueSize The number of NDArrays that the input queue for this plugin can hold when
  *            NDPluginDriverBlockingCallbacks=0.  Larger queues can decrease the number of dropped arrays,
  *            at the expense of more NDArray buffers being allocated from the underlying driver's NDArrayPool.
  * \param[in] blockingCallbacks Initial setting for the NDPluginDriverBlockingCallbacks flag.
  *            0=callbacks are queued and executed by the callback thread; 1 callbacks execute in the thread
  *            of the driver doing the callbacks.
  * \param[in] NDArrayPort Name of asyn port driver for initial source of NDArray callbacks.
  * \param[in] NDArrayAddr asyn port driver address for initial source of NDArray callbacks.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to 0 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to 0 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] maxThreads The maximum number of threads this driver is allowed to use. If 0 then 1 will be used.
  */
NDPluginFastCCD::NDPluginFastCCD(const char *portName, int queueSize, int blockingCallbacks,
                         const char *NDArrayPort, int NDArrayAddr,
                         int maxBuffers, size_t maxMemory,
                         int priority, int stackSize, int maxThreads)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks,
                   NDArrayPort, NDArrayAddr, 1, maxBuffers, maxMemory,
                   asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                   asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                   ASYN_MULTIDEVICE, 1, priority, stackSize, maxThreads)
{
    //static const char *functionName = "NDPluginFastCCD";

    createParam(NDPluginFastCCDNameString,              asynParamOctet,   &NDPluginFastCCDName);
    createParam(NDPluginFastCCDRowsString,              asynParamInt32,   &NDPluginFastCCDRows);
    createParam(NDPluginFastCCDRowOffsetString,         asynParamInt32,   &NDPluginFastCCDRowOffset);
    createParam(NDPluginFastCCDOverscanColsString,      asynParamInt32,   &NDPluginFastCCDOverscanCols);
    createParam(NDPluginFastCCDEnableGainString,        asynParamInt32,   &NDPluginFastCCDEnableGain);
    createParam(NDPluginFastCCDEnableSizeString,        asynParamInt32,   &NDPluginFastCCDEnableSize);
    createParam(NDPluginFastCCDEnableBgndString,        asynParamInt32,   &NDPluginFastCCDEnableBgnd);
    createParam(NDPluginFastCCDAttrOverString,          asynParamInt32,   &NDPluginFastCCDAttrOver);
    createParam(NDPluginFastCCDValidBgndString,         asynParamInt32,   &NDPluginFastCCDValidBgnd);
    createParam(NDPluginFastCCDValidImageString,        asynParamInt32,   &NDPluginFastCCDValidImage);
    createParam(NDPluginFastCCDCaptureBgndString,       asynParamInt32,   &NDPluginFastCCDCaptureBgnd);
    createParam(NDPluginFastCCDBgndSubtrString,         asynParamInt32,   &NDPluginFastCCDBgndSubtr);
    createParam(NDPluginFastCCDTestString,              asynParamInt32,   &NDPluginFastCCDTest);
    createParam(NDPluginFastCCDGain8String,             asynParamFloat64, &NDPluginFastCCDGain8);
    createParam(NDPluginFastCCDGain2String,             asynParamFloat64, &NDPluginFastCCDGain2);
    createParam(NDPluginFastCCDGain1String,             asynParamFloat64, &NDPluginFastCCDGain1);

    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "NDPluginFastCCD");
    setIntegerParam(NDPluginFastCCDRows, 480);
    setIntegerParam(NDPluginFastCCDRowOffset, 0);
    setIntegerParam(NDPluginFastCCDOverscanCols, 0);
    setIntegerParam(NDPluginFastCCDEnableGain, 0);
    setIntegerParam(NDPluginFastCCDEnableSize, 0);
    setIntegerParam(NDPluginFastCCDEnableBgnd, 0);
    setIntegerParam(NDPluginFastCCDAttrOver, 0);
    setIntegerParam(NDPluginFastCCDCaptureBgnd, 0);
    setIntegerParam(NDPluginFastCCDValidBgnd, 0);
    setIntegerParam(NDPluginFastCCDValidImage, 0);
    setIntegerParam(NDPluginFastCCDBgndSubtr, 0);
    setIntegerParam(NDPluginFastCCDTest, 0);
    setDoubleParam(NDPluginFastCCDGain8, 1.0);
    setDoubleParam(NDPluginFastCCDGain2, 4.0);
    setDoubleParam(NDPluginFastCCDGain1, 8.0);

    pBackground = NULL;

    /* Try to connect to the array port */
    connectToArrayPort();
}

/** Configuration command */
extern "C" int NDFastCCDConfigure(const char *portName, int queueSize, int blockingCallbacks,
                                  const char *NDArrayPort, int NDArrayAddr,
                                  int maxBuffers, size_t maxMemory,
                                  int priority, int stackSize, int maxThreads)
{
    NDPluginFastCCD *pPlugin = new NDPluginFastCCD(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
                                           maxBuffers, maxMemory, priority, stackSize, maxThreads);
    return pPlugin->start();
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
static const iocshArg initArg9 = { "maxThreads",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3,
                                            &initArg4,
                                            &initArg5,
                                            &initArg6,
                                            &initArg7,
                                            &initArg8,
                                            &initArg9};
static const iocshFuncDef initFuncDef = {"NDFastCCDConfigure",10,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    NDFastCCDConfigure(args[0].sval, args[1].ival, args[2].ival,
                   args[3].sval, args[4].ival, args[5].ival,
                   args[6].ival, args[7].ival, args[8].ival,
                   args[9].ival);
}

extern "C" void NDFastCCDRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
epicsExportRegistrar(NDFastCCDRegister);
}
