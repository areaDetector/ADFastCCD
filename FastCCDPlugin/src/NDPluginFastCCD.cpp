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

#define MAX(A,B) (A)>(B)?(A):(B)
#define MIN(A,B) (A)<(B)?(A):(B)

static const char *driverName="NDPluginFastCCD";


/** Callback function that is called by the NDArray driver with new NDArray data.
  * Extracts the NthrDArray data into each of the ROIs that are being used.
  * Computes statistics on the ROI if NDPluginFastCCDComputeStatistics is 1.
  * Computes the histogram of ROI values if NDPluginFastCCDComputeHistogram is 1.
  * \param[in] pArray  The NDArray from the callback.
  */
void NDPluginFastCCD::processCallbacks(NDArray *pArray)
{
    /* This function computes the ROIs.
     * It is called with the mutex already locked.  It unlocks it during long calculations when private
     * structures don't need to be protected.
     */

    int dataType;
    int dim;
    NDDimension_t dims[ND_ARRAY_MAX_DIMS], tempDim, *pDim;
    size_t userDims[ND_ARRAY_MAX_DIMS];
    NDArrayInfo arrayInfo, scratchInfo;
    NDArray *pScratch, *pOutput;
    NDColorMode_t colorMode;
    double *pData;
    int enableScale, enableDim[3], autoSize[3];
    size_t i;
    double scale;
    int collapseDims;
    //static const char* functionName = "processCallbacks";
    
    memset(dims, 0, sizeof(NDDimension_t) * ND_ARRAY_MAX_DIMS);

    /* Get all parameters while we have the mutex */
    getIntegerParam(NDPluginFastCCDDim0Bin,      &dims[0].binning);
    getIntegerParam(NDPluginFastCCDDim1Bin,      &dims[1].binning);
    getIntegerParam(NDPluginFastCCDDim2Bin,      &dims[2].binning);
    getIntegerParam(NDPluginFastCCDDim0Reverse,  &dims[0].reverse);
    getIntegerParam(NDPluginFastCCDDim1Reverse,  &dims[1].reverse);
    getIntegerParam(NDPluginFastCCDDim2Reverse,  &dims[2].reverse);
    getIntegerParam(NDPluginFastCCDDim0Enable,   &enableDim[0]);
    getIntegerParam(NDPluginFastCCDDim1Enable,   &enableDim[1]);
    getIntegerParam(NDPluginFastCCDDim2Enable,   &enableDim[2]);
    getIntegerParam(NDPluginFastCCDDim0AutoSize, &autoSize[0]);
    getIntegerParam(NDPluginFastCCDDim1AutoSize, &autoSize[1]);
    getIntegerParam(NDPluginFastCCDDim2AutoSize, &autoSize[2]);
    getIntegerParam(NDPluginFastCCDDataType,     &dataType);
    getIntegerParam(NDPluginFastCCDEnableScale,  &enableScale);
    getDoubleParam(NDPluginFastCCDScale, &scale);
    getIntegerParam(NDPluginFastCCDCollapseDims, &collapseDims);

    /* Call the base class method */
    NDPluginDriver::beginProcessCallbacks(pArray);
    
    /* Get information about the array */
    pArray->getInfo(&arrayInfo);
    
    userDims[0] = arrayInfo.xDim;
    userDims[1] = arrayInfo.yDim;
    userDims[2] = arrayInfo.colorDim;

    /* Make sure dimensions are valid, fix them if they are not */
    for (dim=0; dim<pArray->ndims; dim++) {
        pDim = &dims[dim];
        if (enableDim[dim]) {
            size_t newDimSize = pArray->dims[userDims[dim]].size;
            pDim->offset  = requestedOffset_[dim];
            pDim->size    = requestedSize_[dim];
            pDim->offset  = MAX(pDim->offset,  0);
            pDim->offset  = MIN(pDim->offset,  newDimSize-1);
            if (autoSize[dim]) pDim->size = newDimSize;
            pDim->size    = MAX(pDim->size,    1);
            pDim->size    = MIN(pDim->size,    newDimSize - pDim->offset);
            pDim->binning = MAX(pDim->binning, 1);
            pDim->binning = MIN(pDim->binning, (int)pDim->size);
        } else {
            pDim->offset  = 0;
            pDim->size    = pArray->dims[userDims[dim]].size;
            pDim->binning = 1;
        }
    }

    /* Update the parameters that may have changed */
    setIntegerParam(NDPluginFastCCDDim0MaxSize, 0);
    setIntegerParam(NDPluginFastCCDDim1MaxSize, 0);
    setIntegerParam(NDPluginFastCCDDim2MaxSize, 0);
    if (pArray->ndims > 0) {
        pDim = &dims[0];
        setIntegerParam(NDPluginFastCCDDim0MaxSize, (int)pArray->dims[userDims[0]].size);
        if (enableDim[0]) {
            setIntegerParam(NDPluginFastCCDDim0Min,  (int)pDim->offset);
            setIntegerParam(NDPluginFastCCDDim0Size, (int)pDim->size);
            setIntegerParam(NDPluginFastCCDDim0Bin,  pDim->binning);
        }
    }
    if (pArray->ndims > 1) {
        pDim = &dims[1];
        setIntegerParam(NDPluginFastCCDDim1MaxSize, (int)pArray->dims[userDims[1]].size);
        if (enableDim[1]) {
            setIntegerParam(NDPluginFastCCDDim1Min,  (int)pDim->offset);
            setIntegerParam(NDPluginFastCCDDim1Size, (int)pDim->size);
            setIntegerParam(NDPluginFastCCDDim1Bin,  pDim->binning);
        }
    }
    if (pArray->ndims > 2) {
        pDim = &dims[2];
        setIntegerParam(NDPluginFastCCDDim2MaxSize, (int)pArray->dims[userDims[2]].size);
        if (enableDim[2]) {
            setIntegerParam(NDPluginFastCCDDim2Min,  (int)pDim->offset);
            setIntegerParam(NDPluginFastCCDDim2Size, (int)pDim->size);
            setIntegerParam(NDPluginFastCCDDim2Bin,  pDim->binning);
        }
    }

    /* This function is called with the lock taken, and it must be set when we exit.
     * The following code can be exected without the mutex because we are not accessing memory
     * that other threads can access. */
    this->unlock();

    /* Extract this ROI from the input array.  The convert() function allocates
     * a new array and it is reserved (reference count = 1) */
    if (dataType == -1) dataType = (int)pArray->dataType;
    /* We treat the case of RGB1 data specially, so that NX and NY are the X and Y dimensions of the
     * image, not the first 2 dimensions.  This makes it much easier to switch back and forth between
     * RGB1 and mono mode when using an ROI. */
    if (arrayInfo.colorMode == NDColorModeRGB1) {
        tempDim = dims[0];
        dims[0] = dims[2];
        dims[2] = dims[1];
        dims[1] = tempDim;
    }
    else if (arrayInfo.colorMode == NDColorModeRGB2) {
        tempDim = dims[1];
        dims[1] = dims[2];
        dims[2] = tempDim;
    }
    
    if (enableScale && (scale != 0) && (scale != 1)) {
        /* This is tricky.  We want to do the operation to avoid errors due to integer truncation.
         * For example, if an image with all pixels=1 is binned 3x3 with scale=9 (divide by 9), then
         * the output should also have all pixels=1. 
         * We do this by extracting the ROI and converting to double, do the scaling, then convert
         * to the desired data type. */
        this->pNDArrayPool->convert(pArray, &pScratch, NDFloat64, dims);
        pScratch->getInfo(&scratchInfo);
        pData = (double *)pScratch->pData;
        for (i=0; i<scratchInfo.nElements; i++) pData[i] = pData[i]/scale;
        this->pNDArrayPool->convert(pScratch, &pOutput, (NDDataType_t)dataType);
        pScratch->release();
    } 
    else {        
        this->pNDArrayPool->convert(pArray, &pOutput, (NDDataType_t)dataType, dims);
    }

    /* If we selected just one color from the array, then we need to collapse the
     * dimensions and set the color mode to mono */
    colorMode = NDColorModeMono;
    if ((pOutput->ndims == 3) && 
        (arrayInfo.colorMode == NDColorModeRGB1) && 
        (pOutput->dims[0].size == 1)) 
    {
        collapseDims = 1;
        pOutput->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    }
    else if ((pOutput->ndims == 3) && 
        (arrayInfo.colorMode == NDColorModeRGB2) && 
        (pOutput->dims[1].size == 1)) 
    {
        collapseDims = 1;
        pOutput->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    }
    else if ((pOutput->ndims == 3) && 
        (arrayInfo.colorMode == NDColorModeRGB3) && 
        (pOutput->dims[2].size == 1)) 
    {
        collapseDims = 1;
        pOutput->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    }
    
    /* If collapseDims is set then collapse any dimensions of size 1 */
    if (collapseDims) {
        int i=0, j;
        while ((i < pOutput->ndims) && (pOutput->ndims > 1)) {
            if (pOutput->dims[i].size == 1) {
                for (j=i+1; j<pOutput->ndims; j++) {
                    pOutput->dims[j-1] = pOutput->dims[j];
                }
                if (pOutput->ndims > 1) pOutput->ndims--;
            } else {
               i++;
            }
        }
    }
    this->lock();

    /* Set the image size of the ROI image data */
    setIntegerParam(NDArraySizeX, 0);
    setIntegerParam(NDArraySizeY, 0);
    setIntegerParam(NDArraySizeZ, 0);
    if (pOutput->ndims > 0) setIntegerParam(NDArraySizeX, (int)pOutput->dims[userDims[0]].size);
    if (pOutput->ndims > 1) setIntegerParam(NDArraySizeY, (int)pOutput->dims[userDims[1]].size);
    if (pOutput->ndims > 2) setIntegerParam(NDArraySizeZ, (int)pOutput->dims[userDims[2]].size);

    NDPluginDriver::endProcessCallbacks(pOutput, false, true);
    callParamCallbacks();

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

    if        (function == NDPluginFastCCDDim0Min) {
        requestedOffset_[0] = value;
    } else if (function == NDPluginFastCCDDim1Min) {
        requestedOffset_[1] = value;
    } else if (function == NDPluginFastCCDDim2Min) {
        requestedOffset_[2] = value;
    } else if (function == NDPluginFastCCDDim0Size) {
        requestedSize_[0] = value;
    } else if (function == NDPluginFastCCDDim1Size) {
        requestedSize_[1] = value;
    } else if (function == NDPluginFastCCDDim2Size) {
        requestedSize_[2] = value;
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

    /* ROI general parameters */
    createParam(NDPluginFastCCDNameString,              asynParamOctet, &NDPluginFastCCDName);

     /* ROI definition */
    createParam(NDPluginFastCCDDim0MinString,           asynParamInt32, &NDPluginFastCCDDim0Min);
    createParam(NDPluginFastCCDDim1MinString,           asynParamInt32, &NDPluginFastCCDDim1Min);
    createParam(NDPluginFastCCDDim2MinString,           asynParamInt32, &NDPluginFastCCDDim2Min);
    createParam(NDPluginFastCCDDim0SizeString,          asynParamInt32, &NDPluginFastCCDDim0Size);
    createParam(NDPluginFastCCDDim1SizeString,          asynParamInt32, &NDPluginFastCCDDim1Size);
    createParam(NDPluginFastCCDDim2SizeString,          asynParamInt32, &NDPluginFastCCDDim2Size);
    createParam(NDPluginFastCCDDim0MaxSizeString,       asynParamInt32, &NDPluginFastCCDDim0MaxSize);
    createParam(NDPluginFastCCDDim1MaxSizeString,       asynParamInt32, &NDPluginFastCCDDim1MaxSize);
    createParam(NDPluginFastCCDDim2MaxSizeString,       asynParamInt32, &NDPluginFastCCDDim2MaxSize);
    createParam(NDPluginFastCCDDim0BinString,           asynParamInt32, &NDPluginFastCCDDim0Bin);
    createParam(NDPluginFastCCDDim1BinString,           asynParamInt32, &NDPluginFastCCDDim1Bin);
    createParam(NDPluginFastCCDDim2BinString,           asynParamInt32, &NDPluginFastCCDDim2Bin);
    createParam(NDPluginFastCCDDim0ReverseString,       asynParamInt32, &NDPluginFastCCDDim0Reverse);
    createParam(NDPluginFastCCDDim1ReverseString,       asynParamInt32, &NDPluginFastCCDDim1Reverse);
    createParam(NDPluginFastCCDDim2ReverseString,       asynParamInt32, &NDPluginFastCCDDim2Reverse);
    createParam(NDPluginFastCCDDim0EnableString,        asynParamInt32, &NDPluginFastCCDDim0Enable);
    createParam(NDPluginFastCCDDim1EnableString,        asynParamInt32, &NDPluginFastCCDDim1Enable);
    createParam(NDPluginFastCCDDim2EnableString,        asynParamInt32, &NDPluginFastCCDDim2Enable);
    createParam(NDPluginFastCCDDim0AutoSizeString,      asynParamInt32, &NDPluginFastCCDDim0AutoSize);
    createParam(NDPluginFastCCDDim1AutoSizeString,      asynParamInt32, &NDPluginFastCCDDim1AutoSize);
    createParam(NDPluginFastCCDDim2AutoSizeString,      asynParamInt32, &NDPluginFastCCDDim2AutoSize);
    createParam(NDPluginFastCCDDataTypeString,          asynParamInt32, &NDPluginFastCCDDataType);
    createParam(NDPluginFastCCDEnableScaleString,       asynParamInt32, &NDPluginFastCCDEnableScale);
    createParam(NDPluginFastCCDScaleString,             asynParamFloat64, &NDPluginFastCCDScale);
    createParam(NDPluginFastCCDCollapseDimsString,      asynParamInt32, &NDPluginFastCCDCollapseDims);

    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "NDPluginFastCCD");

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
