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

#include <epicsString.h>
#include <epicsMutex.h>
#include <iocsh.h>

#include <asynDriver.h>

#include <epicsExport.h>
#include "NDPluginDriver.h"
#include "NDPluginFastCCD.h"

#include "cin.h"

static const char *driverName="NDPluginFastCCD";

/** Callback function that is called by the NDArray driver with new NDArray data.
  * Draws overlays on top of the array.
  * \param[in] pArray  The NDArray from the callback.
  */
void NDPluginFastCCD::processCallbacks(NDArray *pArray)
{
    /* This function does the actual operations
     * It is called with the mutex already locked.  It unlocks it during long calculations when private
     * structures don't need to be protected.
     */

    NDArray *pOutput;

    /* Call the base class method */
    NDPluginDriver::processCallbacks(pArray);

    /* We always keep the last array so read() can use it.
     * Release previous one. */
    if (this->pArrays[0]) {
        this->pArrays[0]->release();
    }

    /* Copy the input array format but not the data */
    this->pArrays[0] = this->pNDArrayPool->copy(pArray, NULL, 1);
    pOutput = this->pArrays[0];
    
    /* Get information about the array needed later */
    pOutput->getInfo(&this->arrayInfo);

    /* Get paraemeters which we need for processing the image */
    int offset0, offset1, offset2, enabled, dpval, dataen;
    getIntegerParam(NDPluginFastCCDOffset0, &offset0);
    getIntegerParam(NDPluginFastCCDOffset1, &offset1);
    getIntegerParam(NDPluginFastCCDOffset2, &offset2);
    getIntegerParam(NDPluginFastCCDEnable, &enabled);
    getIntegerParam(NDPluginFastCCDEnableData, &dataen);
    getIntegerParam(NDPluginFastCCDDPVal, &dpval);

    /* This function is called with the lock taken, and it must be set when we exit.
     * The following code can be exected without the mutex because we are not accessing memory
     * that other threads can access. */
    this->unlock();

    if((pOutput->dataType == NDUInt16) && enabled){
      // We can only process NDUnit16 images just pass if not. 

      // Get the data pointer
      epicsUInt16 *data = (epicsUInt16 *)pOutput->pData;
      epicsUInt16 ctrl;

      for(size_t i=0; i<pOutput->dims[1].size; i++){
        for(size_t j=0; j<pOutput->dims[0].size; j++){
          ctrl = *data & 0xE000;
          if(dataen){
            *data = *data & ~0xE000;
            if((ctrl & 0xC000) == 0xC000){
              // Minimum Gain
              *data = 0x1000 + 8 * (*data - (epicsUInt16)offset2);
            } else if ((ctrl & 0x4000) == 0x4000) {
              // Gain
              *data = 0x1000 + 4 * (*data - (epicsUInt16)offset1);
            } else if ((ctrl & 0x2000) == 0x2000) {
              // Dropped Packet
              *data = (epicsUInt16)dpval;
            } else {
              // Maximum gain
              *data = 0x1000 + *data - (epicsUInt16)offset0;
            } 
          } else {
            *data = ctrl;
          }
          data++; // Advance the pointer
        }
      }
    }

    this->lock();

    /* Get the attributes for this driver */
    this->getAttributes(this->pArrays[0]->pAttributeList);
    /* Call any clients who have registered for NDArray callbacks */
    this->unlock();
    doCallbacksGenericPointer(this->pArrays[0], NDArrayData, 0);
    this->lock();
    callParamCallbacks();
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
    static const char *functionName = "NDPluginFastCCD";


    createParam(NDPluginFastCCDOffset0String,    asynParamInt32,  &NDPluginFastCCDOffset0);
    createParam(NDPluginFastCCDOffset1String,    asynParamInt32,  &NDPluginFastCCDOffset1);
    createParam(NDPluginFastCCDOffset2String,    asynParamInt32,  &NDPluginFastCCDOffset2);
    createParam(NDPluginFastCCDDPValString,      asynParamInt32,  &NDPluginFastCCDDPVal);
    createParam(NDPluginFastCCDEnableString,     asynParamInt32,  &NDPluginFastCCDEnable);
    createParam(NDPluginFastCCDEnableDataString, asynParamInt32,  &NDPluginFastCCDEnableData);

    int offset = 0x1000;

    setIntegerParam(NDPluginFastCCDOffset0, offset);
    setIntegerParam(NDPluginFastCCDOffset1, offset);
    setIntegerParam(NDPluginFastCCDOffset2, offset);
    setIntegerParam(NDPluginFastCCDDPVal, offset);
    setIntegerParam(NDPluginFastCCDEnable, 0);
    setIntegerParam(NDPluginFastCCDEnableData, 0);

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
