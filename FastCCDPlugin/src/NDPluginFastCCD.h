#ifndef NDPluginFastCCD_H
#define NDPluginFastCCD_H

#include "NDPluginDriver.h"

/* ROI general parameters */
#define NDPluginFastCCDNameString               "NAME"                /* (asynOctet,   r/w) Name of this ROI */

/* ROI definition */
#define NDPluginFastCCDDim0MinString            "DIM0_MIN"          /* (asynInt32,   r/w) Starting element of ROI in each dimension */
#define NDPluginFastCCDDim1MinString            "DIM1_MIN"          /* (asynInt32,   r/w) Starting element of ROI in each dimension */
#define NDPluginFastCCDDim2MinString            "DIM2_MIN"          /* (asynInt32,   r/w) Starting element of ROI in each dimension */
#define NDPluginFastCCDDim0SizeString           "DIM0_SIZE"         /* (asynInt32,   r/w) Size of ROI in each dimension */
#define NDPluginFastCCDDim1SizeString           "DIM1_SIZE"         /* (asynInt32,   r/w) Size of ROI in each dimension */
#define NDPluginFastCCDDim2SizeString           "DIM2_SIZE"         /* (asynInt32,   r/w) Size of ROI in each dimension */
#define NDPluginFastCCDDim0MaxSizeString        "DIM0_MAX_SIZE"     /* (asynInt32,   r/o) Maximum size of ROI in each dimension */
#define NDPluginFastCCDDim1MaxSizeString        "DIM1_MAX_SIZE"     /* (asynInt32,   r/o) Maximum size of ROI in each dimension */
#define NDPluginFastCCDDim2MaxSizeString        "DIM2_MAX_SIZE"     /* (asynInt32,   r/o) Maximum size of ROI in each dimension */
#define NDPluginFastCCDDim0BinString            "DIM0_BIN"          /* (asynInt32,   r/w) Binning of ROI in each dimension */
#define NDPluginFastCCDDim1BinString            "DIM1_BIN"          /* (asynInt32,   r/w) Binning of ROI in each dimension */
#define NDPluginFastCCDDim2BinString            "DIM2_BIN"          /* (asynInt32,   r/w) Binning of ROI in each dimension */
#define NDPluginFastCCDDim0ReverseString        "DIM0_REVERSE"      /* (asynInt32,   r/w) Reversal of ROI in each dimension */
#define NDPluginFastCCDDim1ReverseString        "DIM1_REVERSE"      /* (asynInt32,   r/w) Reversal of ROI in each dimension */
#define NDPluginFastCCDDim2ReverseString        "DIM2_REVERSE"      /* (asynInt32,   r/w) Reversal of ROI in each dimension */
#define NDPluginFastCCDDim0EnableString         "DIM0_ENABLE"       /* (asynInt32,   r/w) If set then do ROI in this dimension */
#define NDPluginFastCCDDim1EnableString         "DIM1_ENABLE"       /* (asynInt32,   r/w) If set then do ROI in this dimension */
#define NDPluginFastCCDDim2EnableString         "DIM2_ENABLE"       /* (asynInt32,   r/w) If set then do ROI in this dimension */
#define NDPluginFastCCDDim0AutoSizeString       "DIM0_AUTO_SIZE"    /* (asynInt32,   r/w) Automatically set size to max */
#define NDPluginFastCCDDim1AutoSizeString       "DIM1_AUTO_SIZE"    /* (asynInt32,   r/w) Automatically  set size to max */
#define NDPluginFastCCDDim2AutoSizeString       "DIM2_AUTO_SIZE"    /* (asynInt32,   r/w) Automatically  set size to max */
#define NDPluginFastCCDDataTypeString           "ROI_DATA_TYPE"     /* (asynInt32,   r/w) Data type for ROI.  -1 means automatic. */
#define NDPluginFastCCDEnableScaleString        "ENABLE_SCALE"      /* (asynInt32,   r/w) Disable/Enable scaling */
#define NDPluginFastCCDScaleString              "SCALE_VALUE"       /* (asynFloat64, r/w) Scaling value, used as divisor */
#define NDPluginFastCCDCollapseDimsString       "COLLAPSE_DIMS"     /* (asynInt32,   r/w) Collapse dimensions of size 1 */

/** Extract Regions-Of-Interest (ROI) from NDArray data; the plugin can be a source of NDArray callbacks for
  * other plugins, passing these sub-arrays. 
  * The plugin also optionally computes a statistics on the ROI. */
class epicsShareClass NDPluginFastCCD : public NDPluginDriver {
public:
    NDPluginFastCCD(const char *portName, int queueSize, int blockingCallbacks, 
                 const char *NDArrayPort, int NDArrayAddr,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize, int maxThreads);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

protected:
    /* ROI general parameters */
    int NDPluginFastCCDName;
    #define FIRST_NDPLUGIN_ROI_PARAM NDPluginFastCCDName

    /* ROI definition */
    int NDPluginFastCCDDim0Min;
    int NDPluginFastCCDDim1Min;
    int NDPluginFastCCDDim2Min;
    int NDPluginFastCCDDim0Size;
    int NDPluginFastCCDDim1Size;
    int NDPluginFastCCDDim2Size;
    int NDPluginFastCCDDim0MaxSize;
    int NDPluginFastCCDDim1MaxSize;
    int NDPluginFastCCDDim2MaxSize;
    int NDPluginFastCCDDim0Bin;
    int NDPluginFastCCDDim1Bin;
    int NDPluginFastCCDDim2Bin;
    int NDPluginFastCCDDim0Reverse;
    int NDPluginFastCCDDim1Reverse;
    int NDPluginFastCCDDim2Reverse;
    int NDPluginFastCCDDim0Enable;
    int NDPluginFastCCDDim1Enable;    
    int NDPluginFastCCDDim2Enable;    
    int NDPluginFastCCDDim0AutoSize;    
    int NDPluginFastCCDDim1AutoSize;    
    int NDPluginFastCCDDim2AutoSize;    
    int NDPluginFastCCDDataType;
    int NDPluginFastCCDEnableScale;
    int NDPluginFastCCDScale;
    int NDPluginFastCCDCollapseDims;

private:
    int requestedSize_[3];
    int requestedOffset_[3];
};
    
#endif
