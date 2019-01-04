<h1 align="center"> areaDetector FastCCD driver </h1>
<h2 align="center"> December 20th, 2018 </h2> 
<h2 align="center"> Stuart B. Wilkins </h2>
<h2 align="center"> NSLS-II, Brookhaven National Laboratory </h2>

Contents
--------

-   [Overview](#overview)
-   [Implementation of standard driver parameters](#implementation-of-standard-driver-parameters)
-   [FastCCD specific parameters](#fastccd-specific-parameters)
-   [Configuration](#configuration)
-   [Setting the image size](#setting-the-image-size)
-   [Auto Configuration of the FastCCD](#auto-configuration-of-the-fastccd)
-   [MEDM screens](#medm-screens)
-   [Connection management](#connection-management)

Overview
--------

This is an [EPICS](http://www.aps.anl.gov/epics/)
[areaDetector](areaDetector.html) driver for the FastCCD detector
produced by the detector group at Lawrence Berkeley National Laboratory.
The driver is supported under Linux. The driver acts as a high level
driver over a lower level c-driver which communicates with the camera
via UDP sockets. The lower level c driver, *libcin* can be found [on
github here](https://github.com/NSLS-II/libcin).

This driver inherits from [ADDriver](areaDetectorDoc.html#ADDriver). It
implements nearly all of the parameters in
[asynNDArrayDriver.h](
http://cars9.uchicago.edu/software/epics/areaDetectorDoxygenHTML/asyn_n_d_array_driver_8h.html)
and in [ADArrayDriver.h](
http://cars9.uchicago.edu/software/epics/areaDetectorDoxygenHTML/_a_d_driver_8h.html).
It also implements a number of parameters that are specific to the
FastCCD camera and can controll the boot-up sequence of the camera amd
all settings such as bias voltages and timing.

Implementation of standard driver parameters
--------------------------------------------

The driver redefines the choices for several of the parameters defined
in ADDriver.h. The ADTriggerMode choices for the FastCCD are:

-   Internal (Software trigger)
-   External 1 (external trigger on input 1)
-   External 2 (external trigger on input 2)
-   External 1+2 (external trigger on input 1 OR input 2)

The NDDataType choices for the FastCCD are:

-   NDUInt16 (16 bit data)

The NDColorMode choices for the FastCCD are:

-   NDColorModeMono (monochromatic data)

FastCCD specific parameters
---------------------------

The FastCCD driver implements the following parameters in addition to
those in asynNDArrayDriver.h and ADDriver.h:

<table border="1" cellpadding="2" cellspacing="2" style="text-align: left">

<tr>
<td colspan=7 align="center">
<b>
Parameter Definitions in FastCCD.cpp and EPICS Record Definitions in FastCCD.template
</b>
</td>
</tr>

<tr>
<td>Parameter index variable</td>
<td>asyn interface</td>
<td>Access</td>
<td>Description</td>
<td>drvInfo string</td>
<td>EPICS record name</td>
<td>EPICS record type</td>
<tr>

<tr>
<td colspan=7 align="center">
<b>
FastCCD Image Size and Overscan Settings
</b>
</td>
</tr>

<tr>
<td>FastCCDOverscanRows</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Specify the number of overscan rows to use when descrambling</td>
<td>OVERSCAN_ROWS</td>
<td>\$(P)\$(R)OverscanRows<br />\$(P)\$(R)OverscanRows_RBV</td>
<td>longout<br />longin</td>
<tr>

<tr>
<td>FastCCDOverscanCols</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Specify the number of overscan columns to use when descrambling</td>
<td>OVERSCAN_COLS</td>
<td>\$(P)\$(R)OverscanCols<br />\$(P)\$(R)OverscanCols_RBV</td>
<td>longout<br />longin</td>
<tr>

<tr>
<td colspan=7 align="center">
<b>
FastCCD fCRIC and CIN Settings 
</b>
</td>
</tr>

<tr>
<td>FastCCDMux1</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Set the output muxer on the CIN to monitor timing signals. Allowed values are:<br />
EXPOSE<br />
VCLK1<br />
VCLK2<br />
VCLK3<br />
ATG<br />
VFSCLK1<br />
VFSCLK2<br />
VFSCLK3<br />
HCLK1<br />
HCLK2<br />
OSW<br />
RST<br />
CONVERT<br />
SHUTTER<br />
SWTRIGGER<br />
TRIGMON</td>
<td>FCCD_MUX1</td>
<td>\$(P)\$(R)Mux1<br />\$(P)\$(R)Mux1_RBV</td>
<td>mbbo<br />mbbi</td>
</tr>

<tr>
<td>FastCCDMux2</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Set the output muxer on the CIN to monitor timing signals. Allowed values are:<br />
EXPOSE<br />
VCLK1<br />
VCLK2<br />
VCLK3<br />
ATG<br />
VFSCLK1<br />
VFSCLK2<br />
VFSCLK3<br />
HCLK1<br />
HCLK2<br />
HCLK3<br />
OSW<br />
RST<br />
CONVERT<br />
SAVE<br />
HWTRIG</td>
<td>FCCD_MUX2</td>
<td>\$(P)\$(R)Mux2<br />\$(P)\$(R)Mux2_RBV</td>
<td>mbbo<br />mbbi</td>
</tr>

<tr>
<td>FastCCDFCRICGain</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Sets the fCRIC gain. Allowed values are:<br />
Auto<br />
x2<br />
x1</td>
<td>FCRIC_GAIN</td>
<td>\$(P)\$(R)FCRICGain<br />\$(P)\$(R)FCRICGain_RBV</td>
<td>mbbo<br />mbbi</td>
</tr>

<tr>
<td>FastCCDFCRICClamp</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Sets the fCRIC clamp mode. Allowed values are:<br />
CLAMP OFF<br />
CLAMP ON</td>
<td>FCRIC_CLAMP</td>
<td>\$(P)\$(R)FCRICClamp<br />\$(P)\$(R)FCRICClamp_RBV</td>
<td>bo<br />bi</td>
</tr>

<tr>
<td>FastCCDFOTest</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Sets the fiberoptic module test mode. Allowed values are:<br />
Off<br />
On</td>
<td>FO_TEST</td>
<td>\$(P)\$(R)FOTest<br />\$(P)\$(R)FOTest_RBV</td>
<td>bo<br />bi</td>
</tr>

<tr>
<td colspan=7 align="center">
<b>
FastCCD CIN Setting Upload
</b>
</td>
</tr>

<tr>
<td>FastCCDFirmwarePath</td>
<td>asynParamOctet</td>
<td>r/w</td>
<td>Sets the path for the CIN firmware file to upload</td>
<td>FIRMWARE_PATH</td>
<td>\$(P)\$(R)FirmwarePath<br />\$(P)\$(R)FirmwarePath_RBV</td>
<td>waveform</td>
</tr>

<tr>
<td>FastCCDFirmwareUpload</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Uploads the firmware to the CIN specified in the _FastCCDFirmwarePath_</td>
<td>FIRMWARE_UPLOAD</td>
<td>\$(P)\$(R)FwUpload<br />\$(P)\$(R)FwUpload_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td>FastCCDClockPath</td>
<td>asynParamOctet</td>
<td>r/w</td>
<td>Sets the path for the clocking (timing) file to upload</td>
<td>CLOCK_PATH</td>
<td>\$(P)\$(R)ClockPath<br />\$(P)\$(R)ClockPath_RBV</td>
<td>waveform</td>
</tr>

<tr>
<td>FastCCDClockUpload</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Uploads the clock (timing) to the CIN specified in the _FastCCDClockPath_</td>
<td>CLOCK_UPLOAD</td>
<td>\$(P)\$(R)ClockUpload<br />\$(P)\$(R)ClockUpload_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td>FastCCDFCRICPath</td>
<td>asynParamOctet</td>
<td>r/w</td>
<td>Sets the path for the fCRIC configuration file to upload</td>
<td>FCRIC_PATH</td>
<td>\$(P)\$(R)FCRICPath<br />\$(P)\$(R)FCRICPath_RBV</td>
<td>waveform</td>
</tr>

<tr>
<td>FastCCDFCRICUpload</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Uploads the fCRIC configuration to the CIN specified in the _FastCCDFCRICPath_</td>
<td>FCRIC_UPLOAD</td>
<td>\$(P)\$(R)FCRICUpload<br />\$(P)\$(R)FCRICUpload_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td>FastCCDBiasPath</td>
<td>asynParamOctet</td>
<td>r/w</td>
<td>Sets the path for the sensor bias configuration file to upload</td>
<td>BIAS_PATH</td>
<td>\$(P)\$(R)BiasPath<br />\$(P)\$(R)BiasPath_RBV</td>
<td>waveform</td>
</tr>

<tr>
<td>FastCCDBiasUpload</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Uploads the bias configuration to the CIN specified in the _FastCCDBiasPath_</td>
<td>BIAS_UPLOAD</td>
<td>\$(P)\$(R)BiasUpload<br />\$(P)\$(R)BiasUpload_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td colspan=7 align="center">
<b>
FastCCD Auto Setting Parameters
</b>
</td>
</tr>

<tr>
<td>FastCCDTimingMode</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Set the timing mode to use for auto booting the FastCCD</td>
<td>TIMING_MODE</td>
<td>\$(P)\$(R)TimingMode<br />\$(P)\$(R)TimingMode_RBV</td>
<td>mbbo<br />mbbi</td>
</tr>

<tr>
<td>FastCCDTimingName</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode last set by the FastCCD auto boot sequence.</td>
<td>TIMING_NAME</td>
<td>\$(P)\$(R)TimingName_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName0</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 1</td>
<td>TIMING_NAME_0</td>
<td>\$(P)\$(R)TimingName1_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName1</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 2</td>
<td>TIMING_NAME_1</td>
<td>\$(P)\$(R)TimingName1_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName2</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 3</td>
<td>TIMING_NAME_2</td>
<td>\$(P)\$(R)TimingName1_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName3</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 4</td>
<td>TIMING_NAME_3</td>
<td>\$(P)\$(R)TimingName3_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName4</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 5</td>
<td>TIMING_NAME_4</td>
<td>\$(P)\$(R)TimingName4_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName5</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 6</td>
<td>TIMING_NAME_5</td>
<td>\$(P)\$(R)TimingName5_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName6</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 7</td>
<td>TIMING_NAME_6</td>
<td>\$(P)\$(R)TimingName6_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName7</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 8</td>
<td>TIMING_NAME_7</td>
<td>\$(P)\$(R)TimingName7_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName8</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 9</td>
<td>TIMING_NAME_8</td>
<td>\$(P)\$(R)TimingName8_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td>FastCCDTimingName9</td>
<td>asynParamOctet</td>
<td>ro</td>
<td>The name of the timing mode assigned to mode number 10</td>
<td>TIMING_NAME_9</td>
<td>\$(P)\$(R)TimingName9_RBV</td>
<td>stringin</td>
</tr>

<tr>
<td colspan=7 align="center">
<b>
FastCCD Auto Setting Commands
</b>
</td>
</tr>

<tr>
<td>FastCCDBoot</td>
<td>asynParamInt32</td>
<td>r/w</td> 
<td>Boot the CIN using the timing mode defined by
_FastCCDTimingMode_. This causes the CIN to be reset (power cycled), the
firmware to be uploaded and the timing to be uploaeded to the CIN.</td>
<td>BOOT</td>
<td>\$(P)\$(R)Boot<br />\$(P)\$(R)Boot_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td>FastCCDSendTiming</td>
<td>asynParamInt32</td>
<td>r/w</td> 
<td>Send the timing information to the CIN as set by 
_FastCCDTimingMode_. This causes the CIN timing to change.</td>
<td>SEND_TIMING</td>
<td>\$(P)\$(R)SendTiming<br />\$(P)\$(R)SendTiming_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td>FastCCDSendFCRIC</td>
<td>asynParamInt32</td>
<td>r/w</td> 
<td>Send the fCRIC configuration information to the CIN as set by 
_FastCCDTimingMode_.</td>
<td>SEND_FCRIC</td>
<td>\$(P)\$(R)SendFCRIC<br />\$(P)\$(R)SendFCRIC_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td>FastCCDSendBias</td>
<td>asynParamInt32</td>
<td>r/w</td> 
<td>Send the bias voltage configuration information to the CIN as set by 
_FastCCDTimingMode_.</td>
<td>SEND_BIAS</td>
<td>\$(P)\$(R)SendBias<br />\$(P)\$(R)SendBias_RBV</td>
<td>busy<br />bi</td>
</tr>

<tr>
<td colspan=7 align="center">
<b>
FastCCD CIN Power Settings
</b>
</td>
</tr>

<tr>
<td>FastCCDPower</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Powers on and off the CIN (0 = off, 1 = on)</td>
<td>CIN_POWER</td>
<td>\$(P)\$(R)Power<br />\$(P)\$(R)Power_RBV</td>
<td>bo<br />bi</td>
</tr>

<tr>
<td>FastCCDFPPower</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Powers on and off the CIN front panel (0 = off, 1 = on)</td>
<td>CIN_POWER</td>
<td>\$(P)\$(R)FPPower<br />\$(P)\$(R)FPPower_RBV</td>
<td>bo<br />bi</td>
</tr>

</table>

Configuration
-------------

The FastCCD driver is created with the FastCCDConfig command, either
from C/C++ or from the EPICS IOC shell.

    int FastCCDConfig(char *portName, 
					  int maxBuffers, size_t maxMemory,
                      int priority, int stackSize,
				      int packetBuffer, int imageBuffer,
                      const char *baseIP, const char *fabricIP, 
                      const char *fabricMAC)

The **packetBuffer** and **imageBuffer** parameters specify how many 
packet frames and images to buffer in the `libcin` driver. These are 
directly passed to `libcin` on initialization. 

The **baseIP** and **fabricIP** are the ip addresses of the base and fabric
interfaces on the camera interface node (CIN). A null \"\" string sets the 
default as defined in the `libcin` driver.

The **fabricMAC** parameter causes the `libcin` driver to set the MAC 
address of the CIN on initialization. This can be used to broadcast the
image data by using the `FF:FF:FF:FF:FF` mac address. 

For details on the meaning of the other parameters to this function
refer to the detailed documentation on the FastCCDConfig function in
the 
[FastCCD.cpp documentation](
http://cars9.uchicago.edu/software/epics/areaDetectorDoxygenHTML/_fast_c_c_d_8cpp.html)
 and in the documentation for the constructor for the 
[FastCCD class](
http://cars9.uchicago.edu/software/epics/areaDetectorDoxygenHTML/class_fast_c_c_d.html).  

Setting the image size
----------------------

Due to the multicolum nature of the FastCCD, setting the image size is not
trivial. This is due to the nature of the image descrambling and if overscan is
used on each supercolumn. For this reason the image size is governed by two
paramaters, `$(P)$(R)OverscanCols` and `$(P)$(R)ADSizeY`.

The definition used for the image orientation in this driver is that columns
are along the _X_ direction and rows are along the _Y_ direction. As the number
of columns per _supercolumn_ is fixed in timing to 10 columns, the only free
parameter is the number of overscan cols to include and specified by the
`$(P)$(R)OverscanCols` PV. For this reason setting of the `$(P)$(R)ADSizeX` PV
is ignored by the driver.  The number of rows defines how many rows per column
to process and is set by the `$(P)$(R)ADSizeY` PV. 

The actual image size is therefore given by:

* _X_ Direction (row wise) `$(P)$(R)ADSizeY` [usually set to 2000 to have some overscan]
* _Y_ Direction (column wise) (10 + `$(P)$(R)OverscanCols`) * 96 [for no overscan 960]

Auto Configuration of the FastCCD
---------------------------------
The FastCCD can be configured by using the built in _firmware_ and settings in
the [libcin driver](https://github.com/NSLS-II/libcin). This is controled by the 
PVs defined in the section *FastCCD Auto Setting Commands*. The PVs 
`$(P)$(R)TimingName[0..9]_RBV` show strings with names defined in the `libcin` 
driver corresponding to the modes 1 through 10. To boot into one of these modes, 
the user should set the PV `$(P)$(R)TimingMode` to number of the mode required
(0 through 9). The PV `$(P)$(R)TimingName_RBV` will indicate the mode selected. 

To boot the FastCCD, the user should write 1 to the PV `$(P)$(R)Boot`. This will
cause the FastCCD to reset the CIN (by power cycling it) and upload the firmware 
and timing configuration. At this point the user should be able to see the test
pattern from the CIN by triggering the detector. The user can then configure the
_fCRICs_ by writing 1 to the PV `$(P)$(R)SendFCRIC`. The bias configuration can 
be upload by writing 1 to the PV `$(P)$(R)SendBias`.  The latter two steps require
the camera head to be powered. This is out of the scope of this driver. 

MEDM screens
------------

MEDM screens are not used at NSLS-II as CSS-BOY is the default display manager

Connection management
---------------------

As the FastCCD _CIN_ only uses UDP sockets, there is no permanent connection,
the driver relies on being able to read such parameters as the serial nnumber
of the software to be able to confirm that commuincation with the driver is
occuring.

