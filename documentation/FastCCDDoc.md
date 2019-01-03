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
-   [MEDM screens](#medm-screens)
-   [Connection management](#connection-management)

Overview
--------

This is an [EPICS](http://www.aps.anl.gov/epics/)
[areaDetector](areaDetector.html) driver for the FastCCD detector
produced by the detector group at Lawrence Berkely National Laboratory.
The driver is supported under Linux. The driver acts as a high level
driver over a lower level c-driver which communicates with the camera
via UDP sockets. The lower level c driver, *libcin* can be found [on
github here](https://github.com/NSLS-II/libcin).

This driver inherits from [ADDriver](areaDetectorDoc.html#ADDriver). It
implements nearly all of the parameters in
[asynNDArrayDriver.h](areaDetectorDoxygenHTML/asyn_n_d_array_driver_8h.html)
and in [ADArrayDriver.h](areaDetectorDoxygenHTML/_a_d_driver_8h.html).
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
<td>$(P)$(R)OverscanRows<br />$(P)$(R)OverscanRows_RBV</td>
<td>longout<br />longin</td>
<tr>

<tr>
<td>FastCCDOverscanCols</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Specify the number of overscan columns to use when descrambling</td>
<td>OVERSCAN_COLS</td>
<td>$(P)$(R)OverscanCols<br />$(P)$(R)OverscanCols_RBV</td>
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
<td>$(P)$(R)Mux1<br />$(P)$(R)Mux1_RBV</td>
<td>mbbo<br />mbbi</td>
<tr>

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
<td>$(P)$(R)Mux2<br />$(P)$(R)Mux2_RBV</td>
<td>mbbo<br />mbbi</td>
<tr>

<tr>
<td>FastCCDFCRICGain</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Sets the fCRIC gain. Allowed values are:<br />
Auto<br />
x2<br />
x1</td>
<td>FCRIC_GAIN</td>
<td>$(P)$(R)FCRICGain<br />$(P)$(R)FCRICGain_RBV</td>
<td>mbbo<br />mbbi</td>
<tr>

<tr>
<td>FastCCDFCRICClamp</td>
<td>asynParamInt32</td>
<td>r/w</td>
<td>Sets the fCRIC clamp mode. Allowed values are:<br />
CLAMP OFF<br />
CLAMP ON</td>
<td>FCRIC_CLAMP</td>
<td>$(P)$(R)FCRICClamp<br />$(P)$(R)FCRICClamp_RBV</td>
<td>bo<br />bi</td>
<tr>

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
[FastCCD.cpp documentation](areaDetectorDoxygenHTML/_fast_c_c_d_8cpp.html)
 and in the documentation for the constructor for the 
[FastCCD class](areaDetectorDoxygenHTML/class_fast_c_c_d.html).  

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

MEDM screens
------------

MEDM screens are not used at NSLS-II as CSS-BOY is the default display manager

Connection management
---------------------

As the FastCCD _CIN_ only uses UDP sockets, there is no permanent connection,
the driver relies on being able to read such parameters as the serial nnumber
of the software to be able to confirm that commuincation with the driver is
occuring.

