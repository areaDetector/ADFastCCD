<h1 align="center"> areaDetector FastCCD driver </h1>
<h2 align="center"> December 20th, 2018 </h2> 
<h2 align="center"> Stuart B. Wilkins </h2>
<h2 align="center"> NSLS-II, Brookhaven National Laboratory </h2>

Contents
--------

-   [Overview](#Overview)
-   [Implementation of standard driver parameters](#Implementation of standard driver parameters)
-   [Prosilica specific parameters](#Driver_parameters)
-   [Configuration](#Configuration)
-   [MEDM screens](#MEDM_screens)
-   [Connection management](#ConnectionManagement)

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

The NDDataType choices for the Prosilica are:

-   NDUInt16 (16 bit data)

The NDColorMode choices for the Prosilica are:

-   NDColorModeMono (monochromatic data)

FastCCD specific parameters
---------------------------

The FastCCD driver implements the following parameters in addition to
those in asynNDArrayDriver.h and ADDriver.h:




Configuration {#Configuration}
------------------------------

The Prosilica driver is created with the prosilicaConfig command, either
from C/C++ or from the EPICS IOC shell.

    int prosilicaConfig(char *portName,
                        const char* cameraId,
                        int maxBuffers, size_t maxMemory,
                        int priority, int stackSize, int maxPvAPIFrames)
      

The **cameraId** string can be any of the following:

-   The camera\'s UniqueId, which is a number assigned by the vendor to
    each Prosilica camera, e.g. 50110.
-   The camera\'s IP address, e.g. 164.54.160.48.
-   The camera\'s IP DNS name, e.g. gse-prosilica1.cars.aps.anl.gov.

Using the UniqueId has the advantage that the cameras can be configured
to use DHCP, and hence have non-predictable TCP/IP addresses. However,
if the UniqueId is used then the areaDetector IOC must be on the same
subnet as the camera, since cameras cannot be found by UniqueID through
routers. The simplest way to determine the uniqueId of a camera is to
run the Prosilica GigEViewer application, select the camera, and press
the \"i\" icon on the bottom of the main window to show the camera
information for this camera. The Unique ID will be displayed on the
first line in the information window.

The IP address or IP DNS name can be used for cameras with fixed IP
addresses, and **must** be used for cameras that are not on the local
subnet.

The maxPvAPIFrames parameter controls how many frame buffers will be
used by the PvAPI library. This is the last parameter in the
prosilicaConfig command, and if it is absent the default value of 2 is
used, which is sufficient in most circumstances. However, with very high
frame rates or busy IOCs increasing this value can reduce dropped
frames.

For details on the meaning of the other parameters to this function
refer to the detailed documentation on the prosilicaConfig function in
the [prosilica.cpp
documentation](areaDetectorDoxygenHTML/prosilica_8cpp.html) and in the
documentation for the constructor for the [prosilica
class](areaDetectorDoxygenHTML/classprosilica.html).

There an example IOC boot directory and startup script
([iocBoot/iocProsilica/st.cmd)](prosilica_st_cmd.html) provided with
areaDetector.

MEDM screens {#MEDM_screens style="text-align: left"}
-----------------------------------------------------

MEDM screens are not used at NSLS-II as CSS-BOY is the default display manager

Connection management {#ConnectionManagement}
---------------------------------------------

As the FastCCD _CIN_ only uses UDP sockets, there is no permanent connection,
the driver relies on being able to read such parameters as the serial nnumber
of the software to be able to confirm that commuincation with the driver is
occuring.

