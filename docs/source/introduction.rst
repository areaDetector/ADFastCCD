Introduction
============

This is an `EPICS <http://www.aps.anl.gov/epics/>`__
`areaDetector <areaDetector.html>`__ driver for the FastCCD detector
produced by the detector group at Lawrence Berkeley National Laboratory.
The driver is supported under Linux. The driver acts as a high level
driver over a lower level c-driver which communicates with the camera
via UDP sockets. The lower level c driver, *libcin* can be found `on
github here <https://github.com/NSLS-II/libcin>`__.

This driver inherits from `ADDriver <areaDetectorDoc.html#ADDriver>`__.
It implements nearly all of the parameters in
`asynNDArrayDriver.h <http://cars9.uchicago.edu/software/epics/areaDetectorDoxygenHTML/asyn_n_d_array_driver_8h.html>`__
and in
`ADArrayDriver.h <http://cars9.uchicago.edu/software/epics/areaDetectorDoxygenHTML/_a_d_driver_8h.html>`__.
It also implements a number of parameters that are specific to the
FastCCD camera and can controll the boot-up sequence of the camera amd
all settings such as bias voltages and timing.

.. image:: fccd1.png
    :width: 300px
    :align: center

