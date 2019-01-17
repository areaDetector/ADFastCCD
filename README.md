areaDetector driver for FastCCD
===============================

An 
[EPICS](http://www.aps.anl.gov/epics/) 
[areaDetector](http://cars.uchicago.edu/software/epics/areaDetector.html)
driver for the 
[LBNL FastCCD](https://sites.google.com/a/lbl.gov/fast-ccd-camera-systems)

This driver is supported under linux and uses the 
[libcin](https://github.com/NSLS-II-CSX/libcin)
driver developed by Stuart Wilkins at the 
[NSLS-II](https://www.bnl.gov/ps). This library acts as the interface between
the UDP controlled _Camera Interface Node_ and the areaDetector driver. Please
see the `libcin`
[readme](https://github.com/NSLS-II/libcin/blob/master/README.md) for
information on the required network setup. 

Additional information:
* [Documentation](https://areaDetector.github.io/ADFastCCD/index.html)
* [Release notes and links to source and binary releases](RELEASE.md).
