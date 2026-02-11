# wavi
**W**hisker sensor **A**rray data acquisition and **Vi**sulization.

Note: this document might not be up to date with the code.

This folder contains an applet `wavi.m` that will launch a GUI for sensor array data collection. Some functions are in other folders of this repo, make sure to add those to your MATLAB path.

The applet can be launched by running the `wavi.m` file. However, parameters can only be specified through commandline. In the MATLAB command window, run, e.g.

```matlab
wavi(Fs=80,nsensor=9)
```

This will specify the sampling frequency (`Fs`) to be 80 Hz, and the number of sensors to be 9.

The applet only operates if the data acquisition hardware is connected.

This folder also contains the `wavi_driver.m`, which is the older script version. It might be useful for debugging.
