# Repo Description
This project contains a breakout board for the Diodes Inc. PI4IOE5V6416 16 channel I2C I/O expander, as well as a C driver, for easier incorporation into your projects.

[The datasheet can be found here](https://www.diodes.com/assets/Datasheets/PI4IOE5V6416.pdf).

# Hardware
The ECad tool of choice is KiCad for this project. 

Manufacturing files for revision 3.0 are available in the **/Manufacturing** directory.

Photos of the board are available in **/photos**

The schematic is available in the root directory as a pdf. 

# Driver \(C\)
The C driver is designed to be implementation-agnostic. I recommend reading through the header files to gain a better understanding of the driver itself, and what's available. Regardless, the driver works off of a handle principle:

You create a device handle that is populated with call-backs to all the different functions needed to communicate with the device (read / write). 

This handle is then passed to all the functions in the public API which read/write to the device.