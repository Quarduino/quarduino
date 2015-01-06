quarduino
=========

This is the software for an attempt to build a functioning quadrocopter as a school project. The project is maintained by three friends from Herrgårdsgymnasiet, Säffle, Sweden.

![The "quarduino"](http://i.imgur.com/919ByQY.jpg?1)

The software is basically:
* Receiving input from an RX-transmitter
* Receiving data from a ADXL345 accelerometer and a ITG3200 gyro
* Calculating speed for each motor based on the data received above to stabilize and maintain a desired tilt
* Sending the speed to a speed controller connected to four motors

As you might see, we are actually using two arduinos, an Arduino UNO and a DUE. The reason for that is because we couldn't get the gyro/accelerometer library to work on our DUE (the existing Wire library isn't really compatible with it) so we decided to get data from the gyro and accelerometer using an UNO and transfer it to a DUE using serial for computation. Apparently, the problem with Wire.h on the DUE can be resolved but this approach works for us, so we'll continue with it.

We're currently at the stabilization stage and have no more than a month to complete it. Info about the exact hardware we're using including CAD projects for 3D-printing will be available soon.
