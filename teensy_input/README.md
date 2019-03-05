# modules 

## Set up

![alt text](https://forum.pjrc.com/attachment.php?attachmentid=8070&d=1473225638)

### platformio

> Couldn't get vscode to work

> Get CLion to work: https://docs.platformio.org/en/latest/ide/clion.html

#### Command LIst
|command |purpose| comments|
|---|---|---|
|platformio device monitor -b 115200| defaut is 960|With IMU reading, it reads continuously. But the serial monitor on Clion has the problem with ports everytime reuploads|

## INPUT

### Receiver

[SBUS reader](https://github.com/bolderflight/SBUS)

### IMU

[LSM9DS1](https://github.com/sparkfun/LSM9DS1_Breakout)
Note that the SCL port is on 19, SDA is 20 https://www.pjrc.com/teensy/td_libs_Wire.html

## OUTPUT

### ESC 

