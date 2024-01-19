## Miniz-Board
This repository contains firmware and driver for the new offboard version of Georgia Tech's miniature racecar, BuzzRacer
The platform starts as a commercially available Kyosho Miniz MR-03 radio control car, with the control board replaced by a custom designed PCB board. The PCB board contains two IC motor controller for steering servo and drive motor. The board is controlled by an Arduino IoT Nano 33 board, which is stacked on top of the custom PCB. The project is done to enable control over the low-level onboard controller dynamics, previously implemented as a proprietary black-box circuit from Kyosho.

During normal operation, the onboard Arduino should be connected wirelessly to a local network, the SSID and password of the network should be hardcoded to the firmware source code. A PC computer running the driver should also be connected to the network. The computer should then be able to control the steering and throttle of the vehicle, as well as receing realtime sensor updates from the vehicle. 

The platform is capable of
 * Two way communication between computer and a platform
 * Multiple vehicle operation
 * Online update of vehicle parameters
 * Realtime sensor reading

This platform is currently used by the VIP program at Georgia Tech under supervision of Dr. Tsiotras. For more information on the program, please contact the authors

## Pictures
![Custom PCB](https://github.com/Nick-Zhang1996/miniz-board/blob/main/pics/p2.png)
![Platform with Arduino installed](https://github.com/Nick-Zhang1996/miniz-board/blob/main/pics/p1.png)

### TO-DO
Several things left undone:
Program
 * Use SAMD timer interrupt to handle PWM generation, to move PWM frequency to a higher inaudible range
 * Run PID control loop for steering servo at higher frequency

Circuit
 * Add sensor for motor current

#### Author
Matthrew Kelsey(original), Nick Zhang(current maintainer)

#### Tricks
  Because we use timer, sometimes our code interfere with booloader, and cause problem when uploading. If problem occurs, like If you get error 'no device found on /dev/ttyACM0',:

 * Double tap the reset button on Arduino Nano 33 IoT to force bootloader mode. 
 * Close the serial monitor
 * If upload fails, copy the command to upload, and run it in a terminal.


