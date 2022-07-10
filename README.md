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

#### Author
Matthrew Kelsey(original), Nick Zhang(current maintainer)

