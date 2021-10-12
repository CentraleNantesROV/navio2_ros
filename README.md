## ROS 2 package for low-level communication with Navio2 board.

This packages exposes capabilities of Navio2 over ROS 2. They are adapted from the examples in the `Navio2/C++` sub-module. Clone with `--recurse-submodules` to get it from Emlid.

Current development includes:

    - AHRS (`imu` param should be `mpu` or `lsm`)
    - ADC measurements
    - Barometer measurements (temperature / pressure)
    - LED control
    - PWM control
        
A PWM example is provided in `pwm_example.cpp`. The `PWM_Base` node can be inherited from in order to define your own subscription topics. In the example, a single topic uses the custom `PWM` message but in practice PWM should receive messages with more semantics: servo angle, propeller/thruster velocity, light intensities, etc.

## Setting up ROS 2 for Navio2

While Emlid only support their own Raspbian image, it is possible to install a raw Ubuntu server on the Raspberry Pi and setup the necessary steps to connect to the Navio2 hat.

A corresponding discussion can be found on [Emlid's forum](https://community.emlid.com/t/navio2-support-for-ubuntu-20-04/19956). Especially:

    - [pigpio](https://github.com/joan2937/pigpio) has to be installed by hand 
    - The `/boot/firmware/usercfg.txt` has to be tuned
    - most importantly, [Navio2 overlays](https://community.emlid.com/t/navio2-support-for-ubuntu-20-04/19956/18) have to be decompiled on Emlid's Raspbian and recompiled on the Ubuntu Pi

Setting up the Ubuntu / Navio2 softwares is on the roadmap of the `navio2_ros` package.
