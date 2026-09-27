# foc-simple-rtic
Field Oriented motor control with RTIC in Rust

# foc-simple-rtic

Library for controlling blcd motors with Field Oriented Control (foc).
With this library one can control blcd motors, with RTIC 2.x.

This library is a follow-up of https://crates.io/crates/foc-simple.


## Goals

* All goals from https://crates.io/crates/foc-simple
* But only for Rtic, Embassy dependencies are removed.
* Faster loop: at least at 25 Khz.
* No async used in the fast loop. RTIC locking mechanismes are used 
* (Rtic) async message queus used in the user interface
* 2 user interfaces are provided:
*   A basic command line interface in an serial terminal
*   A (very) basic VESC interface over an serial terminal.


The library does provide interfaces for:
* angle sensor
* current sensor
* serial communication
* pwm driver

The user of this library should implement these interfaces. The user will have to solve all the nitty gritty details for controlling the hardware.

In this way the library can stay very generic. 

This library provides:
* run the motor in angle mode. Set the angle in radians
* run the motor in torque mode. Set the torque with a value from 0..1
* run the motor in velocity mode. Set the speed in turns/sec
* Set the acceleration in velodity mode. Set the acceleration in turns/sec2
* Absolute maximum torque is set at startup of the code
* Torque limit can be set via the user interface
* A command line application with ahelp function

This library does NOT provide:
* current sensor implementation. Although current sense implementation can be done easy, as the library is already prepared for this. See the update function in FocPwm object.


# Layers

The library does contain the following layers
* The trait definititions to be implemented by the user
* The FOC control loop, which should run in the application, in an RTIC timer interrupt context, at at least 25 KHrz
* Command line user interface
* VESC examples
* Examples for controlling motors, with 2 different platforms (nucleo-f103rb and storm32). Examples are available in the github repository

# Calibration

Calibration should be done before each startup of the application.
Calibration parameters (direction and electrical offset) can be discovered automatically. or set in the application, if known.

The discoverd values can be observed in the RTT channel.

The command menu has an entry to set the calibration offset and the number of poles. Use these function with care, only for test cases. If set incorrectly it can damage the motor.

