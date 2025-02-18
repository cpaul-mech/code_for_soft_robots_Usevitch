# Code for Soft Robotics Usevitch

This repository holds the code for the soft robotics project for the moon advised by Dr. Nathan Usevitch at BYU. 

## Guide to the Folders Above

- **[Code for Arduinos](/Revised_Arduino_Code)**: This folder holds updated code for controlling the motors, as stated below. The major differences lie in the structure of the code and in the steps required to upload the code to an Arduino. For more information, see [readme](/Revised_Arduino_Code/README.md).
- **[DEPRECATED - Code For Arduinos](/Code_for_Arduinos)**: **DEPRECATED** This folder holds old the code for the Arduinos that will be used in the project. The arduinos are being used to control the [motors](https://www.servocity.com/60-rpm-hd-premium-planetary-gear-motor-w-encoder/) and run the [radio](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/#:~:text=nRF24L01%20Transceiver%20Module,-Let's%20take%20a&text=It%20uses%20the%202.4%20GHz,2.4%20%E2%80%93%202.5GHz%20ISM%20band) communication network. (see the README.md in the sub-folder for more information) This folder also contains the code with which the arduinos will communicate with the MATLAB dynamics simulation.
- **[Code for Raspberry Pi](/Code_for_RPi)**: This folder holds the code and setup tutorials for the Raspberry Pi that can be used as the interface between the RF24 network and MATLAB. The RPi currently is configured to run a TCP server and a single RF24 module that sends out the commands from a MATLAB script.
- **[MATLAB Dynamics Simulation Code](/MATLAB_dynamics_code):** This folder holds the files written by Dr. Usevitch and Isaac Weaver that are used to simulate the dynamics of our soft robot in MATLAB.
- **[Python Dynamics Simulation Code](/Python_dynamics_code/):** This folder holds the python dynamics and simulation files written by Spencer Stowell and Annie O'Bryan to simulate various triangle and truss configurations of the soft robot.
