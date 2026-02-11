# Gantry control

There are two parts in the gantry control code. The `clearcore` folder contains code for the ClearCore boards, which drives the ClearPath motors. Both the ClearCore boards and the ClearPath motors are from the company Teknic. The `matlab` folder contains code that runs on a PC to communicate with the ClearCore boards.

## Gantry system

The two carriages are driven by two ClearCore boards sparately. The front one has two motors for x-y motion and the back one has one extra for angle of attack control. Each motor is connected through two cables, a blue cable to the ClearCore board for instructions and a black cable to the power hub. Both power hubs are powered by the same power supply. All the boards also require additional 24VDC from a DC power supply.

## Setup

For new PCs, follow the [Arduino IDE 2.X - Installation/Setup](https://teknic-inc.github.io/ClearCore-library/_arduino_i_d_e_install.html). If the Teknic ClearCore does not show up after installing the wrapper, follow [manual installation guide](https://www.teknic.com/files/downloads/manual_install_instructions_arduino.pdf).

For new users on an old computer (where drivers have been installed already), just follow the step 3 in the manual installation guide might be enough since the wrapper might have been installed for all users.

## Running the gantry

### Initialization
Before turn on the power, manually position all degree of freedoms of the carriages away from the edge. Turn the sensor box to face forward. This is to ensure enough space for potential test movement upon launch.

Turn on DC power supply and activate CH1 to turn the power supply to the motors on. There will be a heavy click upon powering on, which is an indicator of normal operation. All degrees of freedom will do a small swing upon powering on.

Connect the Xbox controller (wired or Bluetooth).

Lauch the Jakiro app by running the file `jakiro.m`. Connect the ClearCore boards via the GUI. Adjust the two carriages to the starting/home position one by one. That is, activating the interactive control mode only one at a time. 

Use the game controller to adjust the position. Controls:
- left joystick - move front/left carriage
- right joystick - move the back/right carriage
- left trigger - decel (reduce speed for finner adjustment of the position)
- right triggle - accel
- left stick press - set current position as home position
- option button - return home
- left/right bumper - turn AOA (back carriage only)

When accidents happen, mostly carriages colliding with one another or hitting the edge, turn off the power immediately and restart the process.

### Running path tracking

To be added.