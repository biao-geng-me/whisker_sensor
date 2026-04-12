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

### Reset

Should collision happen, you'll hear heavy click sounds. The positions of the carriages will be inaccurate. A reset is needed. Turn off the power and move carriages to marked positions manually. Turn the sensor box so the angle of attack is zero.

### Running path tracking

Select a path file in the Experiment panel of the Jakiro app. Click a run mode (Path-Path, PathHuman, PathAgentPre, or PathAgent). The front carriage will follow the selected path profile.

For **Agent-Train** mode, the Python RL server must be running first (see [RL agent server](#rl-agent-server) below).

## MATLAB dependencies

### Simulink 3D Animation (vrjoystick)

Joystick/gamepad input uses `vrjoystick`, which ships with the **Simulink 3D Animation** toolbox. Install it from the MATLAB Add-On Explorer if it is not already present:

1. In MATLAB, go to **Home → Add-Ons → Get Add-Ons**.
2. Search for **Simulink 3D Animation** and install it.
3. Verify with `vrjoystick(1)` in the command window — it should return a joystick object when a controller is connected.

### HebiKeyboard

Keyboard input uses `HebiKeyboard`, a utility class from the **HEBI Robotics MATLABInput**. It reads key states at a hardware level (`'native'` mode) so control responds even when the MATLAB figure is not in focus.

1. Download the HEBI Robotics MATLAB API from [github.com/HebiRobotics/MatlabInput](https://github.com/HebiRobotics/MatlabInput) (click **Code → Download ZIP** or clone the repo).
2. Extract the archive to a convenient location, e.g. `C:\tools\hebi-MATLABInput`.
3. Add it to the MATLAB path (see path setup below).

Verify the install with `HebiKeyboard('native')` in the MATLAB command window — it should return a keyboard object without error.

### MATLAB path setup

`Jakiro.m` depends on the `wavi/` module (data acquisition) and the HEBI Robotics API for keyboard input. Before launching, add all required directories to the MATLAB path:

```matlab
addpath('path/to/whisker_sensor_repo/gantry_control/matlab');
addpath('path/to/whisker_sensor_repo/wavi');
addpath('path/to/hebi-MATLABInput');  % HebiKeyboard
```

Or use **Home → Set Path** and add the three folders above. Save the path so this is not needed on every launch.

## Python environment (RL agent server)

### Requirements

The Python server requires:

- Python 3.10+
- `numpy`
- `matplotlib` (TkAgg backend — standard on Windows)
- `torch` (PyTorch, CPU or CUDA)

### Environment location (hardcoded)

The MATLAB app launches the server by activating a virtual environment at a **hardcoded path**:

```
%USERPROFILE%\py_envs\rl\
```

For example, on a typical Windows machine this is `C:\Users\<YourName>\py_envs\rl\`.

Create the environment there:

```powershell
python -m venv "$env:USERPROFILE\py_envs\rl"
& "$env:USERPROFILE\py_envs\rl\Scripts\Activate.ps1"
pip install numpy matplotlib torch
```

Install a CUDA-enabled PyTorch build if GPU training is desired (see [pytorch.org](https://pytorch.org/get-started/locally/) for the correct install command).

### Starting the server manually

The Jakiro app can launch the server automatically via the **Server Config** button in the Experiment panel. To start it manually instead:

```powershell
& "$env:USERPROFILE\py_envs\rl\Scripts\Activate.ps1"
cd path/to/whisker_sensor_repo/gantry_control/python
python main_server_loop.py
```

The server listens on `127.0.0.1:65432` by default. Connect from Jakiro using the **Connect Agent** button after the server reports it is waiting for a client.