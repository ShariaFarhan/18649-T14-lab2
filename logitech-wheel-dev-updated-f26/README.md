# Logitech Steering Wheel Module

This module allows access from Python to the steering wheel SDK provided by Logitech for their gaming steering wheels. The API of this module is the same as the "Logitech Gaming Steering Wheel SDK" version 8.75.30. This SDK and it's documentation can be downloaded from the [Logitech G website](https://www.logitechg.com/en-us/innovation/developer-lab.html). The module has been tested on Python 3.8 and only works on Windows.  

This module includes part of the before mentioned steering wheel SDK (the `LogitechSteeringwheel.dll` and `LogitechSteeringWheelEnginesWrapper.dll` files). These files do not fall under the open source license with the rest of the module. They are re-distributed with permission from Logitech (also see the README file in the SDK). Therefore, there is no need to obtain the SDK. The only dependency of this module is the Logitech Gaming Software.

## Installation instructions

The `logitech-steering-wheel` module is available from PyPi and can be installed using pip (`pip install logitech-steering-wheel`). The only dependency is the Logitech Gaming Software that can be downloaded for [Logitech's website](https://support.logi.com/hc/en-gb/articles/360025298053-Logitech-Gaming-Software). The module has been tested with version 5.10. This version works with older steering wheels. According to Logitech, the SDK is also compatible with newer versions of the gaming software.

In some cases, Windows has been known to install other drivers when a steering wheel is plugged in. These default drivers installed by Windows do not work with the SDK. The solution to is problem is to:
1) plug in the steering wheel,
2) uninstall the driver Windows automatically installed, and leave steering wheel plugged-in and turned on, and
3) Install the Logitech Gaming Software (this should automatically install the correct drivers)

## Using the SDK

For quick access, the SDK documentation is included in the repo.
The respective Python functions to access DLL functionality can be found [here](logitech_steering_wheel/_wrapper.py)
Refer to the script [proxy_gui.py](proxy_gui.py) for details on interfacing with the wheel.


## Proxy Application Test

You will require 2 machines (call them `W` and `R`), with at least 1 running Windows (`W`) 
to interface with the logitech wheel. Make sure `W` and `R` can communicate over either wireless
or Ethernet.

### Setup
1. Plug in the steering wheel to `W` after performing the installation instructions above
successfully. Make sure the wheel powers on.
2. Configure the IP Addresses and Ports accordingly in applications `proxy_gui.py` and `proxy_receiver/receiver.c`.
3. Compile C receiver on `R`: `cd proxy_receiver; gcc -pthread receiver.c -o proxy_receiver`

### Running the applications
1. Run `python3 proxy_gui.py` script on `W`
2. Run `./proxy_receiver` on `R`

You should see wheel state reception on `R` and force feedback data on `W`. The data rates on
either side can be set by changing the `TX_INTERVAL_MS` field



## UDP Data Encoding

The following section highlights the data encoding for the current communication protocol in the
test application. This was designed to be simple enough to enable meeting the basic requirements.
You may add more functionality if you'd like

### Wheel ---> System

The transmission from `proxy_gui.py` send the following:
1. 32-bit packet counter
2. [DIJOYSTATE2](logitech_steering_wheel/_state.py) ctype structure

These are both packed in order into a bytearray. Note, this enables convenient unpacking of
wheel state into an object in C, as demonstrated in the test [receiver](proxy_receiver/receiver.c)

### System ---> Wheel

The `proxy_gui.py` expect to receive:
1. A single byte representing the signed direction of force (valid values include -100 to 100 
as highlighted in the SDK)



