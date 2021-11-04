# BionxConsole

This is a reverse engineering project for a BionX electric bike control console. It is implemented in C++ for an Arduino Uno microcontroller and allows communication via the CAN bus using a SparkfunCANShield.

## Project Content

This project contains four sub-folders. Each one is independent and performs a specific task :

 - `sparkfun_sniffer`: sniffs messages from the CAN bus 
 - `sparkfun_shutdown`: allows to shut down the battery through CAN commands
 - `sparkfun_screen`: shows speed and pedal info in the Arduino screen
 - `sparkfun_bionx`: Implements the assistance and speed control algorithm


## System description

In the BionX e-bike, a CANBus ensures the communication between the battery, the engine, and the control console.  

```
 -----     -------		      ============
|Motor|   |Battery| 	   ||BionXConsole|| 
 -----		   -------		      ============
   ||		      ||			          ||
   ==============================================	<- CAN bus
```

The communication works in cycles that constitute the main loop of the BionX console routine.
In the very beginning, a set of parameters are initialized and the CAN cycle routine is started (each step requires sending specific messages in the CAN bus).
Each routine starts with a "CAN Loop starting" procedure. It contains 4 successive message exchanges with the motor (Motor CAN cycles) then one battery CAN cycle.


### `Sparkfun_sniffer`

In this project, we use an Arduino Uno microcontroller equipped with a Sparkfun shield for multiple purposes. 
The first and easiest thing that can be made is to use the microcontroller to listen and report how the three elements communicate in the CAN bus.
The system is wired this way

```
 -----    -------       =============			      ==========
|Motor|  |Battery| 	   ||BionXConsole|| 	   ||ArduinoUno|| 
 -----  	 ------        =============         ==========
   ||		    ||		      	   ||                      ||
   =============================================================== <- CAN bus
```

In this setup, the Arduino does not communicate any message to the other components. It just "sniffs" messages between the motor and the battery and the BionXconsole and reports them in the Arduino buffer. It is preferable to use the Arduino IDE to get easily this information.

As you can know if you have dealt with e-bike subcomponents communication protocols, these often do not use standardized protocols. It is surely not the same between different companies products and sometimes it is not the same in products of the same company. Using this configuration, we can learn how the components communicate and get all the necessary message codes to build a customized control console.

To get the code that performs this task, look at the folder `sparkfun_sniffer/`.

### `Sparkfun_shutdown`

Here, we start to use the Arduino as the control console. The set-up can be represented as follows

```
 -----       -------	       ==========
|Motor|     |Battery| 	   ||ArduinoUno|| 
 -----		     -------        ==========
   ||		    ||			               ||              
   ==================================================== <- CAN bus
```

The second thing I implemented is how to shut down the battery. This is very important since it helps to minimize energy consumption and it is done fully electronically (bus commands), contrary to the turning which is done through a button. The battery releases a sound when it is turned off. 

To get the code that performs this task, look at the folder `sparkfun_shutdown/`.


### `Sparkfun_bionx`

Here, we use the Arduino as the control console and we implement a speed and assistance control algorithm. The set-up stays globally the same, except adding an LCD Sparkfun module to get the speed and the pedal force values and a joystick to control speed assistance:

```
 -----       -------	       ==========			            -----     ----------
|Motor|     |Battery| 	   ||ArduinoUno|| ---------- | LCD | + | Joystick | 
 -----		     -------		      ==========			            -----      ----------
   ||		        ||			            ||              
   =====================================================	 <- CAN bus
```

The LCD seems necessary since testing the e-bike requires moving. The power supply of the controller can either be ensured by the e-bike battery (or an extra 9V non-rechargeable battery if you are lazy). 

To get the code that performs this task, look at the folder `sparkfun_bionx/`.


## Credits  & Resources

- This project is highly inspired by Dimitrios Simitas and [FranksMicro](https://github.com/franksmicro) anterior works about the subject (https://northerndtool.com/bionx). 
  [NOTE: Dimitrios Simitas website is no longer available].
- Dimitrios and Frank's original resources are not available online AFAIK.
- The CANbus library for Arduino is provided by [Fabian Greif](https://github.com/dergraaf) and [Sukkin Pang](https://www.linkedin.com/in/sukkin-pang-8503578/).


## Author & License

Copyright (c) 2018 Ali Saghiran - This work is all released under the GNU General Public License v3.0.
*Some files are released with other non-commercial licenses. Thes are compatible with a GPLv3 project.*

Contact : a{dot}saghiran[at]gmail{dot}com 
