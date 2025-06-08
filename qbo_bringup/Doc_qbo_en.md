# Q.boards

The Qbo robot needs to interact with its surroundings. Qbo has some sensors and actuators in order to achieve this goal.

The Q.boards have been designed to aquiere the sensors data and to make it available for the PC that is embedded in Qbo. The boards also gives the PC the capability to controll the actuators of Qbo.

In order to control the sensors and motors that comes with Qbo, three boards have been designed. Their names are Q.board1, Q.board2 and Q.board3. Qbo has two additional boards whose names are Q.board4 and Q.board5 that serve as an IMU sensor and as the mouth LED matrix respectively.

## Q.board1

The Q.board1 is the base control board of Qbo. Its main functions are the following:

    Control of two DC Motors
    Control of the audio amplifier
    Control of the I²C Bus

In order to achieve this goal the board is equipped with a fast ATmega1280 microcontroller that is compatible with the Arduino IDE.

The motors and sensors are all connected to the ATmega1280 microcontroller.

A motor controller is integrated in the board. This motor controller is able to move two 12V, 2A motors. The recomended motors are the EMG30 motors.
The EMG30 (encoder, motor, gearbox 30:1) is a 12v motor fully equipped with encoders and a 30:1 reduction gearbox. It is ideal for small or medium robotic applications, providing cost effective drive and feedback for the user. It also includes a standard noise suppression capacitor across the motor windings.
Connector

    The EMG30 is supplied with a 6 way JST  connector (part no PHR-6) at the end of approx 90mm of cable as standard.
    The connections are:

    Wire colour		Connection
    Purple (1)		Hall Sensor B Vout
    Blue (2)		Hall sensor A Vout
    Green (3)		Hall sensor ground
    Brown (4)		Hall sensor Vcc
    Red (5)		+ Motor
    Black (6)		- Motor
    Wire colours are from the actual cable.
    The hall sensors accept voltages between 3.5v and 20v.
    The outputs are open collector and require pull-ups to whatever signal level is required.
    On the MD25 they are powered from 12v and pulled up to 5v for the signals.

    specification

    Rated voltage		12v
    Rated torque		1.5kg/cm
    Rated speed 		170rpm
    Rated current		530mA
    No load speed		216
    No load current		150mA
    Stall Current 		2.5A
    Rated output		4.22W
    Encoder counts per output shaft turn		360

    Measured Shaft Speed when used off-load with MD23 and 12v supply.
    Minimum Speed		1.5rpm
    Maximum Speed		200rpm


The hall pins of the encoders of the motors (two for each one) are routed to interruption enabled pins in the ATmega1280 microcontroller, so a PID controller is possible with the board.

An I²C bus connector is also present in the board, so several sensors and devices can be attached to it. There are at the moment four types of devices. The SRF10 sensors are used to avoid hitting on the walls. The LCD03 board is used to give some extra information of the status of the robot. The Q.board4 is controlled by the I²C Bus and can be used as an inertial measurement unit to detect falls of the robot and to improve the positioning system.

The ATmega1280 serial bus is connected to a Serial_to_USB converter. Thanks to the Arduino bootloader the board can be programed and controlled easily with the computer.

A program to control the motors and sensors which come with Qbo is already finished, but it can be modified by anyone who wants to increase or change its functionality.

The board has an integrated audio amplifier. The audio input for this amplifier is a 3-pin connector in the board with the Audio Left, ground and Audio Right signals. These signals must be analog signals.


## Q.board2

The Q.board2 is located at the head of Qbo and its a compatible Arduino Duemilanove board. It has the same connectors for the shields as the original Duemilanove board.

It has an integrated shift register ic (integrated circuit) that controls the mouth leds.

It has 3 electrect microphones inputs routed to 3 analog pins of the ATmega328 microcontroller. These microphone inputs are also routed to an analog multiplexer, so one of the 3 microphone inputs is delivered to the microphone output the board comes with. In Qbo this output is connected to the PC mic input or to the PC line input.

It has an additional analog input. This additional input is connected in Qbo to the PC audio output in order to know when the PC is outputting some sound so the mouth can be changed simultaneously, but it can be used for other purposes.

It has 2 servo connectors where the head pan and tilt servos. The servo power comes from a voltage regulator that takes the power from the power connector in the board.

The logic of the board takes the power from the USB port.

A program to control all these devices is already finished, but it can be modified by anyone who wants to increase its functionality.


## Q.board3

This board acts as an energy controller board. Its purpose is to achieve the better energy performance for Qbo.

The board makes it possible to completely turn the PC and other Qbo components power off.

It also gives a battery charging capability to Qbo.

The board is governed by a stm32f103 32bit Cortex-M3 based microcontroller by STMicroelectronics. It has a switch digital input that serves as a power on/off input, a digital output to light a led and a digital output that acts as the PC on/off switch button.

The board has a connector for the battery and a connector for an external power supply. If no power supply is connected, the board redirects the battery power to the power outputs. If the external power supply is connected, the board switches the output power from the battery to the external power supply and activates the battery charging.

Four different power outputs are available in the board. Two of them have unregulated outputs, so its output level is the battery level or the external power supply power. The other two outputs have regulated outputs. Their output level is 12V and its main purposes are to supply power to the PC and to the audio amplifier. The PC output also has a current sensor so it is possible to check if the PC is on or off.

The maximum power output of the board is 150W.

The board also has an I2C connector. It acts as an I2C device so the battery level and the power state of the board can be read by other microcontrollers.

The board can be programed trough a serial connector easily. It is important to note that due to the high power nature of the board it is not recommended to change the original software designed for the board without the needed knowledge of its operating principle.

## Q.board4

This board contains the gyroscope and accelerometer sensors, both accesible trough an I²C interface. It also has 3 expansion pins not used by default.

The schematic of the board is available in the following link: qboard4_v1.pdf

The list of components of the board is available in the following link: qboard4_v1.xls

## Q.board5

This is the mouth LED matrix board. It contains 20 standard LEDs to form different mouth shapes and a tri-color LED that acts as the Qbo nose.