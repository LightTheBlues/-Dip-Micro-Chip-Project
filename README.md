#Software Engineering (CS 4306)

##Project Title
##Prototyping of applications requiring CAN/CAN-FD on dsPIC33C Digital Signal Controller (DSC)

Team name: μdip [micro-dip]
Team members: Andrew DiFranco, Bryan Marquez, SeungJun Ryu, Yeongwoong Choi, Zhi Yong Li

Project client and contact information
Microchip, pr@microchip.com (public), academic@microchip.com (academic)

Communication medium (slack, github project, etc.)
Discord channel, Google docs, github, email (with client)

Short description of the project
Control area network (CAN) protocols facilitate the transmission of serial data between multiple nodes. Using CAN, data aggregated in a central processing unit can be transmitted elsewhere. 

In this project, using firmware that is adapted from the dsPIC33EV 5V CAN-LIN STARTER KIT, CAN communication will be implemented into the dsPIC33C Touch-CAN-LIN curiosity development board. Using CAN, the data from various nodes will be taken as input at the dsPIC33C Touch-CAN-LIN curiosity development board and outputted through the native CAN connector on the board. We will be showcasing CAN communication between a vehicle’s nodes by writing and reading serially encoded messages using the digital signal processor (dsPIC33C) via the Curiosity Board provided by MicroChip.

List of functions the software will provide

Disclaimer
The specific functions within the starter kit and curiosity board have not been demonstrated for us. The list below provides the general framework for which we wish to implement executables. Everything below is subject to change as we learn more about the board’s capabilities/ specific functions.
Modify firmware on the Starter kit for the DSC to be implemented on the Curiosity Development Board Demo source code will be provided for the dsPIC33EV board. We will learn from this and implement our own demo firmware onto the Curiosity Development board.

Engine Information (node 1)
Collect information from nodes reading/ writing engine diagnostic information into the network.

Wheel/ tire info (node 2)
Collect information from nodes reading/ writing wheel diagnostic information into the network.


Climate settings (node 3)
Collect information from nodes reading/ writing climate diagnostic information into the network.
Testing pre-existing button firmware on the DSC Ensure the firmware already baked into the Curiosity Development board works and can implemented to execute/ manipulate CAN functions.
