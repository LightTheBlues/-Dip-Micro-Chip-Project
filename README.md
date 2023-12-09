
# Software Engineering (CS 4306) - Prototyping of applications requiring CAN/CAN-FD on dsPIC33C Digital Signal Controller (DSC)

Team name: μdip [micro-dip] 

Team members: Andrew DiFranco, Bryan Marquez, SeungJun Ryu, Yeongwoong Choi, Zhi Yong Li

## Project client and contact information
Microchip, pr@microchip.com (public), academic@microchip.com (academic)

## Communication medium (slack, github project, etc.)
Discord channel, Google docs, github, email (with client)

# Short description of the project
CAN (control area network) is a communication protocol that is used to handle the transmission of data between multiple nodes that all share a common bus. Each node has equal access to the bus and can transmit messages asynchronously whenever they have data that may be important elsewhere. This transmission is done in a broadcast manner, which means that all other nodes connected to the bus will receive the message. The broadcast method of communication creates reduced overhead in communication when compared to other communication protocols that use point-to-point communication, and also is more efficient in scenarios where the data needs to be received by multiple nodes. CAN also contains methods of handling errors within messages and nodes that may not be functioning properly. As a result of the efficent nature of CAN, it is present in many automotive applications. 

In this project, a demonstration of the CAN protocol will be shown by creating the firmware for the dsPIC33C Touch-CAN-LIN curiosity development board, which houses the dsPIC33CK1024MP710 digital signal controller. The demonstration will involve a common bus which connects the curiousity board with another board known as the dsPIC33EV 5V CAN-LIN STARTER KIT. To show messages being send in real time, a CAN analyzer developed by Microchip will also be present on the bus, allowing for messages to be displayed on an external computer. 

# List of functions the software will provide

### Message transmission & reception between both nodes 

### Handling of message arbitration 

### Handling of errors in cases where messages are transmitted incorrectly 

### User interaction with both boards to alter how messages are transmitted

