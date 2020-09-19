# Community-Management-ESD
Repo containing codes for Embedded Systems Design project

This project was done as part of the Embedded Systems Design course. The project aims to model a cluster of nodes (STM32 boards) spread across a region.
The nodes contain a temperature sensor, and a Zigbee transceiver. There is a base station that collects data from these nodes and actuates a motor 
depending on a threshold level of average humidity and temperature. 

## Prototype 

The system acts as a prototype of a network containing nodes and 2 levels of cluster heads. Data accumulated by Node, is sent via Zigbee to Cluster Head 1,
which routes its data along with Node's data to Cluster Head 2, which routes the received data, along with its own temperature logs to the Base Station. 

The Base Station is interfaced to a motor driver, and a flow meter. Depending on conclusions drawn from received data, the motor is switched on until 
the necessary quantity of water has been pumped, sensed via the flow meter. 

## Implementation

This project has been implemented using Keil uVision and STM32F04 development boards. HAL Libraries have been used wherever necessary. Arguably, the board
setup could have been done via STM32CUBEMX, but the coursework demanded an exercise in manual configuration of the board via Keil. 
