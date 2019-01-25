# EMSbus-raspberry-python

Introduction.

A number of boiler brands, for instance Nefit and Buderus, use a two-wire connection for communication between the various devices in the heating system, the EMS-bus. EMS stands for "Energy Management System". The intention of this project is to monitor the status and the performance of devices attached to the EMS-bus, typically a boiler and a thermostat, and to have the ability to control the heating system via EMS-bus as well. The modular setup should allow one to make an interface to for instance domoticz. This project is designed to utilize a Raspberry Pi, running raspbian, and is programmed in Python3.

Software modules.

bfsm.py:

The driver for the EMS-bus uses two independant but synchronised Finite State Machines (FSMs) to handle the full duplex communication at the side of the user application and the half duplex communication with the physical EMS-bus. A short survey of the available FSMs written in Python revealed a few modules, with much more functionality than needed in this project. In order to have this project run on even the smallest Raspberry Pi, a minimal implementation of a FSM was created.

Class bfsm in module bfsm.py requires a fully populated matrix specifying for each state and each event (stimulus) the event-action to take and the new state. Optionally, a vector can be specified, which defines the state-action to perform just prior to the execution of the event-action. The state-action might change the event-queue and it might force the currently scheduled event-action to be skipped by returning a False value.

