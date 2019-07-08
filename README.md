# EMSbus-raspberry-python

### Introduction.

A number of boiler brands, for instance Nefit and Buderus, use a two-wire
connection for communication between the various devices in the heating system,
the EMS-bus. EMS stands for "Energy Management System". The intention of this
project is to monitor the status and the performance of devices attached to the
EMS-bus, typically a boiler and a thermostat, and to have the ability to control
the heating system via EMS-bus as well. The design goals for this project are
(a) to utilize a Raspberry Pi, running raspbian, and (b) to program as much as
possible in Python3. It uses Xymon as the reporting tool.

For this project a lot of information about the (proprietary) EMS-bus is found
on various websites. The site [https://github.com/bbqkees/EMS-Wiki] tries to
give an overview of the available documentation.

### Software modules.

#### bfsm.py:

The driver for the EMS-bus uses two independent but synchronised Finite State
Machines (FSMs) to handle the full duplex communication at the side of the user
application and the half duplex communication with the physical EMS-bus. A short
survey of the available FSMs written in Python revealed a few modules, with much
more functionality than needed in this project. In order to have this project
run on even the smallest Raspberry Pi, a minimal implementation of a FSM was
created.

Class bfsm in module bfsm.py requires a fully populated matrix specifying for
each state and each event (stimulus) the event-action to take and the new state.
Optionally, a vector can be specified, which defines the state-action to perform
just prior to the execution of the event-action. The state-action might change
the event-queue and it might force the currently scheduled event-action to be
skipped by returning a False value.

#### emsvar.py

Module emsvar.py contains class definitions for the various types of variables
to be found in the messages exchanged on the EMS-bus. Currently the following
classes are defined:
<br>
 . ems_ascii    : a string of one of more octets containing ASCII characters
<br>
 . ems_datetime : a set of 6 octets containing a time stamp
<br>
 . ems_flag     : a field of one bit
<br>
 . ems_numeric  : a field which contains a two's complement number, which is
                  either integer (no divisor) or floating point (divisor != 1)
<br>
 . ems_sumima   : a numeric field with a high update frequency. This class
                  maintains the minimum, maximum and average value.
<br>
 . ems_switch   : an ems_flag class, with values 'On' or 'Off'
<br>
 . ems_version  : a set of two octets containing the major an minor version numbers
<br>
 
