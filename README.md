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
the event-queue.

#### emsvar.py

Module emsvar.py contains class definitions for the various types of variables
to be found in the messages exchanged on the EMS-bus. Currently the following
classes are defined:
<br>
<table>
 <tr> <th>Class</th> <th>Short description</th> </tr>
 <tr> <td>ems_ascii</td> <td>a string of one of more octets containing ASCII characters</td> </tr>
 <tr> <td>ems_datetime</td> <td>a set of 6 octets containing a time stamp</td> </tr>
 <tr> <td>ems_flag</td> <td>a field of one bit</td> </tr>
 <tr> <td>ems_numeric</td> <td>a field which contains a two's complement number, which is either integer (no divisor) or floating point (divisor != 1)</td> </tr>
 <tr> <td>ems_sumima</td> <td>a numeric field with a high update frequency, of which the minimum, maximum and average value are maintained</td> </tr>
 <tr> <td>ems_switch</td> <td>an ems_flag class, with values 'On' or 'Off'</td> </tr>
 <tr> <td>ems_version</td> <td>a set of two octets containing the major and minor version numbers</td> </tr>
</table>

Repeating fields and repeating structures are not implemented (yet).

#### emsbus.py

Module emsbus.py defines methods to read frames from and write frames to the
half-duplex EMS-bus. It takes care of the synchronisation issues. It validates
the check-sum upon receipt of a frame and adds a check-sum prior to sending a
frame. Additionally, it collects information about the performance
characteristics of the EMS-bus.

This module defines two modes of operation, monitor and participate. In the
former mode all frames are read and passed on, irrespective of the destination
address. Output to the bus is silently ignored. In the participate mode only
frames addressed to this device are passed on, including broadcasts. If a poll
request is received, any frames queued for transmission are sent.

This module has been used in monitor mode for more then a year. The participate
mode is hardly tested. A better method to (read and) write break signals on the
EMS-bus is needed, and perhaps required, in order to use the participate mode.

