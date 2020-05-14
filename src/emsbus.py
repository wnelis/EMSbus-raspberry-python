#
# Class emsbus defines the methods to read frames from and write frames to the
# (asynchronous, half-duplex) EMS-bus. Additionally, it collects information
# about the performance characteristics of the EMS-bus.
#
# This class does not interpret the content of the frames being received or
# sent. Upon receipt of a frame, it validates the check-sum and removes it from
# the frame. Upon sending a frame, it will compute and append the check-sum
# prior to sending the frame.
#
# This class does handle the synchronisation of input and output on the
# half-duplex EMS-bus. Any egress frames are buffered within the class untill a
# poll-request for this devices is received. Then the buffered data followed by
# a poll-reply are sent onto the bus.
#
# There are two different layouts of a frame on the EMS-bus. The polling
# messages as well as the write replies consist of a single byte, followed by a
# break signal (<BRK>). The second frame layout is used for all other types of
# messages and is as follows:
#   [src] [dest] [type] [offset] [data] [chk] <BRK>
# Except for field [data], all fields consist of one octet. The length of field
# [data] depends on the message type [type], but is at least one octet long.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2018.09
#
# To do:
# - An answer to a write command should arrive within 100 [ms].
# - Check: use interCharTimeout in serial input?
# - Include a version number, which is retrievable at run time.
#
import bfsm				# Basic finite state machine
import ctypes				# Invoke C function
import queue				# Inter thread communication
import serial				# Asynchronous serial i/o
import syslog				# Syslog access
import termios				# POSIX style tty control
import threading			# Run program sections concurrently
import time
import types				# Map function onto bound method
import watchdog				# Resettable watchdog timer

#
# Define the offset of the header fields in a frame. The user data starts at
# offset EMS_Data and extends untill the end of the frame. The offset in field
# EMS_Offset assigns value 0 to offset EMS_Data in the frame. Note that the
# checksum field, the last field (octet) in the frame, is already removed from
# the frame.
#
EMS_Source= 0
EMS_Destin= 1
EMS_Type  = 2
EMS_Offset= 3
EMS_Data  = 4

#
# Define the 'ack' and 'nack' to be sent immediatly following a write request.
# The reply codes defined below cannot be and are not used as device identifier.
#
EMS_Write_reply= (			# Write reply codes
  0x01,					# Succes
  0x04					# Failure
)

EMSBUS_Break_time= 0.002		# Break time, 19 bits @ 9600 [b/s]
EMSBUS_Min_frame_size= 4		# Minimum frame size of RQ, RP or WQ,
					#   excluding the checksum
EMSBUS_Max_frame_size= 34		# Maximum frame size of RQ, RP or WQ,
					#   excluding the checksum
EMSBUS_PolReq_Timeout= 0.200		# Poll request time out [s]
EMSBUS_ReaReq_Timeout= 0.125		# Read request time out [s]
EMSBUS_WriReq_Timeout= 0.125		# Write request time out [s]

#
# Serial device parameter definitions.
#
SERIAL_Device= '/dev/ttyAMA0'		# PL011 UART device name on RPi
# Define the special character codes used in the octet stream. Both a break and
# a framing error are indicated with a three character string, which starts
# with b'\xff\x00'. An octet with value b'\xff' as received on the serial port
# is doubled.
SERIAL_Escape= 0xff			# Indicator for meta information
SERIAL_Break = b'\xff\x00\x00'		# Break detected
SERIAL_FrmErr= 0x00			# Framing error detected

#
# This driver supports three modes, the monitor mode, the participate mode and a
# mix of both modes. In monitor mode, all frames are read and passed,
# irrespective of the destination address. The poll requests and replies are
# silently discarded. Output to the bus is silently ignored.
# In participate mode, only frames addressed to this device are passed on,
# including the broadcasts. The poll requests are scanned for a poll of this
# device and if one is found, any frames queued in the output queue are
# transmitted.
# In mixed mode all ingress frames are forwarded, egress frames are forwarded at
# the appropriate time, and the polls are handled like in participate mode.
#
EMSBUS_MODE_MONITOR= 1			# Monitor (read-only and all) mode
EMSBUS_MODE_PARTCPT= 2			# Participate( read/write) mode
EMSBUS_MODE_MONPART= 3			# Do both
#
EMSBUS_MASK_MONITOR= 0x01		# Forward some or all ingress frames
EMSBUS_MASK_EGRESS = 0x02		# Forward egress frames


class Emsbus():
  '''Driver to read frames from and sent frames onto the EMS bus'''

  def __init__( self, device, mode ):
  # Preset instance variables.
    if device <= 0  or  device > 0x7f  or  device in EMS_Write_reply:
      raise ValueError( "Illegal device id: %04x" % device )
    if mode < EMSBUS_MODE_MONITOR  or  mode > EMSBUS_MODE_MONPART:
      raise ValueError( "Illegal mode: %d" % mode )
    self.device= device			# Save my EMS bus device id
    self.mode  = mode			# Save mode: monitor, participate or mixed
    self.name  = 'bus'			# Name to use in syslog messages
  #
  # Change the type of the actions from 'function reference' to 'bound method
  # reference'. Then the 'self' parameter will be added automatically, giving
  # the action methods access to the instance variables.
  #
    for mode in Emsbus.FSMDT:
      for gress in Emsbus.FSMDT[mode]:
        for state in Emsbus.FSMDT[mode][gress]:
          for stim in Emsbus.FSMDT[mode][gress][state]:
            Emsbus.FSMDT[mode][gress][state][stim]= (
              Emsbus.FSMDT[mode][gress][state][stim][0],	# Do not change
              types.MethodType(Emsbus.FSMDT[mode][gress][state][stim][1],self) )
  #
    for mode in Emsbus.FSMSA:
      for gress in Emsbus.FSMSA[mode]:
        for state in Emsbus.FSMSA[mode][gress]:
          if Emsbus.FSMSA[mode][gress][state] is None:
            continue
          Emsbus.FSMSA[mode][gress][state]= types.MethodType(Emsbus.FSMSA[mode][gress][state],self)
  #
    self.serial= None			# Serial port object instance

    self.reader_thread= None		# Ingress variables
    self.reader_alive = None
    self.iframe       = bytearray()	# Assembled ingress frame
    self.iframe_error = 0		# Number of errors in current/last frame
    self.iframe_queue = queue.Queue()	# Queue for frames read from bus
    self.iframe_time  = None		# Time of arrival of first octet of frame
    self.iframe_type  = None		# Type of frame

    self.writer_thread= None		# Egress variables
    self.writer_alive = None
    self.eframe       = None
    self.eframe_queue = queue.Queue()	# Queue for frames to be written to bus
    self.eframe_time  = None

    self.idisp_thread = None		# Ingress dispatcher variables
    self.idisp_alive  = None
    self.idisp_queue  = queue.Queue()
    self.idisp_frame  = None
    self.idisp_fsm    = bfsm.Bfsm( Emsbus.FSMDT[self.mode]['ingress'],
                                   Emsbus.FSMSA[self.mode]['ingress'] )
    self.idisp_fsm_wdt= watchdog.WatchdogTimer()
    self.idisp_log    = None		# Call back for logging a frame
    self.idisp_log_slf= None		# Object instance to be used for logging

    self.edisp_thread = None		# Egress dispatcher variables
    self.edisp_alive  = None
    self.edisp_queue  = queue.Queue()	# Input from user application
    self.edisp_buffer = queue.Queue()	# Buffer area, frames awaiting transmission
    self.edisp_frame  = bytearray()
    self.edisp_type   = None
    self.edisp_fsm    = bfsm.Bfsm( Emsbus.FSMDT[self.mode]['egress'],
                                   Emsbus.FSMSA[self.mode]['egress'] )
    self.edisp_fsm_wdt= watchdog.WatchdogTimer()

  # Preset the SME-bus statistics.
    self.sbs= dict(			# Define EMS bus statistics save area
      bus_address_conflict  = 0,	# Total frames received from my address
      ingress_total_frames  = 0,	# Total ingress frame count
      ingress_total_octets  = 0,	# Total ingress octet count
      ingress_echo_frames   = 0,	# Total suppressed echos of egress frames
      ingress_empty_frames  = 0,	# Total ingress empty frame count
      ingress_short_frames  = 0,	# Total ingress short frame count
      ingress_errors        = 0,	# Total ingress erred octets
      ingress_err_frames    = 0,	# Total ingress erred frame count
      ingress_err_octets    = 0,	# Total ingress octets in erred frames
      ingress_err_timeout   = 0,	# Total ingress time outs
      ingress_err_protocol  = 0,	# Total ingress protocol errors
      ingress_emsplus_frames= 0,	# Total ingress frames using EMS-plus format
      ingress_polreq_frames = 0,	# Total ingress poll request frame count
      ingress_polrep_frames = 0,	# Total ingress poll reply frame count
      ingress_reareq_frames = 0,	# Total ingress read request frame count
      ingress_rearep_frames = 0,	# Total ingress read reply frame count
      ingress_wrireq_frames = 0,	# Total ingress write request frame count
      ingress_wrirep_frames = 0,	# Total ingress write reply frame count
      egress_total_frames   = 0,	# Total egress frame count
      egress_total_octets   = 0,	# Total egress octet count
      egress_polrep_frames  = 0,	# Total egress poll reply frame count
      egress_reareq_frames  = 0,	# Total egress read request frame count
      egress_rearep_frames  = 0,	# Total egress read reply frame count
      egress_wrireq_frames  = 0,	# Total egress write request frame count
      egress_wrirep_frames  = 0,	# Total egress write reply frame count
      egress_err_short_frames= 0,	# Total egress frames, too short
      egress_err_long_frames= 0,	# Total egress frames, too long
      egress_err_timeout    = 0,	# Total egress time outs
      egress_err_protocol   = 0,	# Total egress protocol errors
      start_time = time.time()		# Start of collection of statistics
    )

 #
 # Finite State Machine methods.
 # -----------------------------
 #
 # Define the methods which are used as actions in the finite state machines
 # (fsm). Their names start with either 'ifsm' (ingress fsm) or 'efsm' (egress
 # fsm).
 #
  def ifsa_handle_timeout( self ):	# Time out call back
    self.sbs['ingress_err_timeout']+= 1
    self.idisp_fsm.HandleEvent( 'timout' )

  def ifsa_start_wdt( self ):		# State action, set watch dog timer
    if   self.idisp_fsm.NxtState in ('RxRq','XmRq'):
      self.idisp_fsm_wdt.start( EMSBUS_ReaReq_Timeout, self.ifsa_handle_timeout )
    elif self.idisp_fsm.NxtState in ('RxWq','XmWq'):
      self.idisp_fsm_wdt.start( EMSBUS_WriReq_Timeout, self.ifsa_handle_timeout )
    return True				# No change in event queue

  def ifsa_stop_wdt( self ):		# State action, stop watch dog timer
    self.idisp_fsm_wdt.stop()
    return True				# No change in event queue


 # Method ifsm_check_polrep checks the device address in the poll_reply. As the
 # echo of a poll_reply sent by this entity is suppressed in thread Reader, the
 # device address of this entity should not appear in poll_replies received by
 # this entity. Thus the reception of a poll_reply with the address of this
 # entity probably means that another device with the same address is attached
 # to the EMS-bus. Stop this entity from sending data onto the EMS-bus. (Fall
 # back to monitor mode?)
  def ifsm_check_polrep( self ):	# Check the polrep
    if self.idisp_frame['Frame'][0] != self.device:
      self.idisp_frame= None		# Ignore frame
      return
    pass				# Do something dreadfull

  def ifsm_do_nothing( self ):		# Do nothing
    pass

  def ifsm_do_rearep( self ):		# Change frame type to ReadReply
    self.idisp_frame['Type']= 'rearep'
    self.idisp_fsm.AugmentEvent( 'rearep' )
    self.sbs['ingress_rearep_frames']+= 1

  def ifsm_do_wrireq( self ):		# Change frame type to WriteRequest
    self.idisp_frame['Type']= 'wrireq'
    self.idisp_fsm.AugmentEvent( 'wrireq' )
    self.sbs['ingress_wrireq_frames']+= 1

  def ifsm_forward_frame( self ):	# Forward frame to 'output' queue
    self.idisp_queue.put( self.idisp_frame )
    self.idisp_frame= None

  def ifsm_handle_rearep( self ):	# Report error if not a broadcast
    if self.idisp_frame['Frame'][EMS_Destin] != 0:
      self.ifsm_report_error()
    self.ifsm_forward_frame()

  def ifsm_ignore_frame( self ):	# Ignore frame
    self.idisp_frame= None		# Clear reference to frame

  def ifsm_passon_polreq( self ):	# Pass on a polreq to egress_dispatcher
    if self.idisp_frame['Frame'][0] == self.device | 0x80:
      self.edisp_queue.put( 'PQ' )	# Inform egress_dispatcher
    self.idisp_frame= None		# forget frame

  def ifsm_passon_rearep( self ):	# Pass on a rearep
    if self.idisp_frame['Frame'][EMS_Destin] in (0x00,self.device):
      self.ifsm_forward_frame()		# Pass on request to user application
    else:
      self.ifsm_ignore_frame()

  def ifsm_passon_reareq( self ):	# Pass on a reareq
    if self.idisp_frame['Frame'][EMS_Destin] != self.device | 0x80:
      self.idisp_frame= None		# Forget frame
      return
    self.edisp_queue.put( 'RQ' )	# Notify egress_dispatcher
    self.ifsm_forward_frame()		# Pass on request to user application

  def ifsm_passon_wrireq( self ):	# Pass on a wrireq
    if self.idisp_frame['Frame'][EMS_Destin] != self.device:
      self.idisp_frame= None
      return
    self.edisp_queue.put( 'WQ' )	# Notify egress_dispatcher
    self.ifsm_forward_frame()		# Pass on request to user application

  def ifsm_report_error( self ):	# Report a protocol error
    (state,stim)= self.idisp_fsm.GetState()
    self._log_message( "Error, idisp recieved {0} in state {1}".format(stim,state) )
    self.sbs['ingress_err_protocol']+= 1

  def ifsm_reppe_and_forf( self ):	# REPort Protocol Error, FORward Frame
    self.ifsm_report_error()
    self.ifsm_forward_frame()

  def ifsm_reppe_and_ignf( self ):	# REPort Protocol Error, IGNore Frame
    self.ifsm_report_error()
    self.ifsm_ignore_frame()

  def ifsm_reprxd_and_forf( self ):	# REPort Read eXchange Done, FORward Frame
    if   self.idisp_frame['Frame'][EMS_Destin] == self.device:
      self.ifsm_repwxd_and_forf()
#   elif self.idisp_frame['Frame'][EMS_Destin] == 0x00:
#     self.ifsm_forward_frame()		# Pass on request to user application
    else:
      self.idisp_frame= None

  def ifsm_repwxd_and_forf( self ):	# REPort Write eXchange Done, FORward Frame
    self.edisp_queue.put( 'XD' )	# Notify egress_dispatcher
    self.ifsm_forward_frame()		# Pass on request to user application


  def efsa_handle_timeout( self ):	# Time out call back
    self.sbs['egress_err_timeout']+= 1
    self.edisp_fsm.HandleEvent( 'timout' )

  def efsa_start_wdt_er( self ):	# State action, set watch dog timer
    if   self.edisp_fsm.NxtState == 'WeXd':
      self.edisp_fsm_wdt.start( EMSBUS_PolReq_Timeout, self.efsa_handle_timeout )
    return True				# No change in event queue

  def efsa_stop_wdt_er( self ):		# State action, stop watch dog timer
    cs= (self.edisp_fsm.State,self.edisp_fsm.Stimulus)
    if cs == ('WePq','rcvxd'):
      self.edisp_fsm_wdt.stop()
    return True				# No change in event queue

  def efsa_start_wdt_ir( self ):	# State action, set watch dog timer
    if   self.edisp_fsm.NxtState == 'WiRp':
      self.edisp_fsm_wdt.start( EMSBUS_ReaReq_Timeout, self.efsa_handle_timeout )
    elif self.edisp_fsm.NxtState == 'WiWp':
      self.edisp_fsm_wdt.start( EMSBUS_WriReq_Timeout, self.efsa_handle_timeout )
    return True				# No change in event queue

  def efsa_stop_wdt_ir( self ):		#  State action, stop watch dog timer
    cs= (self.edisp_fsm.State,self.edisp_fsm.Stimulus)
    if cs in ( ('WiRp','rearep'), ('WiRpb','rearep') ):
      self.edisp_fsm_wdt.stop()
    if cs in ( ('WiWp','wrirep'), ('WiWpb','wrirep') ):
      self.edisp_fsm_wdt.stop()
    return True				# No change in event queue


  def efsm_buffer_frame( self ):	# Buffer frame temporarily
    qv= { 'Frame': self.edisp_frame, 'Type': self.edisp_frame_type }
    self.edisp_buffer.put( qv )
    self.edisp_frame= None

  def efsm_do_nothing( self ):		# Do nothing
    pass

  def efsm_do_rearep( self ):		# Change frame type to ReadReply
    self.edisp_frame_type= 'rearep'
    self.edisp_fsm.AugmentEvent( 'rearep' )

  def efsm_do_wrireq( self ):		# Change frame type to WriteRequest
    self.edisp_frame_type= 'wrireq'
    self.edisp_fsm.AugmentEvent( 'wrireq' )

  def efsm_forward_frame( self ):	# Forward recieved frame
    qv= { 'Frame': self.edisp_frame, 'Type': self.edisp_frame_type }
    self.eframe_queue.put( qv )
    self.edisp_frame= None

  def efsm_forward_buffer( self ):	# Forward buffered frame
    qv= self.edisp_buffer.get()
    self.eframe_queue.put( qv )

  def efsm_handle_poll( self ):		# Handle a polreq
    if not self.edisp_buffer.empty():
      pass
    self.efsm_send_polrep()		# Push polrep

  def efsm_ignore_frame( self ):	# Ignore frame
    self.edisp_frame= None		# Clear reference to frame

  def efsm_report_error( self ):	# Report a protocol error
    (state,stim)= self.edisp_fsm.GetState()
    self._log_message( "Error, edisp recieved {0} in state {1}".format(stim,state) )
    self.sbs['egress_err_protocol']+= 1

  def efsm_reppe_and_ignf( self ):	# REPort Protocol Error, IGNore Frame
    self.efsm_report_error()
    self.edisp_frame= None

  def efsm_send_polrep( self ):		# Send a poll reply
    qv= { 'Frame': bytes([self.device]), 'Type': 'polrep' }
    self.eframe_queue.put( qv )		# Push polrep
    if self.edisp_buffer.empty():
      self.edisp_fsm.AugmentEvent( 'bufemp' )


 #
 # Finite state machine transition matrices.
 # -----------------------------------------
 #
  FSMDT= { EMSBUS_MODE_MONITOR :
    { 'ingress' :
      { 'Init' :			# State = Initial
        { 'polreq' : ( 'Init', ifsm_ignore_frame   ),
          'polrep' : ( 'Init', ifsm_ignore_frame   ),
          'reareq' : ( 'RxRq', ifsm_forward_frame  ),	# Start timer
          'rearep' : ( 'Init', ifsm_handle_rearep  ),
          'wrireq' : ( 'RxWq', ifsm_forward_frame  ),	# Start timer
          'wrirep' : ( 'Init', ifsm_reppe_and_forf ),	# Error
          'rporwq' : ( 'Init', ifsm_do_wrireq      ),
          'errfrm' : ( 'RxEf', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        },
        'RxRq' :			# State = Received ReadRequest
        { 'polreq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'polrep' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'reareq' : ( 'RxRq', ifsm_reppe_and_forf ),	# Error
          'rearep' : ( 'Init', ifsm_forward_frame  ),
          'wrireq' : ( 'RxWq', ifsm_reppe_and_forf ),	# Error
          'wrirep' : ( 'Init', ifsm_reppe_and_forf ),	# Error
          'rporwq' : ( 'RxRq', ifsm_do_rearep      ),
          'errfrm' : ( 'RxEf', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        },
        'RxWq' :			# State = Received WriteRequest
        { 'polreq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'polrep' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'reareq' : ( 'RxRq', ifsm_reppe_and_forf ),	# Error
          'rearep' : ( 'Init', ifsm_reppe_and_forf ),	# Error
          'wrireq' : ( 'RxWq', ifsm_reppe_and_forf ),	# Error
          'wrirep' : ( 'Init', ifsm_forward_frame  ),
          'rporwq' : ( 'Init', ifsm_do_wrireq      ),	# Error
          'errfrm' : ( 'RxEf', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        },
        'RxEf' :			# State = Received erred frame
        { 'polreq' : ( 'Init', ifsm_ignore_frame   ),
          'polrep' : ( 'Init', ifsm_ignore_frame   ),
          'reareq' : ( 'RxRq', ifsm_forward_frame  ),
          'rearep' : ( 'Init', ifsm_forward_frame  ),
          'wrireq' : ( 'RxWq', ifsm_forward_frame  ),
          'wrirep' : ( 'Init', ifsm_forward_frame  ),
          'rporwq' : ( 'Init', ifsm_do_rearep      ),	# Just a guess
          'errfrm' : ( 'RxEf', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        }
      },
      'egress' :
      { 'Init':				# State = Initial
        { 'rcvpq'  : ( 'Init', efsm_do_nothing     ),
          'rcvrq'  : ( 'Init', efsm_do_nothing     ),
          'rcvwq'  : ( 'Init', efsm_do_nothing     ),
          'rcvxd'  : ( 'Init', efsm_do_nothing     ),
          'reareq' : ( 'Init', efsm_ignore_frame   ),
          'rearep' : ( 'Init', efsm_ignore_frame   ),
          'wrireq' : ( 'Init', efsm_ignore_frame   ),
          'wrirep' : ( 'Init', efsm_ignore_frame   ),
          'rporwq' : ( 'Init', efsm_ignore_frame   )
        }
      }
    },

    EMSBUS_MODE_PARTCPT:
    { 'ingress' :
      { 'Init' :			# State = Initial
        { 'polreq' : ( 'Init', ifsm_passon_polreq  ),
          'polrep' : ( 'Init', ifsm_check_polrep   ),
          'reareq' : ( 'Init', ifsm_passon_reareq  ),
          'rearep' : ( 'Init', ifsm_passon_rearep  ),
          'xmtrq'  : ( 'XmRq', ifsm_do_nothing     ),	# Start timer too
          'wrireq' : ( 'Init', ifsm_passon_wrireq  ),
          'wrirep' : ( 'Init', ifsm_ignore_frame   ),
          'xmtwq'  : ( 'XmWq', ifsm_do_nothing     ),   # Start timer too
          'rporwq' : ( 'Init', ifsm_do_wrireq      ),
          'errfrm' : ( 'Init', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        },
        'XmRq' :			# State = Xmitted ReadRequest
        { 'polreq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'polrep' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'reareq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'rearep' : ( 'Init', ifsm_reprxd_and_forf),
          'xmtrq'  : ( 'XmRq', ifsm_report_error   ),
          'wrireq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'wrirep' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'xmtwq'  : ( 'XmRq', ifsm_report_error   ),
          'rporwq' : ( 'XmRq', ifsm_do_rearep      ),
          'errfrm' : ( 'Init', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        },
        'XmWq' :			# State = Xmitted WriteRequest
        { 'polreq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'polrep' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'reareq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'rearep' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'xmtrq'  : ( 'XmWq', ifsm_report_error   ),
          'wrireq' : ( 'XmWq', ifsm_reppe_and_ignf ),	# Error
          'wrirep' : ( 'Init', ifsm_repwxd_and_forf),
          'xmtwq'  : ( 'XmWq', ifsm_report_error   ),
          'rporwq' : ( 'Init', ifsm_reppe_and_ignf ),	# Error
          'errfrm' : ( 'Init', ifsm_ignore_frame   ),
          'timout' : ( 'Init', ifsm_do_nothing     )
        }
      },
      'egress' :
      { 'Init':				# State = Initial
        { 'rcvpq'  : ( 'Init' , efsm_send_polrep    ),
          'rcvrq'  : ( 'WiRp' , efsm_do_nothing     ),
          'rcvwq'  : ( 'WiWp' , efsm_do_nothing     ),
          'rcvxd'  : ( 'Init' , efsm_report_error   ),
          'reareq' : ( 'WePq' , efsm_buffer_frame   ),
          'rearep' : ( 'WePq' , efsm_buffer_frame   ),
          'wrireq' : ( 'WePq' , efsm_buffer_frame   ),
          'wrirep' : ( 'Init' , efsm_reppe_and_ignf ),
          'rporwq' : ( 'Init' , efsm_do_wrireq      ),
          'bufemp' : ( 'Init' , efsm_do_nothing     ),
          'timout' : ( 'Init' , efsm_do_nothing     )
        },
        'WiRp':				# State = Waiting for internal rearep
        { 'rcvpq'  : ( 'Init' , efsm_send_polrep    ),
          'rcvrq'  : ( 'WiRp' , efsm_report_error   ),
          'rcvwq'  : ( 'WiRp' , efsm_report_error   ),
          'rcvxd'  : ( 'WiRp' , efsm_report_error   ),
          'reareq' : ( 'WiRpb', efsm_buffer_frame   ),
          'rearep' : ( 'Init' , efsm_forward_frame  ),
          'wrireq' : ( 'WiRp' , efsm_reppe_and_ignf ),
          'wrirep' : ( 'WiRp' , efsm_reppe_and_ignf ),
          'rporwq' : ( 'WiRp' , efsm_do_rearep      ),
          'bufemp' : ( 'Init' , efsm_ignore_frame   ),
          'timout' : ( 'Init' , efsm_do_nothing     )
        },
        'WiRpb':			# State = Waiting for internal rearep
        { 'rcvpq'  : ( 'WePq' , efsm_send_polrep    ),
          'rcvrq'  : ( 'WiRpb', efsm_report_error   ),
          'rcvwq'  : ( 'WiRpb', efsm_report_error   ),
          'rcvxd'  : ( 'WiRpb', efsm_report_error   ),
          'reareq' : ( 'WiRpb', efsm_buffer_frame   ),
          'rearep' : ( 'WePq' , efsm_forward_frame  ),
          'wrireq' : ( 'WiRpb', efsm_reppe_and_ignf ),
          'wrirep' : ( 'WiRpb', efsm_reppe_and_ignf ),
          'rporwq' : ( 'WiRpb', efsm_do_rearep      ),
          'bufemp' : ( 'WiRpb', efsm_report_error   ),
          'timout' : ( 'WePq' , efsm_do_nothing     )
        },
        'WiWp':				# State = Waiting for internal wrirep
        { 'rcvpq'  : ( 'Init' , efsm_send_polrep    ),
          'rcvrq'  : ( 'WiRp' , efsm_report_error   ),
          'rcvwq'  : ( 'WiWp' , efsm_report_error   ),
          'rcvxd'  : ( 'WiWp' , efsm_report_error   ),
          'reareq' : ( 'WiWpb', efsm_buffer_frame   ),
          'rearep' : ( 'WiWpb', efsm_buffer_frame   ),
          'wrireq' : ( 'WiWpb', efsm_buffer_frame   ),
          'wrirep' : ( 'Init' , efsm_forward_frame  ),
          'rporwq' : ( 'WiWp' , efsm_do_wrireq      ),
          'bufemp' : ( 'WiWp' , efsm_report_error   ),
          'timout' : ( 'Init' , efsm_do_nothing     )
        },
        'WiWpb':			# State = Waiting for internal wrirep
        { 'rcvpq'  : ( 'WePq' , efsm_send_polrep    ),
          'rcvrq'  : ( 'WiWpb', efsm_report_error   ),
          'rcvwq'  : ( 'WiWpb', efsm_report_error   ),
          'rcvxd'  : ( 'WiWpb', efsm_report_error   ),
          'reareq' : ( 'WiWpb', efsm_buffer_frame   ),
          'rearep' : ( 'WiWpb', efsm_buffer_frame   ),
          'wrireq' : ( 'WiRpb', efsm_buffer_frame   ),
          'wrirep' : ( 'WePq' , efsm_forward_frame  ),
          'rporwq' : ( 'WiRpb', efsm_do_wrireq      ),
          'bufemp' : ( 'WiWpb', efsm_report_error   ),
          'timout' : ( 'WePq' , efsm_do_nothing     )
        },
        'WePq':				# State = Waiting for external polreq
        { 'rcvpq'  : ( 'WeXd' , efsm_forward_buffer ),
          'rcvrq'  : ( 'WiRpb', efsm_do_nothing     ),
          'rcvwq'  : ( 'WiWpb', efsm_do_nothing     ),
          'rcvxd'  : ( 'WePq' , efsm_report_error   ),
          'reareq' : ( 'WePq' , efsm_buffer_frame   ),
          'rearep' : ( 'WePq' , efsm_buffer_frame   ),
          'wrireq' : ( 'WePq' , efsm_buffer_frame   ),
          'wrirep' : ( 'WePq' , efsm_reppe_and_ignf ),
          'rporwq' : ( 'WePq' , efsm_do_wrireq      ),
          'bufemp' : ( 'WePq' , efsm_report_error   ),
          'timout' : ( 'WePq' , efsm_do_nothing     ),
        },
        'WeXd':			# State = Waiting for transfer done
        { 'rcvpq'  : ( 'WeXd' , efsm_report_error   ),
          'rcvrq'  : ( 'WeXd' , efsm_report_error   ),
          'rcvwq'  : ( 'WeXd' , efsm_report_error   ),
          'rcvxd'  : ( 'WePq' , efsm_send_polrep    ),
          'reareq' : ( 'WeXd' , efsm_buffer_frame   ),
          'rearep' : ( 'WeXd' , efsm_buffer_frame   ),
          'wrireq' : ( 'WeXd' , efsm_buffer_frame   ),
          'wrirep' : ( 'WeXd' , efsm_reppe_and_ignf ),
          'rporwq' : ( 'WeXd' , efsm_do_wrireq      ),
          'bufemp' : ( 'WeXd' , efsm_report_error   ),
          'timout' : ( 'WePq' , efsm_send_polrep    ),
        }
      }
    }
  }

 #
 # Finite state machine state action vectors.
 # ------------------------------------------
 #
  FSMSA= { EMSBUS_MODE_MONITOR :
    { 'ingress' :
      { 'Init' : ifsa_start_wdt,
        'RxRq' : ifsa_stop_wdt,
        'RxWq' : ifsa_stop_wdt,
        'RxEf' : None
      },
      'egress' :
      { 'Init' : None }
    },

    EMSBUS_MODE_PARTCPT:
    { 'ingress' :
      { 'Init' : ifsa_start_wdt,
        'XmRq' : ifsa_stop_wdt,
        'XmWq' : ifsa_stop_wdt
      },
      'egress' :
      { 'Init' : efsa_start_wdt_ir,
        'WiRp' : efsa_stop_wdt_ir,
        'WiRpb': efsa_stop_wdt_ir,
        'WiWp' : efsa_stop_wdt_ir,
        'WiWpb': efsa_stop_wdt_ir,
        'WePq' : efsa_start_wdt_er,
        'WeXd' : efsa_stop_wdt_er
      }
    }
  }

 #
 # Private methods.
 # ----------------
 #
 # Private method _calc_checksum computes a check-sum of a given frame. Note
 # that in the available (reverse engineered) documentation this check-sum is
 # referred to as a CRC. However, it is not a CRC! The algorithm is taken from
 # URL https://github.com/danimaciasperea/Calduino.
 #
  def _calc_checksum( self, bfr ):
    mask= 0x0c				# Checksum mask
    chks= 0x00				# Checksum preset
    if len(bfr) < 2:
      raise ValueError( "Frame too short for check-sum calculation" )

    for i in range( len(bfr)-1 ):
      chks = ((chks^mask) << 1) | 0x01  if chks & 0x80  else chks << 1
      chks^= bfr[i]

    return chks & 0xff

  def _flush_iframe( self ):
    self.iframe= bytearray()
    self.iframe_error= 0
    self.iframe_time = None

 #
 # Private method _handle_iframe performs some basic handling of ingress frames.
 # It will handle framing errors, restore \xff octets, suppress the echo of an
 # egress frame, verify the check-sum, discard the frame if the check-sum is not
 # correct and finally, if all is well, pass it on to the appropriate queue.
 # Additionally, the frame counters are updated.
 #
 # Note that the handling of a frame by this method should be completed well
 # before the next octet of data can arrive, thus well within 1 [ms].
 #
 # The octet counters are incremented with an additional octet, to account for
 # the duration of the break, which lasts for at least one octet-time.
 #
 # Idea: should an error indication be forwarded in case of a erred frame? The
 # ingress dispatcher can reset it's state in such a case.
 #
  def _handle_iframe( self ):
  #
  # Do character handling: remove the framing error indicators and replace the
  # escaped 0xff characters by their original value.
  #
    self.iframe_error= 0
    octpos= self.iframe.find( SERIAL_Escape )
    while octpos >= 0:
      octpos+= 1
      if octpos >= len(self.iframe):
        break
      if   self.iframe[octpos] == SERIAL_Escape:
        if octpos+1 < len(self.iframe):
          self.iframe= self.iframe[:octpos] + self.iframe[octpos+1:]
        else:
          self.iframe= self.iframe[:octpos]
          break
      elif self.iframe[octpos] == SERIAL_FrmErr:
        if octpos+1 < len(self.iframe):
          self.iframe= self.iframe[:octpos-1] + self.iframe[octpos+1:]
          self.iframe_error+= 1
      octpos= self.iframe.find( SERIAL_Escape, octpos )

  #
  # In case an echo of our own transmission (sent by method writer) is received,
  # ignore the frame and do not update the (ingress) statistics. This frame is
  # already accounted for in the egress path.
  # To do: check the time of arrival against the time of transmission.
  #
    if self.eframe is not None:
      if self.iframe == self.eframe:
        self.eframe= None		# Match only once
        self._flush_iframe()
        self.sbs['ingress_echo_frames']+= 1
        return				# Ignore frame
#     elif len(self.eframe) > EMSBUS_Min_frame_size  and  len(self.iframe) == len(self.eframe):
#       if self.iframe[-1] != self._calc_checksum( self.iframe ):
#         self.eframe= None		# Match only once
#         self._flush_iframe()
#         return			# Ignore frame

    self.sbs['ingress_total_frames']+= 1
    self.sbs['ingress_total_octets']+= len(self.iframe) + 1

  #
  # If an error was found in (at least) one of the octets, update the error
  # counters, reset the error flag and forget about this frame.
  #
    if self.iframe_error > 0:
      self.sbs['ingress_err_frames']+= 1
      self.sbs['ingress_err_octets']+= len(self.iframe) + 1
      self.sbs['ingress_errors'    ]+= self.iframe_error
      self.iframe= b'ERR'
      self._queue_iframe()
#     self._flush_iframe()
      return				# Ignore erred frame

    if len(self.iframe) == 0:
      self.sbs['ingress_empty_frames']+= 1
      self._flush_iframe()
      return				# Ignore empty frame

  #
  # Handle a frame of one octet. It can be be either a poll request, a poll
  # reply or a write reply.
  #
    elif len(self.iframe) == 1:
      self._queue_iframe()

    elif len(self.iframe) <= EMSBUS_Min_frame_size:
      if self.idisp_log is not None:
        self.idisp_log( self.idisp_log_slf, self.iframe_time,
                        self.iframe, None )
      self.sbs['ingress_short_frames']+= 1
      self.iframe= b'ERR'
      self._queue_iframe()
 #    self._flush_iframe()

  #
  # Handle a 'normal' frame, that is a read request, a read reply or a write
  # request. A frame with a check-sum error is counted and ignored.
  #
    elif self.iframe[-1] == self._calc_checksum( self.iframe ):
      if self.iframe[2] >= 0xf0:	# Check type field
        self.sbs['ingress_emsplus_frames']+= 1
      del self.iframe[-1]		# Remove checksum
      self._queue_iframe()
    else:
      if self.idisp_log is not None:
        self.idisp_log( self.idisp_log_slf, self.iframe_time,
                        self.iframe, self._calc_checksum(self.iframe) )
      self.sbs['ingress_err_frames']+= 1
      self.sbs['ingress_err_octets']+= len(self.iframe) + 1
      self.iframe= b'ERR'
      self._queue_iframe()
#     self._flush_iframe()
    return

 #
 # Private method _log_message writes a message to a syslog file.
 #
  def _log_message( self, Msg ):
    syslog.openlog( 'EMS', 0, syslog.LOG_LOCAL6 )
    syslog.syslog ( ' '.join( (self.name,Msg) ) )
    syslog.closelog()

 #
 # Private method _queue_iframe moves (a copy of) the received frame to a queue
 # local to this object.
 #
  def _queue_iframe( self ):
    qv= { 'Frame': self.iframe, 'Time': self.iframe_time }
    self.iframe_queue.put( qv )
    self._flush_iframe()

 #
 # Private method _start_threads starts four threads. The order is significant,
 # as the ingress dispatcher should be started before the reader, and the writer
 # should be started before the egress dispatcher.
 #
  def _start_threads( self ):
   # Start the thread which dispatches the frames assembled by thread reader.
    self.idisp_alive = True
    self.idisp_thread= threading.Thread( target=self.ingress_dispatcher, name='id' )
    self.idisp_thread.start()

   # Start the thread which assembles frames from the octets received on the
   # serial port.
    self.reader_alive = True
    self.reader_thread= threading.Thread( target=self.reader, name='rx' )
    self.reader_thread.start()

   # Start the thread which writes frames to the serial port.
    self.writer_alive = True
    self.writer_thread= threading.Thread( target=self.writer, name='tx' )
    self.writer_thread.start()

   # Start the thread which dispatches frames received from the user
   # application.
    self.edisp_alive= True
    self.edisp_thread= threading.Thread( target=self.egress_dispatcher, name='ed' )
    self.edisp_thread.start()

 #
 # Private method _stop_threads stops the threads in the reverse order of
 # activation. It will wait for a clean exit of the threads.
 #
  def _stop_threads( self ):
  # Stop thread egress_dispatcher.
    self.edisp_alive= False
    if self.edisp_thread.is_alive():
      self.edisp_queue.put( None )
      self.edisp_thread.join()

  # Stop thread writer.
    self.writer_alive= False
    if self.writer_thread.is_alive():
      self.eframe_queue.put( None )
      self.writer_thread.join()

   # Stop thread reader.
    self.reader_alive= False
    if self.reader_thread.is_alive():
      if hasattr( self.serial, 'cancel_read' ):
        self.serial.cancel_read()
      self.reader_thread.join()

   # Stop thread ingress_dispatcher.
    self.idisp_alive= False
    if self.idisp_thread.is_alive():
      self.idisp_queue.put( None )
      self.idisp_thread.join()

   # Empty the internal queues.

 #
 # Public methods.
 # ---------------
 #
 # Method close reverses the operations of method open.
 #
  def close( self ):
    self.serial.close()			# Close serial port
    self._stop_threads()		# Stop all threads

  def get_mode( self ):
    mode= [ 'Null', 'Monitor', 'Participate', 'Participate & monitor' ]
    return mode[self.mode]

 #
 # Method get_statistics returns the collected statistics.
 #
  def get_statistics( self ):
    return self.sbs.copy()		# Return a copy of the statistics

 #
 # Method log_erred_frames accepts the object instance and a call-back method of
 # that instance. In case a frame is recieved of which the checksum does not
 # match, this call-back is invoked as a method, passing the time of arrival,
 # the erred frame and the computed checksum.
 #
  def log_erred_frames( self, owner, callback ):
    self.idisp_log_slf= owner
    self.idisp_log    = callback

 #
 # Method open starts communications via the serial port. The serial port is by
 # default set to 9600N1. Using termios, iflag PARMRK is set. This causes breaks
 # to be entered as 3 octets ( 0xff, 0x00, 0x00 ) in the input stream. It also
 # causes octets with a framing error to become strings of 3 octets: 0xff, 0x00
 # and the erred octet. Note that a genuine 0xff octet is translated into two
 # octets: 0xff, 0xff.
 #
  def open( self ):
    try:
      self.serial= serial.Serial( port = SERIAL_Device )
    except serial.SerialException as e:
      self._log_message( "Could not open port {}: {}".format(self.serial.name,e) )
      self.serial= None
      return None

   # Set iflag PARMRK to enable break detection and reporting.
    attr= termios.tcgetattr( self.serial.fd )
    attr[0]|= termios.PARMRK
    termios.tcsetattr( self.serial.fd, termios.TCSANOW, attr )

    self._start_threads()
    return self.serial

 #
 # Method read_frame waits until a frame is put onto the internal queue, and
 # passes this frame to the caller.
 #
  def read_frame( self ):
    qv= self.idisp_queue.get()
#   self.idisp_queue.task_done()
    return qv

 #
 # Methods which are started as seperate threads.
 # ----------------------------------------------
 #
 # Method egress_dispatcher is started as a seperate thread. It will recieve
 # frames from the user application, via queue edisp_queue, and it will send
 # them at the time this device is polled by the EMSbus master. Therefore the
 # frames are stored in a local queue untill the poll arrives.
 #
  def egress_dispatcher( self ):
    while self.edisp_alive:
      frame= self.edisp_queue.get()
      lf   = len(frame)
      type = None
      if   lf == 1:
        if frame[0] in EMS_Write_reply:
          type= 'wrirep'
      elif lf == 2:			# A signal from ingress-dispatcher
        type= 'rcv' + frame.lower()
#       frame= None			# Ignore 'frame'
      elif lf < EMSBUS_Min_frame_size:
        self.sbs['egress_total_frames']    += 1
        self.sbs['egress_total_octets']    += lf + 1
        self.sbs['egress_err_short_frames']+= 1
#       frame= None
      elif lf > EMSBUS_Max_frame_size:
        self.sbs['egress_total_frames']   += 1
        self.sbs['egress_total_octets']   += lf + 1
        self.sbs['egress_err_long_frames']+= 1
#       frame= None
      else:
        if   frame[EMS_Destin] == 0x00:		# Certainly a read reply
          type= 'rearep'
        elif frame[EMS_Destin]  & 0x80:		# A read request
          type= 'reareq'
        else:
          type= 'rporwq'			# A write request

      if type is not None:
        self.edisp_frame= frame
        self.edisp_type = type
        self.edisp_fsm.HandleEvent( type )	# Invoke FSM
#     self.edisp_queue.task_done()		# Signal completion frame handling

 #
 # Method ingress_dispatcher is started as a seperate thread. It will take
 # frames assembled by thread reader, determine their type, pass them to the
 # ingress FSM which will complete type determination and forward the frames to
 # the appropiate destination, informing the egress FSM as necessary.
 #
  def ingress_dispatcher( self ):
    while self.idisp_alive:
      qv= self.iframe_queue.get()
      frame= qv['Frame']
  #
  # Try to determine the type of frame.
  # As a read_reply cannot be distinguished from a write_request at this time,
  # the distinction between the two is handled by the FSM, which has the context
  # information to make that decision.
  #
      if len(frame) == 1:
        if frame[0] & 0x80:			# A poll request
          qv['Type']= 'polreq'
          self.sbs['ingress_polreq_frames']+= 1
        elif frame[0] in EMS_Write_reply:	# A write reply
          qv['Type']= 'wrirep'
          self.sbs['ingress_wrirep_frames']+= 1
        else:					# A poll reply
          qv['Type']= 'polrep'
          self.sbs['ingress_polrep_frames']+= 1
          if self.mode != EMSBUS_MODE_MONITOR:
            if frame[0] == self.device:
              self.sbs['bus_address_conflict']+= 1
      elif len(frame) == 2:			# Signals from egress_dispatcher
        qv['Type']= 'xmt' + frame.lower()
      elif frame == b'ERR':
        qv['Type']= 'errfrm'			# A frame with error(s)
      else:
        if   frame[EMS_Destin] == 0x00:		# A read reply (broadcast)
          qv['Type']= 'rearep'
          self.sbs['ingress_rearep_frames']+= 1
        elif frame[EMS_Destin]  & 0x80:		# A read request
          qv['Type']= 'reareq'
          self.sbs['ingress_reareq_frames']+= 1
        else:					# A read reply or write request
          qv['Type']= 'rporwq'
  #
  # Send the frame to the ingress Finite State Machine (FSM).
  #
      self.idisp_frame= qv			# Save frame and meta info
      self.idisp_fsm.HandleEvent( qv['Type'] )	# Invoke FSM
#     self.iframe_queue.task_done()		# Signal completion frame handling

 #
 # Method reader is started as a separate thread, which will capture octets from
 # the serial port (== EMS bus). The successive octets are assembled into a
 # frame: the end-of-frame is marked by the reception of a BREAK signal. Then
 # method _handle_iframe is invoked to handle each ingress frame, one by one.
 #
 # Note that octet value 0xff is used as an escape. It is the start of a BREAK
 # indicator and any octet with this value in the original data stream is
 # doubled. Same care must be taken if the original data stream happens to
 # contain the BREAK indicator. As the escape octet is doubled, the number of
 # escape octets directly preceeding the BREAK indicator show the difference
 # between octet stream resembling the BREAK indicator and a real one. In the
 # latter case the number of preceeding escape octets is even.
 #
  def reader( self ):
    self.serial.reset_input_buffer() ;	# Forget history
    self.iframe= bytearray()		# Empty ingress frame
    self.iframe_time = None		# Time of arrival of first octet of frame

    while True:
      data= self.serial.read(1)		# Wait till next octet arrives
      if not self.reader_alive:		# Stop when asked to
        break

      octcnt= self.serial.in_waiting	# Number of available octets
      if octcnt > 0:
        data+= self.serial.read(octcnt)	# A non-blocking read

      so= 0				# Search offset
      while len(data) > 0:
        if self.iframe_time == None:
          self.iframe_time= time.time()
        octcnt= data.find( SERIAL_Break, so )	# Location of next end-of-frame
        if octcnt == -1:
          self.iframe+= data		# Move partial frame
          data= b''			# Exit loop
        else:
          if octcnt > 0:
            self.iframe+= data[:octcnt]	# Move (last) part of frame
            data  = data[octcnt:]
            octcnt= 0
  # Count the number of consecutive 0xff octets preceding the end-of-frame
  # indicator. If this count is even, it is really an end-of-frame indicator.
  # However if this count is odd, the original octet stream consists of one or
  # more 0xff octets, followed by at least two 0x00 octets.
          cnt= 0
          if len(self.iframe) > 0  and  self.iframe[-1] == SERIAL_Escape:
            for i in range( len(self.iframe)-1, -1, -1 ):
              if self.iframe[i] == SERIAL_Escape:
                cnt+= 1
              else:
                break
          so= cnt % 2			# Set search offset
          if so == 0:			# If a real end-of-frame found
            octcnt+= len( SERIAL_Break )
            data= data[octcnt:]		# Skip delimiter
            self._handle_iframe()	# Do L2 handling of frame

 #
 # Method writer is started as a separate thread, which will transmit the
 # supplied frame(s) onto the serial port (== EMS bus). It will wait for a frame
 # to arrive on the internal queue eframe_queue, update the egress statistics
 # and send the frame immediately to the serial port.
 # It is up to the thread filling the aforementioned queue that the frame is put
 # onto this queue at a time this host is allowed to send that frame. In other
 # words, the half-duplex synchronization on the EMS bus is left to the
 # feeder(s) of queue eframe_queue.
 #
 # Note: at https://github.com/danimaciasperea/Calduino/blob/master/Calduino.cpp
 #  in method Calduino::sendBuffer, a wait time of 3 [ms] is inserted between
 #  two successive octets, and an additional wait time of 2 [ms] after the BREAK
 #  has been sent.
 #
  def writer( self ):
    self.serial.reset_output_buffer()
    ownl= ctypes.CDLL( './ownbreak.so' )	# Load C library
#   attr= termios.tcgetattr( self.serial.fd )	# Retrieve all serial attributes
#   attr[2]&= ~termios.PARODD			# Reset odd-parity-flag

    while self.writer_alive:
      qv= self.eframe_queue.get()
      self.eframe     = qv['Frame']
      self.eframe_time= time.time()

      type= qv['Type']
      self.sbs['egress_total_frames']+= 1
      self.sbs['egress_total_octets']+= len(self.eframe) + 1
      self.sbs['egress_{}_frames'.format(type)]+= 1

      if len( self.eframe ) >= EMSBUS_Min_frame_size:
        self.eframe.append( 0x00 )	# Append octet to contain the check-sum
        self.eframe[-1]= self._calc_checksum( self.eframe )
        self.sbs['egress_total_octets']+= 1

  # If a ReadRequest frame or a WriteRequest frame is to be sent, notify the
  # ingress_dispatcher that an associated reply is to be expected. The reception
  # of the reply is reported to the egress_dispatcher, which then can terminate
  # the current poll cycle in an orderly fashion.
      if   type == 'reareq':
        self.idisp_queue.put( 'RQ' )	# Notify ingress_dispatcher
      elif type == 'wrireq':
        self.idisp_queue.put( 'WQ' )	# Notify ingress_dispatcher

      t0= time.time()
      if self.device == 0x0b:		# Bus master will echo
        for x in self.eframe:
          self.serial.write( bytes([x]) )
          time.sleep( 0.0033 )		# Optimized for my system
      else:
        self.serial.write( self.eframe )	# Write / buffer frame
        time.sleep( len(self.eframe)/960.0 )	# Wait for completion

#     t1= time.time()
# # Send a break, by switching to even parity. Then an 11 bit character is sent,
# # which will be interpreted by the other members of the EMS-bus as a break
# # signal.
#     attr[2]|= termios.PARENB		# Enable (even) parity
#     termios.tcsetattr( self.serial.fd, termios.TCSANOW, attr )
#     t2= time.time()
#     self.serial.write( b'\x00' )	# Send a 'fast break'
#     time.sleep( 0.001 )
#     attr[2]&= ~termios.PARENB		# Disable (even) parity
#     termios.tcsetattr( self.serial.fd, termios.TCSANOW, attr )
  # Send a break signal. For a better timing, a C function is used.
      rc= ownl.sendshortbreak( self.serial.fd )
#     t2= time.time()
#     t2= t2 - t1 ;  t1= t1 - t0
#     print( '{:5.3f} {:5.3f}'.format(t1,t2) )
#     time.sleep( 0.002 )		# Wait some time for echo cancelation

  # If an unsollicited ReadReply has been sent, report the completion of this
  # transfer to the egress_dispatcher. It needs this signal to terminate the
  # current poll cycle in an orderly fashion.
      if type == 'rearep':
        if self.eframe[EMS_Destin] == 0x00:
          self.edisp_queue.put( 'XD' )	# Notify egress_dispatcher

#     self.eframe_queue.task_done()	# Report task done
