#
# Definition of an as simple as possible Finite State Machine.
#
# A basic FSM (BFSM) consists of a list of states, a list of stimuli and a
# matrix (dictionary) with one entry per (state,stimulus)-pair specifying the
# new state and the action to be performed. The list of states and the list of
# stimuli are build from the supplied matrix.
#
# Stimuli can originate from a source external to the BFSM or from the BFSM
# itself. In principle stimuli from both sources are entered into the same queue
# (FIFO) within the BFSM, and are retrieved once the BFSM is ready to handle the
# next stimulus. Some internal stimuli are in fact modifiers on the stimulus
# being handled: these are passed via the high-priority queue.
#
# In addition to event actions, there are state actions. A state action depends
# only on the state. These are performed after entering a state and just before
# the event action is performed. If a state action returns a True value, the
# event action will be invoked too, but if it returns a False value, the event
# action will be skipped.
#
# For convenience a single watchdog-timer, named wdt, is created together with
# the FSM. It is not used within this class.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2018.01
#
import queue
import time
from threading import Timer

#
# Define a simple watchdog timer. A specific exception subclass is defined to
# handle a time-out for which no handler is defined.
#
class BfsmTimeoutException( Exception ):
  '''An unhandled time-out at BFSM.'''

class WatchdogTimer:
  '''A simple, stoppable and restartable watchdog timer.'''
  def __init__( self ):
    self.timeout= None			# Time out value [s]
    self.handler= None			# Call back function, parameterless
    self.timer  = None			# Timer object instance

  def _handler( self ):			# Default time-out handler
    raise BfsmTimeoutException

 # Method reset stops the timer if it is running, and creates and starts a new
 # one using the parameters passed to the previous invokation of method start.
  def reset( self ):			# Reset a running timer
    if self.timer is None:
      return False			# No timer defined
    if self.timer.is_alive():
      self.timer.cancel()
    self.timer= Timer( self.timeout, self.handler )
    self.timer.start()
    return True

 # Method start starts a new timer. If there was a timer already running, it is
 # stopped without further notice.
  def start( self, timeout, Handler=None ):	# Start a timer
    self.timeout= timeout
    self.handler= Handler	if Handler is not None else self._handler
    if self.timer is not None  and  self.timer.is_alive():
      self.timer.cancel()
    self.timer= Timer( self.timeout, self.handler )
    self.timer.start()

 # Method stop stops the timer if it is running. It returns true if the timer
 # is stopped, false if the timer already was expired.
  def stop( self ):			# Stop a timer
    if self.timer is not None  and  self.timer.is_alive():
      self.timer.cancel()
      return True
    else:
      return False


class Bfsm:
  '''A very basic Finite State Machine'''

  def __init__( self, Matrix, Owner, StaAct=None ):
    self.Matrix= Matrix			# Decision table of the FSM
    self.Owner = Owner			# Object 'owning' the actions
    self.StaAct= StaAct			# State pre- and post-action vector

   # Build a list of all possible states and a list of all possible stimuli, by
   # reading the keys of matrix Matrix.
    self.States = []			# Preset list of states
    self.Stimuli= []			# Preset list of stimuli
    for State in Matrix:
      self.States.append( State )
      for Stim in Matrix[State]:
        if not Stim in self.Stimuli:
          self.Stimuli.append( Stim )

   # Check the matrix to be complete, and see if the new state in each entry
   # does exist.
    for State in self.States:
#     assert State in Matrix		# This assertion should never fail
      for Stim in self.Stimuli:
        assert Stim in Matrix[State]
        assert Matrix[State][Stim][0] in self.States

   # If a list state actions is specified, make sure it contains an entry for
   # each state.
    if StaAct is not None:
      for State in self.States:
        self.StaAct.setdefault( State, None )
      for State in StaAct:
        assert State in self.States

   # Define the queues to pass the stimuli, both with normal and with high
   # priority. Preset the object variables.
    self.DefQueue= queue.Queue( 16 )
    self.PriQueue= queue.Queue(  2 )
    self.State   = 'Init'		# Current state
    self.PrvState= None			# Previous state
    self.NxtState= None			# Next state
    self.Stimulus= None			# Event
    self.wdt     = WatchdogTimer()	# Single timer per FSM

 # Private method _Interpret is the interpreter of the BFSM. It fetches the next
 # stimulus from either the high-prior queue or the default queue, determines
 # it's new state and invokes the associated action. If the stimuli queues are
 # empty the interpreter stops, that is it returns control to it's caller.
  def _Interpret( self ):
    while ( 1 ):
      Stim= None
      if   not self.PriQueue.empty():
        Stim=  self.PriQueue.get()
      elif not self.DefQueue.empty():
        Stim=  self.DefQueue.get()
      if Stim is None:
        return

      assert self.Stimuli.count( Stim ) == 1
      (NewState,Action)= self.Matrix[self.State][Stim][:]
      assert self.States.count( NewState ) == 1

      self.Stimulus= Stim
      if self.StaAct is not None:	# Do state action
        if self.StaAct[self.State] is not None:
          self.NxtState= NewState
          if not self.StaAct[self.State]( self.Owner ):
            continue
      Action( self.Owner )		# Invoke event action method
      self.PrvState= self.State
      self.State   = NewState
      self.NxtState= None

 # Method ReportEvent enters the supplied stimulus in the default queue and
 # returns. The stimulus will be handled when method HandleEvent is called.
  def ReportEvent( self, Stim ):
    assert self.Stimuli.count( Stim ) == 1
    self.DefQueue.put( Stim )

 # Method AugmentEvent enters the supplied stimulus in the high priority queue.
 # Typically, this method is called from an action invoked by this FSM, causing
 # this stimulus to be handled directly upon return from the action.
  def AugmentEvent( self, Stim ):
    assert self.Stimuli.count( Stim ) == 1
    self.PriQueue.put( Stim )

 # Method HandleEvent enters the supplied stimulus in the default queue and
 # invokes the FSM interpreter to handle the stimulus.
  def HandleEvent( self, Stim ):
    self.ReportEvent( Stim )
    self._Interpret()

 # Method GetState retrieves the current state and the last stimulus. Note that
 # the state will not change until the action associated with the aforementioned
 # pair (State,Stimulus) is completed.
  def GetState( self ):
    return (self.State,self.Stimulus)
