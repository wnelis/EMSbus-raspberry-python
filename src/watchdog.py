#
# WatchdogTimer.py
#
# This module contains the class definition of a simple watchdog timer. The
# timer can be started, stopped and reset. If the timer expires, a call back
# function is invoked.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2019.01
#
# Note: a timer is a separate thread. After invoking method cancel, the timer is
#   stopped (timer.finished.is_set() == True) but the timer thread may still be
#   alive (timer.is_alive() == True). The time between invoking cancel and the
#   timer thread becoming inactive should be small, even if a large timer value
#   is specified. To make sure that method is_alive will never return True while
#   the timer is stopped, each call to method cancel is followed by a call to
#   method join.
#
from threading import Timer

#
# Define a simple watchdog timer. A specific exception subclass is defined to
# handle a time-out for which no handler is defined.
#
class WdtTimeoutException( Exception ):
  '''An unhandled time-out of a watchdog timer.'''

class WatchdogTimer:
  '''A simple, stoppable and restartable watchdog timer.'''
  def __init__( self ):
    self.timeout= None			# Time out value [s]
    self.handler= None			# Call back function, parameter-less
    self.timer  = Timer( 0, True )	# Dummy timer object instance

  def _handler( self ):			# Default time-out handler
    raise WdtTimeoutException

 #
 # Private method _start contains the common part of methods start and reset. It
 # stops a timer if there is an active one and starts a new timer using the
 # parameters saved in the object.
 #
  def _start( self ):
    if self.timer.is_alive():
      self.timer.cancel()		# Stop timer
      self.timer.join()			# Wait for timer thread to finish

    self.timer= Timer( self.timeout, self.handler )
    self.timer.start()
    return True

 #
 # Method reset stops the timer if it is running, and creates and starts a new
 # one using the parameters passed to the previous invocation of method start.
 # If previously no timer was defined, this method does nothing and returns
 # False, while it returns True if the timer is restarted.
 #
  def reset( self ):			# Reset a running timer
    if self.timeout is None:
      return False			# Error: no timer defined
    return self._start()		# Stop if necessary and start a timer

 #
 # Method start starts a new timer. If there was a timer already running, it is
 # stopped without further notice. The returned value is False if no timeout is
 # specified, otherwise the returned value is True.
 #
  def start( self, timeout, Handler=None ):	# Start a timer
    if timeout is None:			# Check for an illegal value
      return False
    self.timeout= timeout		# Save parameters
    self.handler= Handler	if Handler is not None else self._handler
    return self._start()		# Start a new timer

 #
 # Method stop stops the timer if it is running. It returns True if the timer is
 # stopped, False if the timer already is expired.
 #
  def stop( self ):			# Stop a timer
    if self.timer.is_alive():
      self.timer.cancel()
      self.timer.join()
      return True
    else:
      return False

