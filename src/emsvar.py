#
# Define the variable types which are accessible via the EMSbus.
#
# To do:
# - Repeating fields
# - Repeating structures
#
import datetime
import time

#
# Define the offset of the header fields in a frame. The user data starts at
# offset EMS_Data and extends until the end of the frame. The offset in field
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
# Generic classes.
# ================
#

#
# Class emsvar is a generic class, serving as root for a small hierarchy of
# classes. It defines the common attributes of fields in frames on an EMSbus.
#
class emsvar():
  '''Generic class defining a field as found in frames on EMSbus'''

  def __init__( self, Name, Source, Type ):
    self.name = Name			# Name of field
    self.src  = Source			# EMSbus device identifier
    self.type = Type			# EMSbus frame type
    self.value= None			# Normalized value
    self.tom  = None			# Time of measurement

  def extract( self, frame, tom ):
    if frame[EMS_Source] != self.src:
     return False			# Extraction failed
    if frame[EMS_Type]   != self.type:
      return False			# Extraction failed
    return True

  def get_name( self ):
    return self.name

  def get_value( self ):
    return self.value

  def has_dimension( self ):
    return False


class emsbitvar( emsvar ):
  '''Generic class emsbitvar describes a flag, occupying one bit'''

  def __init__( self, Name, Source, Type, Offset, Bit ):
    super().__init__( Name, Source, Type )	# Parent level initialisation
    self.offset= Offset
    self.bitnbr= Bit
    self.value = None

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
 # Compute the offset range of the octets available in this frame. The range is
 # defined like a slice in Python.
      fao_start= frame[EMS_Offset]	# Frame available octets, start
      fao_end  = fao_start + len(frame) - EMS_Data
      if self.offset >= fao_start  and  self.offset < fao_end:
        self.actoffset= EMS_Data + self.offset - frame[EMS_Offset]
        return True
      else:
        return False			# Field is not available
    else:
      return False			# Extraction failed


class emsbytevar( emsvar ):
  '''Generic class emsbytevar describes a numeric field, occupying one or more octets'''

  def __init__( self, Name, Source, Type, Offset, Size, Divisor, Dimension ):
    super().__init__( Name, Source, Type )	# Parent level initialisation
    self.offset   = Offset
    self.size     = Size
    self.divisor  = Divisor
    self.dimension= Dimension
    self.actoffset= None

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
 # Compute the offset range of the octets available in this frame. The range is
 # defined like a slice in Python.
      fao_start= frame[EMS_Offset]	# Frame available octets, start
      fao_end  = fao_start + len(frame) - EMS_Data
      if self.offset >= fao_start  and  self.offset+self.size <= fao_end:
        self.actoffset= EMS_Data + self.offset - frame[EMS_Offset]
        return True
      else:
        return False			# Field is not available
    else:
      return False			# Extraction failed

  def get_dimension( self ):
    return self.dimension

  def has_dimension( self ):
    return True

#
# Specific  classes.
# ==================
#
# Class ems_ascii describes a field consisting one or more bytes with 7-bit
# ASCII.
#
class ems_ascii( emsvar ):
  '''Class ems_ascii describes a field containing 7-bit ASCII code'''

  def __init__( self, Name, Source, Type, Offset, Size ):
    super().__init__( Name, Source, Type )	# Parent level initialisation
    self.offset= Offset
    self.size  = Size

 #
 # Method extract checks if the field is available in this frame. If not, it
 # returns value False. If it is available, the octets in this field are
 # converted to a string.
 #
  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
  # Compute the offset range of the octets available in this frame. The range is
  # defined like a slice in Python.
      fao_start= frame[EMS_Offset]	# Frame available octets, start
      fao_end  = fao_start + len(frame) - EMS_Data
      if self.offset >= fao_start  and  self.offset+self.size <= fao_end:
        self.value= frame[EMS_Data+self.offset:EMS_Data+self.offset+self.size].decode('ascii')
        self.tom  = tom
        return True
      else:
        return False			# Field is not available
    else:
      return False			# Extraction failed


#
# Class ems_datetime decodes the date and time as broadcasted by the thermostat.
#
class ems_datetime( emsbytevar ):
  '''Class ems_datetime describes a date and time, with some additional flags.'''

  def __init__( self, Name, Source, Type, Offset, Size ):
    super().__init__( Name, Source, Type, Offset, Size, None, None )	# Parent level initialisation
    self.dtf= None

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
      ao= self.actoffset
      self.value= datetime.datetime( frame[ao]+2000, frame[ao+1], frame[ao+3], frame[ao+2], frame[ao+4], frame[ao+5] )
      self.dtf  = frame[ao+7]		# Date-time flags
      self.tom  = tom
      return True
    else:
      return False			# Extraction failed


class ems_flag( emsbitvar ):
  '''Class ems_flag describes a flag, with value either False (0) or True (1).'''

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
      self.value= frame[self.actoffset] & 1<<self.bitnbr != 0
      self.tom  = tom
      return True
    else:
      return False			# Extraction failed


#
# Class ems_numeric describes a field containing a numerical value. If the
# divisor is not specified or has the value 1, the result is a two's conplement
# integer value. If the divisor value is not 1, the result is a floating point
# number. This class can be used for both varying values (like the temperature)
# and for never-decreasing values (like the up time).
#
class ems_numeric( emsbytevar ):
  '''Class ems_numeric defines a field containing a numerical value'''

  def __init__( self, Name, Source, Type, Offset, Size, Divisor, Dimension ):
    super().__init__( Name, Source, Type, Offset, Size, Divisor, Dimension )

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
      ao= self.actoffset
      if self.size >= 2  and frame[ao] == 0x80  and frame[ao+1] == 0x00:
        self.value= None		# Missing sensor
      else:
        self.value= 0
        for x in frame[ao:ao+self.size]:
          self.value= self.value * 256 + x
        if frame[ao] & 0x80:		# If number is negative
          self.value-= 1 << 8*self.size	# Two's complement negative value
        if self.divisor is None  or  self.divisor == 1:
          self.value = int( self.value )
        else:
          self.value/= self.divisor
      self.tom= tom
      return True
    else:
      return False			# Extraction failed

#
# Class ems_sumima is meant for fields with a high update frequency. It
# determines the integral over time, the minimal and the maximal value. The
# latter two are reset when read.
#
class ems_sumima( ems_numeric ):
  '''Class ems_sumima defines a field with high update frequency. The integral
     over time is calculated, resulting in an ever increasing variable. Using
     the integral, the average value during the last measurement interval can be
     computed. Additionally, the minimal and maximal value are determined. The
     minimum and maximum are reset when the values are retrieved.'''

  def __init__( self, Name, Source, Type, Offset, Size, Divisor, Dimension ):
    super().__init__( Name, Source, Type, Offset, Size, Divisor, Dimension )
    self.prv_value= None
    self.prv_time = None
    self.sum_value= 0
    self.min_value= None
    self.max_value= None

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
      if self.value is None:
        self.value= self.sum_value
      else:
        if self.min_value is None:	# Update min and max
          self.min_value= self.value
          self.max_value= self.value
        else:
          self.min_value= min( self.min_value, self.value )
          self.max_value= max( self.max_value, self.value )

        if self.prv_value is not None:	# Update integral (sum)
          self.sum_value+= (self.value + self.prv_value)*0.5*(self.tom - self.prv_time )
        self.prv_value= self.value
        self.prv_time = self.tom
        self.value    = self.sum_value
      return True
    else:
      return False			# Extraction failed

  def get_range( self ):
    min= self.min_value
    max= self.max_value
    self.min_value= None
    self.max_value= None
    return (min,max)


class ems_switch( ems_flag ):
  '''Class ems_switch describes a flag, with value either 'Off' (0) or 'On' (1).'''

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
      self.value= 'On'	if self.value else 'Off'
      return True
    else:
      return False			# Extraction failed


#
# Class ems_version describes a two-byte field, containing the major and minor
# version numbers.
#
class ems_version( emsbytevar ):
  '''Class ems_version defines a two byte field, containing a major and a minor
     version number'''

  def __init__( self, Name, Source, Type, Offset, Size ):
    super().__init__( Name, Source, Type, Offset, Size, None, None )	# Parent level initialisation
    self.value= '?.??'

  def extract( self, frame, tom ):
    if super().extract( frame, tom ):
      ao= self.actoffset
      self.value= '{:d}.{:02d}'.format( frame[ao], frame[ao+1] )
      self.tom  = tom
      return True
    else:
      return False			# Extraction failed
