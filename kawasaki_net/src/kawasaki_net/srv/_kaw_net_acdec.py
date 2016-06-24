"""autogenerated by genpy from kawasaki_net/kaw_net_acdecRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class kaw_net_acdecRequest(genpy.Message):
  _md5sum = "52b83a6e48666aa66c64b047f6134d69"
  _type = "kawasaki_net/kaw_net_acdecRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 acdec
bool always
bool accel

"""
  __slots__ = ['acdec','always','accel']
  _slot_types = ['float32','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       acdec,always,accel

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(kaw_net_acdecRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.acdec is None:
        self.acdec = 0.
      if self.always is None:
        self.always = False
      if self.accel is None:
        self.accel = False
    else:
      self.acdec = 0.
      self.always = False
      self.accel = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_f2B.pack(_x.acdec, _x.always, _x.accel))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 6
      (_x.acdec, _x.always, _x.accel,) = _struct_f2B.unpack(str[start:end])
      self.always = bool(self.always)
      self.accel = bool(self.accel)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_f2B.pack(_x.acdec, _x.always, _x.accel))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 6
      (_x.acdec, _x.always, _x.accel,) = _struct_f2B.unpack(str[start:end])
      self.always = bool(self.always)
      self.accel = bool(self.accel)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_f2B = struct.Struct("<f2B")
"""autogenerated by genpy from kawasaki_net/kaw_net_acdecResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class kaw_net_acdecResponse(genpy.Message):
  _md5sum = "b3e7d1c3a90b7a5cc4ccd286ac981f72"
  _type = "kawasaki_net/kaw_net_acdecResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 Result


"""
  __slots__ = ['Result']
  _slot_types = ['int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       Result

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(kaw_net_acdecResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.Result is None:
        self.Result = 0
    else:
      self.Result = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_struct_b.pack(self.Result))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.Result,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_struct_b.pack(self.Result))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.Result,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
class kaw_net_acdec(object):
  _type          = 'kawasaki_net/kaw_net_acdec'
  _md5sum = 'c61428e9ff2bb38cb561d46ddb522e17'
  _request_class  = kaw_net_acdecRequest
  _response_class = kaw_net_acdecResponse
