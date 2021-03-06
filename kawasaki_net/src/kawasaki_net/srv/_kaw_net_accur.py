"""autogenerated by genpy from kawasaki_net/kaw_net_accurRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class kaw_net_accurRequest(genpy.Message):
  _md5sum = "a82e6eab28d655cbb197d95fa08810b8"
  _type = "kawasaki_net/kaw_net_accurRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 acc
bool always

"""
  __slots__ = ['acc','always']
  _slot_types = ['float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       acc,always

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(kaw_net_accurRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.acc is None:
        self.acc = 0.
      if self.always is None:
        self.always = False
    else:
      self.acc = 0.
      self.always = False

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
      buff.write(_struct_fB.pack(_x.acc, _x.always))
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
      end += 5
      (_x.acc, _x.always,) = _struct_fB.unpack(str[start:end])
      self.always = bool(self.always)
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
      buff.write(_struct_fB.pack(_x.acc, _x.always))
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
      end += 5
      (_x.acc, _x.always,) = _struct_fB.unpack(str[start:end])
      self.always = bool(self.always)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_fB = struct.Struct("<fB")
"""autogenerated by genpy from kawasaki_net/kaw_net_accurResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class kaw_net_accurResponse(genpy.Message):
  _md5sum = "b3e7d1c3a90b7a5cc4ccd286ac981f72"
  _type = "kawasaki_net/kaw_net_accurResponse"
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
      super(kaw_net_accurResponse, self).__init__(*args, **kwds)
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
class kaw_net_accur(object):
  _type          = 'kawasaki_net/kaw_net_accur'
  _md5sum = 'b78542ed5aa44b1c2e048200b592267b'
  _request_class  = kaw_net_accurRequest
  _response_class = kaw_net_accurResponse
