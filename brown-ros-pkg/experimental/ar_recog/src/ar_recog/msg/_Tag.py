"""autogenerated by genmsg_py from Tag.msg. Do not edit."""
import roslib.message
import struct


class Tag(roslib.message.Message):
  _md5sum = "8506b66f10a2975d80e32037f36b9ab4"
  _type = "ar_recog/Tag"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# All screen measurements are in pixels, all spatial measurements are in meters.
# Angles in radians.
uint32 id
# This is a rating of confidence in the tag pattern identification 0 < cf < 1.
float64 cf
uint32 x
uint32 y
# The 'diameter' is the square root of the tag's actual area, as estimated by 
# the AR software.  You can use it to check the cf confidence.
float64 diameter
# This is the estimated distance from viewer to the center of the tag.
float64 distance
float64 xRot 
float64 yRot 
float64 zRot
float64 xMetric
float64 yMetric
float64 zMetric
# Screen coordinates of the four corners.
float64[8] cwCorners
# FOR TESTING ONLY
# Uncommenting this and uncommenting the similarly-marked lines in 
# ar_recog.cpp will put the ARToolkit rotation matrix into the Tag message,
# which can be useful for debugging and testing.
# float64[3] c3
# float64[3] t0
# float64[3] t1
# float64[3] t2

"""
  __slots__ = ['id','cf','x','y','diameter','distance','xRot','yRot','zRot','xMetric','yMetric','zMetric','cwCorners']
  _slot_types = ['uint32','float64','uint32','uint32','float64','float64','float64','float64','float64','float64','float64','float64','float64[8]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       id,cf,x,y,diameter,distance,xRot,yRot,zRot,xMetric,yMetric,zMetric,cwCorners
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Tag, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.cf is None:
        self.cf = 0.
      if self.x is None:
        self.x = 0
      if self.y is None:
        self.y = 0
      if self.diameter is None:
        self.diameter = 0.
      if self.distance is None:
        self.distance = 0.
      if self.xRot is None:
        self.xRot = 0.
      if self.yRot is None:
        self.yRot = 0.
      if self.zRot is None:
        self.zRot = 0.
      if self.xMetric is None:
        self.xMetric = 0.
      if self.yMetric is None:
        self.yMetric = 0.
      if self.zMetric is None:
        self.zMetric = 0.
      if self.cwCorners is None:
        self.cwCorners = [0.,0.,0.,0.,0.,0.,0.,0.]
    else:
      self.id = 0
      self.cf = 0.
      self.x = 0
      self.y = 0
      self.diameter = 0.
      self.distance = 0.
      self.xRot = 0.
      self.yRot = 0.
      self.zRot = 0.
      self.xMetric = 0.
      self.yMetric = 0.
      self.zMetric = 0.
      self.cwCorners = [0.,0.,0.,0.,0.,0.,0.,0.]

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_Id2I8d.pack(_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot, _x.xMetric, _x.yMetric, _x.zMetric))
      buff.write(_struct_8d.pack(*self.cwCorners))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 84
      (_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot, _x.xMetric, _x.yMetric, _x.zMetric,) = _struct_Id2I8d.unpack(str[start:end])
      start = end
      end += 64
      self.cwCorners = _struct_8d.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_Id2I8d.pack(_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot, _x.xMetric, _x.yMetric, _x.zMetric))
      buff.write(self.cwCorners.tostring())
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 84
      (_x.id, _x.cf, _x.x, _x.y, _x.diameter, _x.distance, _x.xRot, _x.yRot, _x.zRot, _x.xMetric, _x.yMetric, _x.zMetric,) = _struct_Id2I8d.unpack(str[start:end])
      start = end
      end += 64
      self.cwCorners = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=8)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_8d = struct.Struct("<8d")
_struct_Id2I8d = struct.Struct("<Id2I8d")
