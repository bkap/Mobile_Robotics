"""autogenerated by genmsg_py from PathSegment.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg

class PathSegment(roslib.message.Message):
  _md5sum = "420947086dd4a9b18e771af02b6b117e"
  _type = "eecs376_msgs/PathSegment"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#Segment type
int8 seg_type

#Segment number
uint32 seg_number

#Segment length
float64 seg_length

#Reference point
geometry_msgs/Point ref_point

#Initial tanget angle
geometry_msgs/Quaternion init_tan_angle

#Curvature
float64 curvature

#Speed limits for this segment
geometry_msgs/Twist max_speeds
geometry_msgs/Twist min_speeds

#Acceleration limit for this segment
float64 accel_limit

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into it's linear and angular parts. 
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['seg_type','seg_number','seg_length','ref_point','init_tan_angle','curvature','max_speeds','min_speeds','accel_limit']
  _slot_types = ['int8','uint32','float64','geometry_msgs/Point','geometry_msgs/Quaternion','float64','geometry_msgs/Twist','geometry_msgs/Twist','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       seg_type,seg_number,seg_length,ref_point,init_tan_angle,curvature,max_speeds,min_speeds,accel_limit
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(PathSegment, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.seg_type is None:
        self.seg_type = 0
      if self.seg_number is None:
        self.seg_number = 0
      if self.seg_length is None:
        self.seg_length = 0.
      if self.ref_point is None:
        self.ref_point = geometry_msgs.msg.Point()
      if self.init_tan_angle is None:
        self.init_tan_angle = geometry_msgs.msg.Quaternion()
      if self.curvature is None:
        self.curvature = 0.
      if self.max_speeds is None:
        self.max_speeds = geometry_msgs.msg.Twist()
      if self.min_speeds is None:
        self.min_speeds = geometry_msgs.msg.Twist()
      if self.accel_limit is None:
        self.accel_limit = 0.
    else:
      self.seg_type = 0
      self.seg_number = 0
      self.seg_length = 0.
      self.ref_point = geometry_msgs.msg.Point()
      self.init_tan_angle = geometry_msgs.msg.Quaternion()
      self.curvature = 0.
      self.max_speeds = geometry_msgs.msg.Twist()
      self.min_speeds = geometry_msgs.msg.Twist()
      self.accel_limit = 0.

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
      buff.write(_struct_bI22d.pack(_x.seg_type, _x.seg_number, _x.seg_length, _x.ref_point.x, _x.ref_point.y, _x.ref_point.z, _x.init_tan_angle.x, _x.init_tan_angle.y, _x.init_tan_angle.z, _x.init_tan_angle.w, _x.curvature, _x.max_speeds.linear.x, _x.max_speeds.linear.y, _x.max_speeds.linear.z, _x.max_speeds.angular.x, _x.max_speeds.angular.y, _x.max_speeds.angular.z, _x.min_speeds.linear.x, _x.min_speeds.linear.y, _x.min_speeds.linear.z, _x.min_speeds.angular.x, _x.min_speeds.angular.y, _x.min_speeds.angular.z, _x.accel_limit))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.ref_point is None:
        self.ref_point = geometry_msgs.msg.Point()
      if self.init_tan_angle is None:
        self.init_tan_angle = geometry_msgs.msg.Quaternion()
      if self.max_speeds is None:
        self.max_speeds = geometry_msgs.msg.Twist()
      if self.min_speeds is None:
        self.min_speeds = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 181
      (_x.seg_type, _x.seg_number, _x.seg_length, _x.ref_point.x, _x.ref_point.y, _x.ref_point.z, _x.init_tan_angle.x, _x.init_tan_angle.y, _x.init_tan_angle.z, _x.init_tan_angle.w, _x.curvature, _x.max_speeds.linear.x, _x.max_speeds.linear.y, _x.max_speeds.linear.z, _x.max_speeds.angular.x, _x.max_speeds.angular.y, _x.max_speeds.angular.z, _x.min_speeds.linear.x, _x.min_speeds.linear.y, _x.min_speeds.linear.z, _x.min_speeds.angular.x, _x.min_speeds.angular.y, _x.min_speeds.angular.z, _x.accel_limit,) = _struct_bI22d.unpack(str[start:end])
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
      buff.write(_struct_bI22d.pack(_x.seg_type, _x.seg_number, _x.seg_length, _x.ref_point.x, _x.ref_point.y, _x.ref_point.z, _x.init_tan_angle.x, _x.init_tan_angle.y, _x.init_tan_angle.z, _x.init_tan_angle.w, _x.curvature, _x.max_speeds.linear.x, _x.max_speeds.linear.y, _x.max_speeds.linear.z, _x.max_speeds.angular.x, _x.max_speeds.angular.y, _x.max_speeds.angular.z, _x.min_speeds.linear.x, _x.min_speeds.linear.y, _x.min_speeds.linear.z, _x.min_speeds.angular.x, _x.min_speeds.angular.y, _x.min_speeds.angular.z, _x.accel_limit))
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
      if self.ref_point is None:
        self.ref_point = geometry_msgs.msg.Point()
      if self.init_tan_angle is None:
        self.init_tan_angle = geometry_msgs.msg.Quaternion()
      if self.max_speeds is None:
        self.max_speeds = geometry_msgs.msg.Twist()
      if self.min_speeds is None:
        self.min_speeds = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 181
      (_x.seg_type, _x.seg_number, _x.seg_length, _x.ref_point.x, _x.ref_point.y, _x.ref_point.z, _x.init_tan_angle.x, _x.init_tan_angle.y, _x.init_tan_angle.z, _x.init_tan_angle.w, _x.curvature, _x.max_speeds.linear.x, _x.max_speeds.linear.y, _x.max_speeds.linear.z, _x.max_speeds.angular.x, _x.max_speeds.angular.y, _x.max_speeds.angular.z, _x.min_speeds.linear.x, _x.min_speeds.linear.y, _x.min_speeds.linear.z, _x.min_speeds.angular.x, _x.min_speeds.angular.y, _x.min_speeds.angular.z, _x.accel_limit,) = _struct_bI22d.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_bI22d = struct.Struct("<bI22d")
