"""autogenerated by genmsg_py from PathList.msg. Do not edit."""
import roslib.message
import struct

import eecs376_msgs.msg
import geometry_msgs.msg
import roslib.msg

class PathList(roslib.message.Message):
  _md5sum = "d011726f2e956efaa86429c23d6081da"
  _type = "eecs376_msgs/PathList"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """#Standard header
Header header

#List of path segments
eecs376_msgs/PathSegment[] path_list

================================================================================
MSG: roslib/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: eecs376_msgs/PathSegment
#Segment type
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
  __slots__ = ['header','path_list']
  _slot_types = ['Header','eecs376_msgs/PathSegment[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,path_list
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(PathList, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      if self.path_list is None:
        self.path_list = []
    else:
      self.header = roslib.msg._Header.Header()
      self.path_list = []

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.path_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.path_list:
        _x = val1
        buff.write(_struct_bId.pack(_x.seg_type, _x.seg_number, _x.seg_length))
        _v1 = val1.ref_point
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v2 = val1.init_tan_angle
        _x = _v2
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_d.pack(val1.curvature))
        _v3 = val1.max_speeds
        _v4 = _v3.linear
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = _v3.angular
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v6 = val1.min_speeds
        _v7 = _v6.linear
        _x = _v7
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v8 = _v6.angular
        _x = _v8
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_d.pack(val1.accel_limit))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.path_list = []
      for i in xrange(0, length):
        val1 = eecs376_msgs.msg.PathSegment()
        _x = val1
        start = end
        end += 13
        (_x.seg_type, _x.seg_number, _x.seg_length,) = _struct_bId.unpack(str[start:end])
        _v9 = val1.ref_point
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v10 = val1.init_tan_angle
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 8
        (val1.curvature,) = _struct_d.unpack(str[start:end])
        _v11 = val1.max_speeds
        _v12 = _v11.linear
        _x = _v12
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v13 = _v11.angular
        _x = _v13
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v14 = val1.min_speeds
        _v15 = _v14.linear
        _x = _v15
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v16 = _v14.angular
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 8
        (val1.accel_limit,) = _struct_d.unpack(str[start:end])
        self.path_list.append(val1)
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.path_list)
      buff.write(_struct_I.pack(length))
      for val1 in self.path_list:
        _x = val1
        buff.write(_struct_bId.pack(_x.seg_type, _x.seg_number, _x.seg_length))
        _v17 = val1.ref_point
        _x = _v17
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v18 = val1.init_tan_angle
        _x = _v18
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_struct_d.pack(val1.curvature))
        _v19 = val1.max_speeds
        _v20 = _v19.linear
        _x = _v20
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v21 = _v19.angular
        _x = _v21
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v22 = val1.min_speeds
        _v23 = _v22.linear
        _x = _v23
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v24 = _v22.angular
        _x = _v24
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        buff.write(_struct_d.pack(val1.accel_limit))
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
      if self.header is None:
        self.header = roslib.msg._Header.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.path_list = []
      for i in xrange(0, length):
        val1 = eecs376_msgs.msg.PathSegment()
        _x = val1
        start = end
        end += 13
        (_x.seg_type, _x.seg_number, _x.seg_length,) = _struct_bId.unpack(str[start:end])
        _v25 = val1.ref_point
        _x = _v25
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v26 = val1.init_tan_angle
        _x = _v26
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        start = end
        end += 8
        (val1.curvature,) = _struct_d.unpack(str[start:end])
        _v27 = val1.max_speeds
        _v28 = _v27.linear
        _x = _v28
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v29 = _v27.angular
        _x = _v29
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v30 = val1.min_speeds
        _v31 = _v30.linear
        _x = _v31
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v32 = _v30.angular
        _x = _v32
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 8
        (val1.accel_limit,) = _struct_d.unpack(str[start:end])
        self.path_list.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_d = struct.Struct("<d")
_struct_4d = struct.Struct("<4d")
_struct_3I = struct.Struct("<3I")
_struct_bId = struct.Struct("<bId")
_struct_3d = struct.Struct("<3d")
