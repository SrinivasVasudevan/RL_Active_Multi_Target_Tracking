# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mbam_interfaces:msg/AgentObservation.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'z_world_x'
# Member 'z_world_y'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AgentObservation(type):
    """Metaclass of message 'AgentObservation'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mbam_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbam_interfaces.msg.AgentObservation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__agent_observation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__agent_observation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__agent_observation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__agent_observation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__agent_observation

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AgentObservation(metaclass=Metaclass_AgentObservation):
    """Message class 'AgentObservation'."""

    __slots__ = [
        '_header',
        '_robot_id',
        '_pose_x',
        '_pose_y',
        '_pose_yaw',
        '_max_num_targets',
        '_z_world_x',
        '_z_world_y',
        '_visible',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'robot_id': 'int32',
        'pose_x': 'double',
        'pose_y': 'double',
        'pose_yaw': 'double',
        'max_num_targets': 'int32',
        'z_world_x': 'sequence<double>',
        'z_world_y': 'sequence<double>',
        'visible': 'sequence<boolean>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.robot_id = kwargs.get('robot_id', int())
        self.pose_x = kwargs.get('pose_x', float())
        self.pose_y = kwargs.get('pose_y', float())
        self.pose_yaw = kwargs.get('pose_yaw', float())
        self.max_num_targets = kwargs.get('max_num_targets', int())
        self.z_world_x = array.array('d', kwargs.get('z_world_x', []))
        self.z_world_y = array.array('d', kwargs.get('z_world_y', []))
        self.visible = kwargs.get('visible', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.robot_id != other.robot_id:
            return False
        if self.pose_x != other.pose_x:
            return False
        if self.pose_y != other.pose_y:
            return False
        if self.pose_yaw != other.pose_yaw:
            return False
        if self.max_num_targets != other.max_num_targets:
            return False
        if self.z_world_x != other.z_world_x:
            return False
        if self.z_world_y != other.z_world_y:
            return False
        if self.visible != other.visible:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'robot_id' field must be an integer in [-2147483648, 2147483647]"
        self._robot_id = value

    @builtins.property
    def pose_x(self):
        """Message field 'pose_x'."""
        return self._pose_x

    @pose_x.setter
    def pose_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pose_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pose_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pose_x = value

    @builtins.property
    def pose_y(self):
        """Message field 'pose_y'."""
        return self._pose_y

    @pose_y.setter
    def pose_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pose_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pose_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pose_y = value

    @builtins.property
    def pose_yaw(self):
        """Message field 'pose_yaw'."""
        return self._pose_yaw

    @pose_yaw.setter
    def pose_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pose_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pose_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pose_yaw = value

    @builtins.property
    def max_num_targets(self):
        """Message field 'max_num_targets'."""
        return self._max_num_targets

    @max_num_targets.setter
    def max_num_targets(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'max_num_targets' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'max_num_targets' field must be an integer in [-2147483648, 2147483647]"
        self._max_num_targets = value

    @builtins.property
    def z_world_x(self):
        """Message field 'z_world_x'."""
        return self._z_world_x

    @z_world_x.setter
    def z_world_x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'z_world_x' array.array() must have the type code of 'd'"
            self._z_world_x = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'z_world_x' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._z_world_x = array.array('d', value)

    @builtins.property
    def z_world_y(self):
        """Message field 'z_world_y'."""
        return self._z_world_y

    @z_world_y.setter
    def z_world_y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'z_world_y' array.array() must have the type code of 'd'"
            self._z_world_y = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'z_world_y' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._z_world_y = array.array('d', value)

    @builtins.property
    def visible(self):
        """Message field 'visible'."""
        return self._visible

    @visible.setter
    def visible(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'visible' field must be a set or sequence and each value of type 'bool'"
        self._visible = value
