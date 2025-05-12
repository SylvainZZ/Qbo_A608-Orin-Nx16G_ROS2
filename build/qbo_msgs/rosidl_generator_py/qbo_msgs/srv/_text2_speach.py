# generated from rosidl_generator_py/resource/_idl.py.em
# with input from qbo_msgs:srv/Text2Speach.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Text2Speach_Request(type):
    """Metaclass of message 'Text2Speach_Request'."""

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
            module = import_type_support('qbo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'qbo_msgs.srv.Text2Speach_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__text2_speach__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__text2_speach__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__text2_speach__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__text2_speach__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__text2_speach__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Text2Speach_Request(metaclass=Metaclass_Text2Speach_Request):
    """Message class 'Text2Speach_Request'."""

    __slots__ = [
        '_sentence',
    ]

    _fields_and_field_types = {
        'sentence': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sentence = kwargs.get('sentence', str())

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
        if self.sentence != other.sentence:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sentence(self):
        """Message field 'sentence'."""
        return self._sentence

    @sentence.setter
    def sentence(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'sentence' field must be of type 'str'"
        self._sentence = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Text2Speach_Response(type):
    """Metaclass of message 'Text2Speach_Response'."""

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
            module = import_type_support('qbo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'qbo_msgs.srv.Text2Speach_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__text2_speach__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__text2_speach__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__text2_speach__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__text2_speach__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__text2_speach__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Text2Speach_Response(metaclass=Metaclass_Text2Speach_Response):
    """Message class 'Text2Speach_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_Text2Speach(type):
    """Metaclass of service 'Text2Speach'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('qbo_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'qbo_msgs.srv.Text2Speach')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__text2_speach

            from qbo_msgs.srv import _text2_speach
            if _text2_speach.Metaclass_Text2Speach_Request._TYPE_SUPPORT is None:
                _text2_speach.Metaclass_Text2Speach_Request.__import_type_support__()
            if _text2_speach.Metaclass_Text2Speach_Response._TYPE_SUPPORT is None:
                _text2_speach.Metaclass_Text2Speach_Response.__import_type_support__()


class Text2Speach(metaclass=Metaclass_Text2Speach):
    from qbo_msgs.srv._text2_speach import Text2Speach_Request as Request
    from qbo_msgs.srv._text2_speach import Text2Speach_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
