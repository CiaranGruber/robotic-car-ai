from __future__ import annotations

import math
import types
from dataclasses import fields, is_dataclass
from typing import Any, Self, Union, get_args, get_origin, get_type_hints


def serialise_float(value: float) -> float | str:
    """
    :param value: A floating-point number that may be non-finite.
    :return: The float, or NaN / Infinity / -Infinity as a string for strict JSON.
    """
    if math.isnan(value):
        return "NaN"
    if math.isinf(value):
        return "Infinity" if value > 0 else "-Infinity"
    return value


def deserialise_float(value: float | str) -> float:
    """
    :param value: A float, or a NaN / Infinity / -Infinity string from serialise.
    :return: The corresponding float value.
    """
    if value == "NaN":
        return math.nan
    if value == "Infinity":
        return math.inf
    if value == "-Infinity":
        return -math.inf
    return float(value)


def serialise_value(value: Any) -> Any:
    """
    :param value: A nested field value from a serialisable type.
    :return: JSON-friendly data.
    """
    if value is None or isinstance(value, (bool, int, str)):
        return value
    serialise = getattr(value, "serialise", None)
    if callable(serialise) and not isinstance(value, type):
        return serialise()
    if isinstance(value, float):
        return serialise_float(value)
    if isinstance(value, dict):
        return {str(key): serialise_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serialise_value(item) for item in value]
    return value


def deserialise_value(data: Any, type_hint: Any) -> Any:
    """
    :param data: JSON-friendly data produced by serialise.
    :param type_hint: The type annotation the data should become.
    :return: An instance matching type_hint.
    """
    origin = get_origin(type_hint)
    if origin is Union or origin is types.UnionType:
        non_none = [arg for arg in get_args(type_hint) if arg is not type(None)]
        if data is None:
            return None
        if len(non_none) != 1:
            raise TypeError(f"Unsupported union type hint: {type_hint!r}")
        return deserialise_value(data, non_none[0])
    if data is None:
        return None
    if origin is list:
        (element_type,) = get_args(type_hint)
        return [deserialise_value(item, element_type) for item in data]
    if origin is tuple:
        element_types = get_args(type_hint)
        if len(element_types) == 2 and element_types[1] is Ellipsis:
            return tuple(deserialise_value(item, element_types[0]) for item in data)
        return tuple(
            deserialise_value(item, element_type)
            for item, element_type in zip(data, element_types, strict=True)
        )
    if type_hint is Any:
        return data
    if type_hint is float:
        return deserialise_float(data)
    if type_hint is int:
        return int(data)
    if type_hint is str:
        return str(data)
    if type_hint is bool:
        return bool(data)
    if isinstance(type_hint, type) and hasattr(type_hint, "deserialise"):
        return type_hint.deserialise(data)
    raise TypeError(f"Cannot deserialise {data!r} as {type_hint!r}")


class Serialisable:
    """Mixin that converts instances to and from JSON-friendly data.

    Dataclass subclasses get a default serialise/deserialise based on their fields. Other types
    should override both methods.
    """

    def serialise(self) -> Any:
        """
        :return: JSON-friendly data for this instance.
        """
        if not is_dataclass(self):
            raise TypeError(f"{type(self).__name__} must implement serialise()")
        return {
            field.name: serialise_value(getattr(self, field.name))
            for field in fields(self)
        }

    @classmethod
    def deserialise(cls, data: Any) -> Self:
        """
        :param data: JSON-friendly data produced by serialise.
        :return: A new instance of this class.
        """
        if not is_dataclass(cls):
            raise TypeError(f"{cls.__name__} must implement deserialise()")
        if not isinstance(data, dict):
            raise TypeError(f"Expected dict for {cls.__name__}, got {type(data).__name__}")
        hints = get_type_hints(cls)
        return cls(**{
            field.name: deserialise_value(data[field.name], hints[field.name])
            for field in fields(cls)
        })
