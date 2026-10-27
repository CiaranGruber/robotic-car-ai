"""Decodes ROS 2 messages (CDR) from their .msg definitions, as a Foxglove bridge sends both.

A bridge advertises each topic with its schema name (such as dream_interfaces/msg/ConeDetections) and the
"ros2msg" text of the message and every type it uses, separated by "=====" lines and "MSG: package/Type" headers.
MessageDecoder reads that text once and then turns each message into nested dicts and lists. Constants (such as
ConeDetection.COLOR_BLUE) are kept per type, so callers can use the names instead of guessing values.
"""
from __future__ import annotations

import re
import struct

PRIMITIVES = {"bool": "?", "byte": "B", "char": "B", "int8": "b", "uint8": "B", "int16": "h", "uint16": "H",
              "int32": "i", "uint32": "I", "int64": "q", "uint64": "Q", "float32": "f", "float64": "d"}
"""struct format of each ROS 2 primitive type."""
BUILTIN_DEFINITIONS = {"builtin_interfaces/Time": [("int32", "sec", None, None), ("uint32", "nanosec", None, None)],
                       "builtin_interfaces/Duration": [("int32", "sec", None, None), ("uint32", "nanosec", None, None)]}
"""Types a definition may use without including them."""
_FIELD = re.compile(r"^([A-Za-z_][\w/]*)(?:<=\d+)?(?:\[(<=)?(\d*)\])?$")


def _type_name(name: str) -> str:
    """pkg/msg/Type and pkg/Type both become pkg/Type."""
    parts = name.split("/")
    return f"{parts[0]}/{parts[-1]}" if len(parts) >= 2 else name


class MessageDecoder:
    """Decodes one topic's messages."""

    def __init__(self, schema_name: str, definition: str):
        """
        :param schema_name: The advertised type, such as sensor_msgs/msg/LaserScan.
        :param definition: The advertised ros2msg text.
        """
        self.root = _type_name(schema_name)
        self.types: dict[str, list[tuple]] = dict(BUILTIN_DEFINITIONS)
        self.constants: dict[str, dict[str, str]] = {}
        sections = re.split(r"^=+\s*$", definition, flags=re.M)
        for index, section in enumerate(sections):
            lines = section.strip().splitlines()
            name = self.root
            if index > 0:
                header = next((line for line in lines if line.startswith("MSG:")), None)
                if header is None:
                    continue
                name = _type_name(header.split(":", 1)[1].strip())
            self._parse(name, [line for line in lines if not line.startswith("MSG:")])

    def _parse(self, name: str, lines: list[str]):
        package = name.split("/")[0]
        fields, constants = [], {}
        for raw in lines:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            tokens = line.split()
            if len(tokens) < 2:
                continue
            if "=" in tokens[1] or (len(tokens) > 2 and tokens[2].startswith("=")):
                key, _, value = line.split(None, 1)[1].partition("=")
                constants[key.strip()] = value.strip()
                continue
            match = _FIELD.match(tokens[0])
            if match is None:
                raise ValueError(f"cannot read field {line!r} of {name}")
            base, bounded, size = match.groups()
            if base not in PRIMITIVES and base not in ("string", "wstring"):
                base = "std_msgs/Header" if base == "Header" else _type_name(base) if "/" in base else f"{package}/{base}"
            array = None if "[" not in tokens[0] else "sequence" if bounded or not size else "fixed"
            fields.append((base, tokens[1], array, int(size) if array == "fixed" else None))
        self.types[name] = fields
        self.constants[name] = constants

    def decode(self, data: bytes) -> dict:
        """
        :param data: One CDR message, starting with its 4-byte encapsulation header.
        :return: The message as nested dicts and lists.
        """
        reader = _Reader(data)
        return reader.read(self.root, self.types)


class _Reader:
    def __init__(self, data: bytes):
        if len(data) < 4:
            raise ValueError("CDR message shorter than its header")
        self.data, self.position = data, 4
        self.endian = "<" if data[1] == 1 else ">"

    def _align(self, size: int):
        self.position += (-(self.position - 4)) % size

    def primitive(self, kind: str):
        fmt = PRIMITIVES[kind]
        size = struct.calcsize(fmt)
        self._align(size)
        value = struct.unpack_from(self.endian + fmt, self.data, self.position)[0]
        self.position += size
        return value

    def string(self) -> str:
        length = self.primitive("uint32")
        text = self.data[self.position:self.position + length].rstrip(b"\0").decode("utf-8", "replace")
        self.position += length
        return text

    def _one(self, kind: str, types: dict):
        if kind in PRIMITIVES:
            return self.primitive(kind)
        if kind in ("string", "wstring"):
            return self.string()
        return self.read(kind, types)

    def read(self, name: str, types: dict) -> dict:
        if name not in types:
            raise ValueError(f"no definition for {name}")
        message = {}
        for kind, field, array, size in types[name]:
            if array is None:
                message[field] = self._one(kind, types)
                continue
            count = self.primitive("uint32") if array == "sequence" else size
            if kind in PRIMITIVES and count:
                fmt = PRIMITIVES[kind]
                self._align(struct.calcsize(fmt))
                values = struct.unpack_from(f"{self.endian}{count}{fmt}", self.data, self.position)
                self.position += count * struct.calcsize(fmt)
                message[field] = list(values)
            else:
                message[field] = [self._one(kind, types) for _ in range(count)]
        return message
