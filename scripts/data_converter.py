"""Convert logged observations and actions into scenario files.

Example::

    python -m scripts.data_converter sync
    python -m scripts.data_converter sync -d
    python -m scripts.data_converter convert -i messy_cone_test -n "Messy cone" -d "Staggered lane"

Pipe a descriptions document into sync::

    echo '[{"input_name": "messy_cone_test", "description": "A run"}]' | python -m scripts.data_converter sync
"""

from __future__ import annotations

import argparse
import ctypes
import json
import logging
import os
import select
import stat
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

if os.name == "nt":
    import msvcrt

_LOGGER = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[1]
"""Repository root. The script lives in scripts/."""

DEFAULT_INPUT_FOLDER = REPO_ROOT / "scenarios" / "data" / "original_data"
"""Default folder of logged observations and actions."""

DEFAULT_OUTPUT_FOLDER = REPO_ROOT / "scenarios" / "data" / "real_scenarios"
"""Default folder for converted scenario files."""

_OBSERVATIONS_SUFFIX = "_observations.json"
"""Filename suffix for a logged observations array."""

_ACTIONS_SUFFIX = "_actions.json"
"""Filename suffix for a logged actions array."""

_TEST_SUFFIX = "_test"
"""Suffix on an input stem, removed when deriving the output stem and display name."""

_SCENARIO_SUFFIX = "_scenario"
"""Suffix on a scenario stem."""

_DESCRIPTION_KEYS = frozenset({"input_name", "output_name", "name", "description"})
"""Fields accepted by a scenario description."""


class ScenarioConverterError(Exception):
    """A scenario conversion failed."""


def _strip_text(value: Any, label: str, *, allow_blank: bool = False) -> str:
    """
    :param value: A description field supplied by the caller.
    :param label: Field name used in the error message.
    :param allow_blank: When true, an empty string is returned as an empty string.
    :return: The stripped text.
    """
    if not isinstance(value, str):
        raise ScenarioConverterError(f"{label} must be a string")
    stripped = value.strip()
    if not stripped and not allow_blank:
        raise ScenarioConverterError(f"{label} must not be empty")
    return stripped


def _reject_unknown(fields: dict[str, Any]):
    """
    :param fields: Description fields to check against the known keys.
    """
    unknown = sorted(set(fields) - _DESCRIPTION_KEYS)
    if unknown:
        raise ScenarioConverterError(f"Unknown description fields: {', '.join(unknown)}")


def _without_suffix(value: str, suffix: str) -> str:
    """
    :param value: Text that may end with suffix.
    :param suffix: Suffix to remove once, when present.
    :return: value without that trailing suffix.
    """
    return value[: -len(suffix)] if value.endswith(suffix) else value


def _display_name(stem: str) -> str:
    """
    :param stem: A file stem whose underscores should become spaces.
    :return: The stem in sentence case.
    """
    words = stem.replace("_", " ").strip()
    return words[:1].upper() + words[1:].lower() if words else ""


def _name_from_input(input_name: str) -> str:
    """
    :param input_name: Stem of a logged observations and actions pair.
    :return: Display name produced by removing a trailing _test.
    """
    return _display_name(_without_suffix(input_name, _TEST_SUFFIX))


def _require_stem(value: str, label: str) -> str:
    """
    :param value: A filename stem that must not be a path.
    :param label: Field name used in the error message.
    :return: The same stem when it is a single path component.
    """
    if value != Path(value).name or "/" in value or "\\" in value:
        raise ScenarioConverterError(f"{label} must be a file stem, not a path")
    return value


@dataclass(frozen=True)
class ScenarioDescription:
    """Names and description for one converted scenario."""
    input_name: str
    output_name: str
    name: str
    description: str

    def __init__(self, **kwargs: Any):
        """Fill missing names from the fields that were provided.

        :param kwargs: Any subset of input_name, output_name, name and description.
        """
        _reject_unknown(kwargs)

        def field(key: str) -> str | None:
            if key not in kwargs:
                return None
            return _strip_text(kwargs[key], key)

        input_name = field("input_name")
        output_name = field("output_name")
        name = field("name")
        description = (
            _strip_text(kwargs["description"], "description", allow_blank=True)
            if "description" in kwargs
            else ""
        )

        # Derive any missing identity fields from the ones that were supplied.
        if name is None:
            if input_name is not None:
                name = _name_from_input(input_name)
            elif output_name is not None:
                name = _display_name(_without_suffix(output_name, _SCENARIO_SUFFIX))
        if input_name is None and name is not None:
            slug = "_".join(name.lower().split())
            input_name = slug if slug.endswith(_TEST_SUFFIX) else f"{slug}{_TEST_SUFFIX}"
        if output_name is None and input_name is not None:
            output_name = f"{_without_suffix(input_name, _TEST_SUFFIX)}{_SCENARIO_SUFFIX}"

        if not input_name or not output_name or not name:
            raise ScenarioConverterError("A description needs an input_name, output_name or name")

        object.__setattr__(self, "input_name", _require_stem(input_name, "input_name"))
        object.__setattr__(self, "output_name", _require_stem(output_name, "output_name"))
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "description", description)


def _validate_entry(item: Any) -> dict[str, str]:
    """
    :param item: One JSON value from a descriptions document.
    :return: The entry limited to known string fields.
    """
    if not isinstance(item, dict):
        raise ScenarioConverterError("Each description must be a JSON object")
    _reject_unknown(item)
    validated: dict[str, str] = {}
    for key in ("input_name", "output_name", "name", "description"):
        if key in item:
            validated[key] = _strip_text(item[key], key, allow_blank=(key == "description"))
    return validated


def _parse_json(text: str) -> Any:
    """
    :param text: JSON text from a descriptions document.
    :return: The parsed JSON value.
    """
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise ScenarioConverterError(f"Descriptions are not valid JSON: {exc}") from exc


def parse_descriptions(text: str) -> list[dict[str, str]]:
    """Parse a descriptions document.

    :param text: JSON array of description objects.
    :return: Validated description entries.
    """
    data = _parse_json(text)
    if not isinstance(data, list):
        raise ScenarioConverterError("Descriptions must be a JSON array")
    return [_validate_entry(item) for item in data]


def parse_convert_description(text: str) -> dict[str, str]:
    """Parse the one description accepted by convert.

    :param text: A JSON object, or a JSON array containing one object.
    :return: The validated description entry.
    """
    data = _parse_json(text)
    if isinstance(data, dict):
        return _validate_entry(data)
    if isinstance(data, list) and len(data) == 1:
        return _validate_entry(data[0])
    raise ScenarioConverterError("convert expects one description object or a one-item array")


def load_descriptions(descriptions_file: Path | None, descriptions_text: str | None) -> list[dict[str, str]]:
    """Load description entries from a file and from piped text.

    :param descriptions_file: Optional path of a descriptions document.
    :param descriptions_text: Optional piped descriptions document.
    :return: Entries from the file, followed by entries from the piped text.
    """
    entries: list[dict[str, str]] = []
    if descriptions_file is not None:
        if not descriptions_file.is_file():
            raise ScenarioConverterError(f"Descriptions file does not exist: {descriptions_file}")
        entries.extend(parse_descriptions(descriptions_file.read_text(encoding="utf-8")))
    if descriptions_text is not None and descriptions_text.strip():
        entries.extend(parse_descriptions(descriptions_text))
    return entries


def _load_json_file(path: Path, expected: type) -> Any:
    """
    :param path: JSON file to read.
    :param expected: Expected top-level type, typically list or dict.
    :return: The parsed JSON value.
    """
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScenarioConverterError(f"Cannot read {path}: {exc}") from exc
    if not isinstance(data, expected):
        kind = "array" if expected is list else "object"
        raise ScenarioConverterError(f"{path} must contain a JSON {kind}")
    return data


def convert_description(
    description: ScenarioDescription,
    input_folder: Path,
    output_folder: Path,
    dry_run: bool = False,
) -> Path:
    """Read one logged pair and write its scenario.

    :param description: Resolved names and description for the scenario.
    :param input_folder: Folder that contains the logged pair.
    :param output_folder: Folder that receives the scenario file.
    :param dry_run: When true, validate the pair and do not write the file.
    :return: The scenario path that was written, or would be written.
    """
    observations_path = input_folder / f"{description.input_name}{_OBSERVATIONS_SUFFIX}"
    actions_path = input_folder / f"{description.input_name}{_ACTIONS_SUFFIX}"
    if not observations_path.is_file():
        raise ScenarioConverterError(f"Observations file does not exist: {observations_path}")
    if not actions_path.is_file():
        raise ScenarioConverterError(f"Actions file does not exist: {actions_path}")

    observations = _load_json_file(observations_path, list)
    actions = _load_json_file(actions_path, list)
    if len(observations) != len(actions):
        raise ScenarioConverterError(f"{description.input_name} has {len(observations)} observations "
                                     f"and {len(actions)} actions")
    steps = []
    for index, (observation, action) in enumerate(zip(observations, actions)):
        if not isinstance(observation, dict) or not isinstance(action, dict):
            raise ScenarioConverterError(
                f"{description.input_name} step {index} must contain "
                "observation and action objects"
            )
        steps.append({"observations": observation, "actions": action})

    scenario: dict[str, Any] = {
        "name": description.name,
        "description": description.description,
        "test_data": steps,
    }
    destination = output_folder / f"{description.output_name}.json"

    # When overwriting, keep prior metadata if the new values are only defaults.
    if destination.is_file():
        existing = _load_json_file(destination, dict)
        if not description.description:
            old_description = existing.get("description")
            if isinstance(old_description, str):
                scenario["description"] = old_description
        if description.name == _name_from_input(description.input_name):
            old_name = existing.get("name")
            if isinstance(old_name, str) and old_name:
                scenario["name"] = old_name

    if dry_run:
        _LOGGER.info(
            "Would write %s from %s and %s (name=%r, description=%r)",
            destination,
            observations_path,
            actions_path,
            scenario["name"],
            scenario["description"],
        )
        return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    rendered = json.dumps(scenario, indent=2, ensure_ascii=False)
    destination.write_text(f"{rendered}\n", encoding="utf-8")
    _LOGGER.info("Wrote %s", destination)
    return destination


def sync_scenarios(
    input_folder: Path,
    output_folder: Path,
    ignore_list: str = "",
    dry_run: bool = False,
    descriptions_file: Path | None = None,
    descriptions_text: str | None = None,
) -> list[Path]:
    """Convert every logged pair in a folder into a scenario file.

    Description entries are included first. Remaining observations files are then
    added with an empty description. Ignored names are skipped.

    :param input_folder: Folder of logged observations and actions.
    :param output_folder: Folder that receives the scenario files.
    :param ignore_list: Comma-separated input names or display names to skip.
    :param dry_run: When true, explain what would be written and write nothing.
    :param descriptions_file: Optional descriptions document.
    :param descriptions_text: Optional piped descriptions document.
    :return: Scenario paths that were written, or that would be written.
    """
    if not input_folder.is_dir():
        raise ScenarioConverterError(f"Input folder does not exist: {input_folder}")

    descriptions = [
        ScenarioDescription(**entry)
        for entry in load_descriptions(descriptions_file, descriptions_text)
    ]

    # Discover any logged pairs that were not covered by a description entry.
    known_inputs = {item.input_name for item in descriptions}
    for path in sorted(input_folder.glob(f"*{_OBSERVATIONS_SUFFIX}")):
        if not path.is_file():
            continue
        stem = path.name[: -len(_OBSERVATIONS_SUFFIX)]
        if stem not in known_inputs:
            descriptions.append(ScenarioDescription(input_name=stem))

    seen_inputs: set[str] = set()
    seen_outputs: set[str] = set()
    for item in descriptions:
        if item.input_name in seen_inputs:
            raise ScenarioConverterError(f"Duplicate input_name {item.input_name!r}")
        if item.output_name in seen_outputs:
            raise ScenarioConverterError(f"Duplicate output_name {item.output_name!r}")
        seen_inputs.add(item.input_name)
        seen_outputs.add(item.output_name)

    ignore_names = [part.strip() for part in ignore_list.split(",") if part.strip()]
    active: list[ScenarioDescription] = []
    for item in descriptions:
        ignored = item.input_name in ignore_names or any(name.lower() == item.name.lower() for name in ignore_names)
        if ignored:
            _LOGGER.info("Skipping ignored input %s", item.input_name)
            continue
        active.append(item)

    if not active:
        _LOGGER.info("No scenarios to sync")
        return []

    return [
        convert_description(item, input_folder, output_folder, dry_run)
        for item in active
    ]


def _split_named_value(value: str, observations_file: bool) -> tuple[Path | None, str]:
    """Split a CLI path into an optional directory and a stem.

    :param value: A path, a filename, or a bare stem.
    :param observations_file: When true, value may be an observations filename.
    :return: The parent directory when value contains one, and the stem.
    """
    path = Path(value)
    filename = path.name
    if observations_file and filename.endswith(_OBSERVATIONS_SUFFIX):
        stem = filename[: -len(_OBSERVATIONS_SUFFIX)]
    elif filename.endswith(".json"):
        if observations_file:
            raise ScenarioConverterError(f"Input file must end with {_OBSERVATIONS_SUFFIX}")
        stem = filename[: -len(".json")]
    else:
        stem = filename
    folder = path.parent if len(path.parts) > 1 else None
    return folder, stem


def convert_scenario(
    input_value: str | None = None,
    output_value: str | None = None,
    name: str | None = None,
    description: str | None = None,
    descriptions_text: str | None = None,
    input_folder: Path | None = None,
    output_folder: Path | None = None,
) -> Path:
    """Convert one logged pair into a scenario file.

    :param input_value: Observations path or input stem.
    :param output_value: Scenario path or output stem.
    :param name: Display name stored in the scenario.
    :param description: Description stored in the scenario.
    :param descriptions_text: Optional piped description object or one-item array.
    :param input_folder: Folder used when input_value does not include a directory.
    :param output_folder: Folder used when output_value does not include a directory.
    :return: The scenario path that was written.
    """
    if input_folder is None:
        input_folder = DEFAULT_INPUT_FOLDER
    if output_folder is None:
        output_folder = DEFAULT_OUTPUT_FOLDER

    fields: dict[str, str] = {}
    if descriptions_text is not None and descriptions_text.strip():
        fields.update(parse_convert_description(descriptions_text))

    input_directory = None
    if input_value is not None:
        input_directory, fields["input_name"] = _split_named_value(input_value, observations_file=True)
    if name is not None:
        fields["name"] = name
    if description is not None:
        fields["description"] = description
    output_directory = None
    if output_value is not None:
        output_directory, fields["output_name"] = _split_named_value(output_value, observations_file=False)

    resolved = ScenarioDescription(**fields)
    source_folder = input_directory if input_directory is not None else input_folder
    destination_folder = (output_directory if output_directory is not None else output_folder)
    return convert_description(resolved, source_folder, destination_folder, dry_run=False)


def _stdin_has_piped_input(stdin: TextIO) -> bool:
    """
    :param stdin: Standard input for this process.
    :return: True when descriptions were piped or redirected and can be read now.
    """
    isatty = getattr(stdin, "isatty", None)
    if callable(isatty) and isatty():
        return False
    fileno = getattr(stdin, "fileno", None)
    if not callable(fileno):
        return callable(isatty) and not isatty()
    try:
        fd = fileno()
    except (OSError, ValueError):
        return False

    if os.name == "nt":
        # PeekNamedPipe distinguishes a redirected file from an empty pipe.
        try:
            handle = msvcrt.get_osfhandle(fd)
        except OSError:
            return False
        # windll stubs do not declare kernel32 exports such as GetFileType.
        kernel32: Any = ctypes.windll.kernel32
        file_type = kernel32.GetFileType(handle) & 0xFFF
        if file_type == 0x0001:  # FILE_TYPE_DISK
            return True
        if file_type != 0x0003:  # FILE_TYPE_PIPE
            return False
        available = ctypes.c_ulong()
        success = kernel32.PeekNamedPipe(handle, None, 0, None, ctypes.byref(available), None)
        return bool(success) and available.value > 0

    # On POSIX, accept a redirected regular file or a pipe that already has data.
    mode = os.fstat(fd).st_mode
    if stat.S_ISREG(mode):
        return True
    if not stat.S_ISFIFO(mode):
        return False
    readable, _, _ = select.select([fd], [], [], 0)
    return bool(readable)


def piped_text(stdin: TextIO) -> str | None:
    """
    :param stdin: Standard input for this process.
    :return: Piped or redirected text, or None when no such input is present.
    """
    if not _stdin_has_piped_input(stdin):
        return None
    text = stdin.read()
    return text if text.strip() else None


def build_parser() -> argparse.ArgumentParser:
    """
    :return: The argument parser for sync and convert.
    """
    parser = argparse.ArgumentParser(prog="data_converter",
                                     description="Convert logged observations and actions into scenario files.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    sync = subparsers.add_parser("sync", help="Convert every logged pair in a folder.")
    sync.add_argument("-i", "--input", type=Path, default=DEFAULT_INPUT_FOLDER,
                      help="Folder of logged observations and actions.")
    sync.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT_FOLDER,
                      help="Folder for scenario files.")
    sync.add_argument("-x", "--ignore-list", default="",
                      help="Comma-separated input names or display names to skip.")
    sync.add_argument("-d", "--dry-run", action="store_true",
                      help="Explain what would be written and write nothing.")
    sync.add_argument("-f", "--descriptions-file", type=Path, help="JSON file of scenario descriptions.")

    convert = subparsers.add_parser("convert", help="Convert one logged pair.")
    convert.add_argument("-i", "--input", help="Observations file or input stem.")
    convert.add_argument("-o", "--output", help="Scenario file or output stem.")
    convert.add_argument("-n", "--name", help="Display name stored in the scenario.")
    convert.add_argument("-d", "--description", help="Description stored in the scenario.")
    return parser


def main(argv: list[str] | None = None, stdin: TextIO | None = None) -> int:
    """Run sync or convert.

    :param argv: Arguments, excluding the program name. Defaults to sys.argv.
    :param stdin: Standard input. A pipe is read as a descriptions document.
    :return: Zero on success, or one when conversion fails.
    """
    if stdin is None:
        stdin = sys.stdin
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    args = build_parser().parse_args(argv)
    piped = piped_text(stdin)
    try:
        if args.command == "sync":
            sync_scenarios(
                input_folder=args.input,
                output_folder=args.output,
                ignore_list=args.ignore_list,
                dry_run=args.dry_run,
                descriptions_file=args.descriptions_file,
                descriptions_text=piped,
            )
        elif args.command == "convert":
            if args.input is None and args.name is None:
                raise ScenarioConverterError("convert requires --input or --name")
            convert_scenario(
                input_value=args.input,
                output_value=args.output,
                name=args.name,
                description=args.description,
                descriptions_text=piped,
            )
        else:
            raise ScenarioConverterError(f"Unknown command {args.command!r}")
    except ScenarioConverterError as exc:
        _LOGGER.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
