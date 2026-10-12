from __future__ import annotations

import json
import logging
from dataclasses import dataclass, replace
from datetime import datetime
from enum import IntEnum
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from policy.input_output import CarActions, CarObservations

_LOGGER = logging.getLogger(__name__)

DEFAULT_LOG_FOLDER = "./logs"
"""Default directory for log files when a folder is required."""

DEFAULT_LOG_LEVEL_NAME = "info"
"""Default Python logging level name used by the ROS parameter."""

OBSERVATIONS_LOG_FILENAME_FORMAT = "%Y-%m-%d_%H-%M_observations.json"
"""strftime pattern for the observations data-log filename."""

ACTIONS_LOG_FILENAME_FORMAT = "%Y-%m-%d_%H-%M_actions.json"
"""strftime pattern for the actions data-log filename."""

GENERAL_LOG_FILENAME_FORMAT = "%Y-%m-%d_%H-%M_logging.log"
"""strftime pattern for the general Python log filename."""


class LogLevel(IntEnum):
    """Python logging levels accepted by LoggingParameters."""
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL


DEFAULT_LOG_LEVEL = LogLevel.INFO
"""Default Python logging level after converting from a name."""

_LOG_LEVELS = {
    "critical": LogLevel.CRITICAL,
    "error": LogLevel.ERROR,
    "warning": LogLevel.WARNING,
    "info": LogLevel.INFO,
    "debug": LogLevel.DEBUG,
}


@dataclass(frozen=True)
class LoggingParameters:
    """Startup settings for policy data logging and standard Python logging.

    Data logging (CarObservations / CarActions) is separate from standard Python logging.
    logging_folder is required when is_data_logging or log_to_file is true.
    """
    is_data_logging: bool
    """When true, record CarObservations and CarActions each policy step."""
    logging_folder: str
    """Directory for data-log and/or general Python log files when either is enabled."""
    log_level: LogLevel
    """Python logging level."""
    log_to_file: bool
    """When true, send general Python logs to a timestamped file under logging_folder."""
    observations_log_filename: str | None
    """Observations JSON filename for the current active run, otherwise None."""
    actions_log_filename: str | None
    """Actions JSON filename for the current active run, otherwise None."""


def _is_usable_log_folder(logging_folder: str) -> bool:
    """
    :param logging_folder: Candidate directory path for log files.
    :return: True when the path can be used as a log directory.
    """
    if not isinstance(logging_folder, str) or not logging_folder.strip():
        return False
    path = Path(logging_folder)
    return not path.exists() or path.is_dir()


def _resolve_log_level(log_level: str) -> LogLevel:
    """
    :param log_level: Requested logging level name, such as info or debug.
    :return: Matching logging module level constant, or DEFAULT_LOG_LEVEL if unrecognised.
    """
    if isinstance(log_level, str) and log_level.strip().lower() in _LOG_LEVELS:
        return _LOG_LEVELS[log_level.strip().lower()]
    _LOGGER.warning("Invalid log level %r; using default log level %r", log_level, DEFAULT_LOG_LEVEL_NAME)
    return DEFAULT_LOG_LEVEL


def validate_parameters(
    is_data_logging: bool,
    logging_folder: str,
    log_level: str,
    log_to_file: bool,
) -> LoggingParameters:
    """Validate logging settings and return a LoggingParameters instance.

    The log folder is required when is_data_logging or log_to_file is true; an empty, non-string,
    or non-directory path then falls back to DEFAULT_LOG_FOLDER. Otherwise the supplied folder is
    kept without checking it. log_level names are converted to logging module constants
    (logging.INFO, and so on); an unrecognised name falls back to DEFAULT_LOG_LEVEL.

    :param is_data_logging: Whether policy step input/output should be recorded.
    :param logging_folder: Requested directory for log files.
    :param log_level: Requested Python logging level name.
    :param log_to_file: Whether general Python logs should also go to a file.
    :return: Validated logging parameters.
    """
    folder = logging_folder
    if (is_data_logging or log_to_file) and not _is_usable_log_folder(logging_folder):
        _LOGGER.warning("Invalid logging folder %r; using default log folder %r", logging_folder, DEFAULT_LOG_FOLDER)
        folder = DEFAULT_LOG_FOLDER
    return LoggingParameters(
        is_data_logging=bool(is_data_logging),
        logging_folder=folder,
        log_level=_resolve_log_level(log_level),
        log_to_file=bool(log_to_file),
        observations_log_filename=None,
        actions_log_filename=None,
    )


def _get_observations_log_filename() -> str:
    """
    :return: Observations data-log filename for the current local date and time.
    """
    return datetime.now().strftime(OBSERVATIONS_LOG_FILENAME_FORMAT)


def _get_actions_log_filename() -> str:
    """
    :return: Actions data-log filename for the current local date and time.
    """
    return datetime.now().strftime(ACTIONS_LOG_FILENAME_FORMAT)


def _get_general_log_filename() -> str:
    """
    :return: General Python log filename for the current local date and time.
    """
    return datetime.now().strftime(GENERAL_LOG_FILENAME_FORMAT)


def _init_json_array_file(path: Path):
    """Replace path with an empty JSON array.

    The file ends with a line containing only ], so later appends can truncate that line.

    :param path: File that should hold a JSON array of logged records.
    """
    path.write_text("[\n]\n", encoding="utf-8")


def _format_log_entry(entry: Any) -> str:
    """Render one log record with a tab at the start of every line.

    :param entry: Already-serialisable record to render.
    :return: Tab-indented JSON for the record, with no trailing newline.
    """
    rendered = json.dumps(entry, ensure_ascii=False, indent="\t")
    return "\t" + rendered.replace("\n", "\n\t")


def _append_json_array(path: Path, entry: Any):
    """Append one serialised record without rewriting the whole array.

    Assumes a valid file that ends with ]. Truncates back to the previous
    non-whitespace character so the comma sits on that record, writes the
    tab-indented entry, then rewrites ].

    :param path: JSON array file to update.
    :param entry: Already-serialisable record to append.
    """
    entry_json = _format_log_entry(entry)
    with path.open("r+", encoding="utf-8", newline="\n") as file:
        file.seek(0, 2)
        end = file.tell()
        if end == 0:
            file.write(f"[\n{entry_json}\n]\n")
            return

        pos = end
        closing_bracket = None
        while pos > 0:
            pos -= 1
            file.seek(pos)
            char = file.read(1)
            if char in "\r\n \t":
                continue
            closing_bracket = char
            break
        if closing_bracket != "]":
            raise ValueError(f"JSON array file {path} does not end with ]")

        probe = pos
        previous = None
        while probe > 0:
            probe -= 1
            file.seek(probe)
            char = file.read(1)
            if char in "\r\n \t":
                continue
            previous = char
            break
        if previous is None:
            raise ValueError(f"JSON array file {path} does not end with ]")

        file.seek(probe + 1)
        file.truncate()
        if previous == "[":
            file.write(f"\n{entry_json}\n]\n")
        else:
            file.write(f",\n{entry_json}\n]\n")


def _write_log_record(path: Path, entry: Any, record_name: str):
    """Append one record. A write failure is logged and re-raised so the policy stops.

    :param path: JSON array file to update.
    :param entry: Already-serialisable record to append.
    :param record_name: Short name used in the failure log, such as observations.
    """
    try:
        _append_json_array(path, entry)
    except Exception:
        _LOGGER.critical("Failed to write the %s data log at %s; stopping the policy", record_name, path)
        raise


def setup_logging(logging_params: LoggingParameters):
    """Configure Python logging.

    Always applies log_level via logging.basicConfig. When log_to_file is true, also writes to
    logging_folder / get_general_log_filename().

    :param logging_params: The parameters for logging.
    """
    if logging_params.log_to_file or logging_params.is_data_logging:
        Path(logging_params.logging_folder).mkdir(parents=True, exist_ok=True)

    config: dict = {"level": logging_params.log_level}
    if logging_params.log_to_file:
        config["filename"] = str(
            Path(logging_params.logging_folder) / _get_general_log_filename()
        )
    logging.basicConfig(**config)


def start_data_log(logging_params: LoggingParameters) -> LoggingParameters:
    """Open a new observations and actions log named for the current local time.

    An existing file with that name is replaced. When data logging is disabled, the parameters
    are returned unchanged.

    :param logging_params: Validated logging parameters.
    :return: Parameters whose data-log filenames match the current local time.
    """
    if not logging_params.is_data_logging:
        return logging_params
    folder = Path(logging_params.logging_folder)
    folder.mkdir(parents=True, exist_ok=True)
    observations_log_filename = _get_observations_log_filename()
    actions_log_filename = _get_actions_log_filename()
    _init_json_array_file(folder / observations_log_filename)
    _init_json_array_file(folder / actions_log_filename)
    return replace(
        logging_params,
        observations_log_filename=observations_log_filename,
        actions_log_filename=actions_log_filename,
    )


def log_input(observations: CarObservations, logging_params: LoggingParameters):
    """Record the policy observations for one step.

    :param observations: The observations passed to the policy for this step.
    :param logging_params: The parameters for logging.
    """
    if not logging_params.is_data_logging or not logging_params.observations_log_filename:
        return
    path = Path(logging_params.logging_folder) / logging_params.observations_log_filename
    _write_log_record(path, observations.serialise(), "observations")


def log_output(actions: CarActions, logging_params: LoggingParameters):
    """Record the policy actions for one step.

    :param actions: The actions chosen by the policy for this step.
    :param logging_params: The parameters for logging.
    """
    if not logging_params.is_data_logging or not logging_params.actions_log_filename:
        return
    path = Path(logging_params.logging_folder) / logging_params.actions_log_filename
    _write_log_record(path, actions.serialise(), "actions")
