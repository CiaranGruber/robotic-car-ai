from __future__ import annotations

import json
import logging
from dataclasses import dataclass, replace
from datetime import datetime
from enum import IntEnum, Enum
from pathlib import Path
from typing import TYPE_CHECKING, Any

from policy.memory import CarMemory

if TYPE_CHECKING:
    from policy.data_logging.run_timing import StepTiming
    from policy.input_output import CarActions, CarObservations

_LOGGER = logging.getLogger(__name__)

DEFAULT_LOG_FOLDER = "./logs"
"""Default directory for log files when a folder is required."""

DEFAULT_LOG_LEVEL_NAME = "info"
"""Default Python logging level name used by the ROS parameter."""

DATA_LOG_STEM_FORMAT = "%Y-%m-%d_%H-%M-%S"
"""strftime pattern for the stem shared by an observations and actions data-log pair."""

OBSERVATIONS_LOG_SUFFIX = "_observations.json"
"""Filename suffix of the observations data log, after the stem."""

ACTIONS_LOG_SUFFIX = "_actions.json"
"""Filename suffix of the actions data log, after the stem."""

MEMORY_LOG_SUFFIX = "_memory.json"
"""Filename suffix of the memory data log, after the stem."""

CONFIG_LOG_SUFFIX = "_config.json"
"""Filename suffix of the policy settings saved with a data-log pair, after the stem."""

TIMING_LOG_SUFFIX = "_timing.json"
"""Filename suffix of the policy step timing data log (StepTiming in run_timing.py), after the stem."""

GENERAL_LOG_FILENAME_FORMAT = "%Y-%m-%d_%H-%M-%S_logging.log"
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

    Data logging (CarObservations, CarMemory, CarActions and StepTiming) is separate from standard
    Python logging.
    logging_folder is required when is_data_logging or log_to_file is true.
    """
    is_data_logging: bool
    """When true, record CarObservations, CarMemory, CarActions and StepTiming each policy step."""
    log_folder: Path
    """Directory for data-log and/or general Python log files when either is enabled."""
    log_level: LogLevel
    """Python logging level."""
    log_to_file: bool
    """When true, send general Python logs to a timestamped file under logging_folder."""
    stem: str | None
    """The data log stem if generated"""

    @property
    def started(self) -> bool:
        return self.stem is not None

    @property
    def observations_log_path(self) -> Path:
        """Observations JSON filename for the current active run, otherwise None."""
        if not self.started:
            raise ValueError("Attempted to get the observations log path but logging has not started")
        return self.log_folder / f"{self.stem}{OBSERVATIONS_LOG_SUFFIX}"

    @property
    def memory_log_path(self) -> Path:
        """Memory JSON filename for the current active run, otherwise None."""
        if not self.started:
            raise ValueError("Attempted to get the memory log path but logging has not started")
        return self.log_folder / f"{self.stem}{MEMORY_LOG_SUFFIX}"

    @property
    def actions_log_path(self) -> Path:
        """Actions JSON filename for the current active run, otherwise None."""
        if not self.started:
            raise ValueError("Attempted to get the actions log path but logging has not started")
        return self.log_folder / f"{self.stem}{ACTIONS_LOG_SUFFIX}"

    @property
    def settings_log_path(self) -> Path:
        """Settings JSON filename for the current active run, otherwise None."""
        if not self.started:
            raise ValueError("Attempted to get the settings log path but logging has not started")
        return self.log_folder / f"{self.stem}{CONFIG_LOG_SUFFIX}"

    @property
    def timing_log_path(self) -> Path:
        """Step timing JSON filename for the current active run, otherwise None."""
        if not self.started:
            raise ValueError("Attempted to get the timing log path but logging has not started")
        return self.log_folder / f"{self.stem}{TIMING_LOG_SUFFIX}"


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
        log_folder=Path(folder),
        log_level=_resolve_log_level(log_level),
        log_to_file=bool(log_to_file),
        stem=None,
    )


def _new_data_log_stem(folder: Path) -> str:
    """
    :param folder: Folder the data-log pair will be written to.
    :return: Stem for a new observations and actions pair: the current local date and time to the second, with _2,
        _3, ... added when a pair started in the same second already exists, so no earlier run is replaced.
    """
    stem = datetime.now().strftime(DATA_LOG_STEM_FORMAT)
    candidate, number = stem, 1
    while any((folder / f"{candidate}{suffix}").exists()
              for suffix in (OBSERVATIONS_LOG_SUFFIX, ACTIONS_LOG_SUFFIX, CONFIG_LOG_SUFFIX)):
        number += 1
        candidate = f"{stem}_{number}"
    return candidate


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
        Path(logging_params.log_folder).mkdir(parents=True, exist_ok=True)

    config: dict = {"level": logging_params.log_level}
    if logging_params.log_to_file:
        config["filename"] = str(
            Path(logging_params.log_folder) / _get_general_log_filename()
        )
    logging.basicConfig(**config)


def start_data_log(logging_params: LoggingParameters,
                   settings: dict[str, Any] | None = None) -> LoggingParameters:
    """Open a new observations, memory, actions and step timing log named for the current local time.

    The four files share one stem, the time to the second, and an existing set is never replaced
    (see _new_data_log_stem). When data logging is disabled, the parameters are returned
    unchanged and nothing is written.

    :param logging_params: Validated logging parameters.
    :param settings: The policy settings this run uses, by parameter name, or None. They are
        saved at settings_log_path, so each recording keeps the settings it was made with even
        after the YAML changes. Values must be JSON-serialisable.
    :return: Parameters whose data-log filenames match the current local time.
    """
    if not logging_params.is_data_logging:
        return logging_params
    logging_params.log_folder.mkdir(parents=True, exist_ok=True)
    stem = _new_data_log_stem(logging_params.log_folder)
    params = replace(logging_params, stem=stem)
    if settings is not None:
        params.settings_log_path.write_text(json.dumps(settings, ensure_ascii=False, indent="\t", sort_keys=True) + "\n", encoding="utf-8")
    # Init arrays
    _init_json_array_file(params.observations_log_path)
    _init_json_array_file(params.memory_log_path)
    _init_json_array_file(params.actions_log_path)
    _init_json_array_file(params.timing_log_path)
    return params


def log_input(observations: CarObservations, logging_params: LoggingParameters):
    """Record the policy observations for one step.

    :param observations: The observations passed to the policy for this step.
    :param logging_params: The parameters for logging.
    """
    if not logging_params.started:
        return
    _write_log_record(logging_params.observations_log_path, observations.serialise(), "observations")


def log_memory(memory: CarMemory, logging_params: LoggingParameters):
    """Record the car memory for one step.

    :param memory: The car memory at this step. The current set, from memory.data, is written.
    :param logging_params: The parameters for logging.
    """
    if not logging_params.started:
        return
    _write_log_record(logging_params.memory_log_path, memory.data.serialise(), "memory")


def log_output(actions: CarActions, logging_params: LoggingParameters):
    """Record the policy actions for one step.

    :param actions: The actions chosen by the policy for this step.
    :param logging_params: The parameters for logging.
    """
    if not logging_params.started:
        return
    _write_log_record(logging_params.actions_log_path, actions.serialise(), "actions")


def log_timing(step: StepTiming, logging_params: LoggingParameters):
    """Record how long one policy step's calculation took.

    :param step: The step's timing, from RunTiming.record.
    :param logging_params: The parameters for logging.
    """
    if not logging_params.started:
        return
    _write_log_record(logging_params.timing_log_path, step.serialise(), "timing")
