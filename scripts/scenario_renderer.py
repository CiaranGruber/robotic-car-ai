"""Command-line helpers for recorded policy scenarios.

Example::

    python -m scripts.scenario_renderer list
    python -m scripts.scenario_renderer show -n straight_even
    python -m scripts.scenario_renderer show -t synthetic -n synthetic_s0
    python -m scripts.scenario_renderer show --path scenarios/data/real_scenarios/straight_even_scenario.json
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from scenarios import SCENARIO_SUFFIX, SCENARIOS_DIR
from scenarios.renderer import ScenarioRenderer
from scenarios.scenario_wrapper import Scenario, ScenarioType

_LOGGER = logging.getLogger(__name__)

_SCENARIO_TYPE_CHOICES = [member.name.lower() for member in ScenarioType]


def available_scenario_names(scenario_type: ScenarioType = ScenarioType.REAL) -> list[str]:
    """List scenario stems found under a scenario type folder.

    :param scenario_type: Scenario folder under scenarios/data/.
    :return: Sorted scenario stems usable with show -n.
    """
    folder = SCENARIOS_DIR / scenario_type.value
    if not folder.is_dir():
        return []
    names: list[str] = []
    for path in sorted(folder.glob(f"*{SCENARIO_SUFFIX}")):
        names.append(path.name.removesuffix(SCENARIO_SUFFIX))
    return names


def _markdown_cell(value: str) -> str:
    """
    :param value: Cell text that may contain markdown table separators.
    :return: Text safe for a single markdown table cell.
    """
    return value.replace("|", "\\|").replace("\n", " ").strip()


def _markdown_table(headers: list[str], rows: list[list[str]]) -> str:
    """
    :param headers: Column titles.
    :param rows: Table body rows; each row must match the header length.
    :return: A GitHub-flavoured markdown table. All columns except the last
        are padded to a shared width; the last column is left unpadded.
    """
    cells = [[_markdown_cell(header) for header in headers]]
    cells.extend([[_markdown_cell(value) for value in row] for row in rows])
    last = len(headers) - 1
    widths = [
        max(len(row[index]) for row in cells) if index < last else 0
        for index in range(len(headers))
    ]

    def format_cell(value: str, index: int) -> str:
        if index < last:
            return value.ljust(widths[index])
        return value

    def format_row(row: list[str]) -> str:
        return "| " + " | ".join(
            format_cell(row[index], index) for index in range(len(headers))
        )

    separator = "| " + " | ".join(
        "-" * (widths[index] if index < last else max(len(headers[index]), 3))
        for index in range(len(headers))
    )
    return "\n".join([format_row(cells[0]), separator, *(format_row(row) for row in cells[1:])])


def list_scenarios():
    """Print available scenarios as a markdown table.

    Columns are type (for show -t), stem (for show -n), display name, and
    description. When no scenario files are present, a short message is
    printed instead.
    """
    rows: list[list[str]] = []
    for scenario_type in ScenarioType:
        type_name = scenario_type.name.lower()
        for name in available_scenario_names(scenario_type):
            scenario = Scenario.from_name(scenario_type, name)
            rows.append([type_name, name, scenario.name, scenario.description or ""])
    if not rows:
        print("No scenarios found.")
        return
    print(_markdown_table(["Type", "Stem", "Name", "Description"], rows))


def resolve_scenario(name: str | None, path: Path | None, scenario_type: ScenarioType = ScenarioType.REAL) -> Scenario:
    """Resolve a scenario from a name or path.

    :param name: Scenario stem without _scenario.json, or None.
    :param path: Path to a scenario JSON file, or None.
    :param scenario_type: Scenario folder used when resolving by name.
    :return: The scenario to render.
    """
    if name is not None:
        scenario = Scenario.from_name(scenario_type, name)
        if not scenario.scenario_path.is_file():
            raise ScenariosCliError(
                f"Scenario file does not exist: {scenario.scenario_path}")
        return scenario
    if path is None:
        raise ScenariosCliError("show requires --name or --path")
    if not path.is_file():
        raise ScenariosCliError(f"Scenario file does not exist: {path}")
    return Scenario(path)


def show_scenario(name: str | None = None, path: Path | None = None, scenario_type: ScenarioType = ScenarioType.REAL) -> None:
    """Open the interactive renderer for one scenario.

    :param name: Scenario stem without _scenario.json, or None.
    :param path: Path to a scenario JSON file, or None.
    :param scenario_type: Scenario folder used when resolving by name.
    """
    scenario = resolve_scenario(name, path, scenario_type)
    ScenarioRenderer(scenario).show()


class ScenariosCliError(Exception):
    """The scenarios command could not complete."""


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser for list and show.

    :return: The argument parser for scenario commands.
    """
    parser = argparse.ArgumentParser(prog="scenarios", description="Work with recorded policy scenarios.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser(
        "list", help="List available scenarios as a markdown table.")

    show = subparsers.add_parser("show", help="Open the interactive scenario renderer.")
    show.add_argument("-t", "--type", choices=_SCENARIO_TYPE_CHOICES, default=ScenarioType.REAL.name.lower(),
                      help="Scenario type for -n. Default: real.")
    source = show.add_mutually_exclusive_group(required=True)
    source.add_argument("-n", "--name", help="Scenario stem under the selected type folder, such as straight_even.")
    source.add_argument("-p", "--path", type=Path, help="Path to a scenario JSON file.")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run a scenarios command.

    :param argv: Arguments, excluding the program name. Defaults to sys.argv.
    :return: Zero on success, or one when the command fails.
    """
    if not logging.getLogger().handlers:
        logging.basicConfig(
            level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "list":
            list_scenarios()
        elif args.command == "show":
            show_scenario(name=args.name, path=args.path, scenario_type=ScenarioType[args.type.upper()])
        else:
            raise ScenariosCliError(f"Unknown command {args.command!r}")
    except ScenariosCliError as exc:
        _LOGGER.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
