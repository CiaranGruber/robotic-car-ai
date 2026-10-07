"""Command-line helpers for recorded policy scenarios.

Example::

    python -m scripts.scenario_renderer show -n straight_even
    python -m scripts.scenario_renderer show --path scenarios/data/real_scenarios/straight_even_scenario.json
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from scenarios.renderer import ScenarioRenderer
from scenarios.scenario_wrapper import Scenario, ScenarioType

_LOGGER = logging.getLogger(__name__)


class ScenariosCliError(Exception):
    """The scenarios command could not complete."""


def build_parser() -> argparse.ArgumentParser:
    """
    :return: The argument parser for scenario commands.
    """
    parser = argparse.ArgumentParser(prog="scenarios", description="Work with recorded policy scenarios.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    show = subparsers.add_parser("show", help="Open the interactive scenario renderer.")
    source = show.add_mutually_exclusive_group(required=True)
    source.add_argument("-n", "--name", help="Scenario name under real_scenarios, such as straight_even.")
    source.add_argument("-p", "--path", type=Path, help="Path to a scenario JSON file.")
    return parser


def resolve_scenario(name: str | None, path: Path | None) -> Scenario:
    """Resolve a scenario from a name or path.

    :param name: Scenario stem without _scenario.json, or None.
    :param path: Path to a scenario JSON file, or None.
    :return: The scenario to render.
    """
    if name is not None:
        scenario = Scenario.from_name(ScenarioType.REAL, name)
        if not scenario.scenario_path.is_file():
            raise ScenariosCliError(
                f"Scenario file does not exist: {scenario.scenario_path}")
        return scenario
    if path is None:
        raise ScenariosCliError("show requires --name or --path")
    if not path.is_file():
        raise ScenariosCliError(f"Scenario file does not exist: {path}")
    return Scenario(path)


def show_scenario(name: str | None = None, path: Path | None = None):
    """Open the interactive renderer for one scenario.

    :param name: Scenario stem without _scenario.json, or None.
    :param path: Path to a scenario JSON file, or None.
    """
    scenario = resolve_scenario(name, path)
    ScenarioRenderer(scenario).show()


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
        if args.command == "show":
            show_scenario(name=args.name, path=args.path)
        else:
            raise ScenariosCliError(f"Unknown command {args.command!r}")
    except ScenariosCliError as exc:
        _LOGGER.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
