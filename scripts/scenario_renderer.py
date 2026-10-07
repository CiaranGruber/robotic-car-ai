"""Command-line helpers for recorded policy scenarios.

Example::

    python -m scripts.scenario_renderer list
    python -m scripts.scenario_renderer show -n straight_even
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


def list_scenarios():
    """Print each available scenario stem and its display name.

    Stems match the values accepted by show -n. When no scenario files are
    present, a short message is printed instead.
    """
    names = available_scenario_names()
    if not names:
        print("No scenarios found.")
        return
    for name in names:
        scenario = Scenario.from_name(ScenarioType.REAL, name)
        print(f"{name}\t{scenario.name}")


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


class ScenariosCliError(Exception):
    """The scenarios command could not complete."""


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser for list and show.

    :return: The argument parser for scenario commands.
    """
    parser = argparse.ArgumentParser(prog="scenarios", description="Work with recorded policy scenarios.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List available scenario names.")

    show = subparsers.add_parser("show", help="Open the interactive scenario renderer.")
    source = show.add_mutually_exclusive_group(required=True)
    source.add_argument("-n", "--name", help="Scenario name under real_scenarios, such as straight_even.")
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
            show_scenario(name=args.name, path=args.path)
        else:
            raise ScenariosCliError(f"Unknown command {args.command!r}")
    except ScenariosCliError as exc:
        _LOGGER.error("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
