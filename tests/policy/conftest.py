"""pytest options for the policy tests.

The Stage 1 unseen cases are kept back for the final MPC vs RL comparison (milestone M1), so tests marked unseen
only run with --run-unseen.
"""
import pytest


def pytest_addoption(parser):
    parser.addoption("--run-unseen", action="store_true", help="also run the Stage 1 unseen cases (M1 only)")


def pytest_configure(config):
    config.addinivalue_line("markers", "unseen: Stage 1 unseen case, only run with --run-unseen")


def pytest_collection_modifyitems(config, items):
    if config.getoption("--run-unseen"):
        return
    skip = pytest.mark.skip(reason="unseen case: run with --run-unseen for the final comparison only")
    for item in items:
        if item.get_closest_marker("unseen"):
            item.add_marker(skip)
