"""Check that policy sources compile and import with CI dependencies.

ROS and generated packages (rclpy, message packages, tf2_ros, etc.) are not on
PyPI. When they are absent, this test installs temporary import stubs so
``scripts/policy_node.py`` can still be imported. Real packages are left alone
when already importable (for example on a ROS workstation).

Run from the repository root with:
``python -m pytest tests/policy/test_src_compiles.py``
"""
from __future__ import annotations

import importlib
import importlib.abc
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import types

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
POLICY_DIR = REPO_ROOT / "policy"
POLICY_NODE = REPO_ROOT / "scripts" / "policy_node.py"

# Top-level names that are provided by ROS / generated interface packages.
_GENERATED_PACKAGE_ROOTS = (
    "ament_index_python",
    "dream_interfaces",
    "geometry_msgs",
    "rcl_interfaces",
    "rclpy",
    "sensor_msgs",
    "std_msgs",
    "tf2_ros",
)


class _GeneratedPackageLoader(importlib.abc.Loader):
    """Loader that creates empty package/module stubs for generated deps."""

    def create_module(
        self,
        spec: importlib.machinery.ModuleSpec,
    ) -> types.ModuleType | None:
        return None

    def exec_module(self, module: types.ModuleType) -> None:
        # Mark as a package so ``from pkg.sub import Name`` can resolve.
        module.__path__ = []  # type: ignore[attr-defined]

        def __getattr__(name: str) -> object:
            full_name = f"{module.__name__}.{name}"
            if name[:1].islower():
                return importlib.import_module(full_name)
            # Uppercase names are treated as classes (e.g. Node, Float32) so
            # ``class PolicyNode(Node)`` and message construction stay valid.
            value = type(name, (), {"__module__": module.__name__})
            setattr(module, name, value)
            return value

        module.__getattr__ = __getattr__  # type: ignore[attr-defined]


class _GeneratedPackageFinder(importlib.abc.MetaPathFinder):
    """Find generated packages only after the normal importers have failed."""

    def find_spec(
        self,
        fullname: str,
        path: list[str] | None,
        target: types.ModuleType | None = None,
    ) -> importlib.machinery.ModuleSpec | None:
        if not any(
            fullname == root or fullname.startswith(f"{root}.")
            for root in _GENERATED_PACKAGE_ROOTS
        ):
            return None
        return importlib.util.spec_from_loader(fullname, _GeneratedPackageLoader())


def _python_sources() -> list[Path]:
    """Return every policy package module plus the policy node script.

    :returns: Source paths relative to the repository layout.
    """
    sources = sorted(POLICY_DIR.rglob("*.py"))
    sources.append(POLICY_NODE)
    return sources


def _bytecode_compile(path: Path) -> None:
    """Compile one source file to bytecode without importing it.

    :param path: Absolute path to a ``.py`` file.
    """
    source = path.read_text(encoding="utf-8")
    compile(source, str(path), "exec", dont_inherit=True)


@pytest.fixture
def generated_package_stubs():
    """Install import stubs for missing ROS / generated packages, then restore."""
    finder = _GeneratedPackageFinder()
    prior_modules = {
        name: sys.modules.get(name)
        for name in list(sys.modules)
        if any(
            name == root or name.startswith(f"{root}.")
            for root in _GENERATED_PACKAGE_ROOTS
        )
    }
    sys.meta_path.append(finder)
    try:
        yield
    finally:
        if finder in sys.meta_path:
            sys.meta_path.remove(finder)
        # Drop stubs installed during the test; restore any pre-existing modules.
        for name in list(sys.modules):
            if any(
                name == root or name.startswith(f"{root}.")
                for root in _GENERATED_PACKAGE_ROOTS
            ):
                if name in prior_modules:
                    previous = prior_modules[name]
                    if previous is None:
                        del sys.modules[name]
                    else:
                        sys.modules[name] = previous
                else:
                    del sys.modules[name]


def test_policy_sources_bytecode_compile() -> None:
    """Every policy source file must be syntactically valid Python."""
    assert POLICY_DIR.is_dir(), f"missing policy package at {POLICY_DIR}"
    assert POLICY_NODE.is_file(), f"missing policy node at {POLICY_NODE}"
    for path in _python_sources():
        _bytecode_compile(path)


def test_policy_sources_import_with_installed_dependencies(
    generated_package_stubs,
) -> None:
    """Import the policy package and node using requirements.txt dependencies.

    Generated ROS packages are stubbed when absent. Failures here usually mean a
    missing PyPI dependency, a broken import, or invalid module-level code.
    """
    # Fresh imports so this check does not rely on other tests having run first.
    for name in list(sys.modules):
        if name == "policy" or name.startswith("policy."):
            del sys.modules[name]
        if name == "ai4r_policy_compile_check":
            del sys.modules[name]

    import policy  # noqa: F401
    import policy.policy_runner  # noqa: F401
    import policy.input_output  # noqa: F401

    spec = importlib.util.spec_from_file_location(
        "ai4r_policy_compile_check",
        POLICY_NODE,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
