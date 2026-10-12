"""Exercise descriptor redirection in a subprocess, without altering pytest's stderr."""
import subprocess
import sys
from pathlib import Path


def test_file_logging_captures_python_and_native_stderr_and_restores_console(tmp_path):
    script = '''
import logging, os, sys, traceback
from pathlib import Path
from policy.data_logging.data_logging import setup_logging, validate_parameters
folder = sys.argv[1]
logging.basicConfig(level=logging.INFO)
params = validate_parameters(False, folder, "info", True)
setup_logging(params)
logging.error("python-log-marker")
print("python-stderr-marker", file=sys.stderr, flush=True)
os.write(2, b"native-stderr-marker\\n")
try:
    raise RuntimeError("traceback-marker")
except RuntimeError:
    traceback.print_exc()
setup_logging(params)
os.write(2, b"second-setup-marker\\n")
setup_logging(validate_parameters(False, folder, "info", False))
os.write(2, b"restored-console-marker\\n")
'''
    result = subprocess.run([sys.executable, "-c", script, str(tmp_path)],
                            cwd=Path(__file__).resolve().parents[2], capture_output=True, text=True, check=True)
    content = "".join(path.read_text() for path in tmp_path.glob("*_logging.log"))
    for marker in ("python-log-marker", "python-stderr-marker", "native-stderr-marker",
                   "traceback-marker", "second-setup-marker"):
        assert marker in content
        assert marker not in result.stderr
    assert "Traceback (most recent call last)" in content
    assert "restored-console-marker" in result.stderr
    assert "restored-console-marker" not in content
