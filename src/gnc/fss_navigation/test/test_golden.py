"""Check native EKF results against frozen outputs from the verified pre-port Python oracle."""

import subprocess
import sys
from pathlib import Path

import numpy as np


def check_golden(runner):
    with np.load(Path(__file__).with_name("ekf_golden.npz"), allow_pickle=False) as fixture:
        commands = str(fixture["commands"])
        expected = fixture["expected"]
    process = subprocess.run([str(runner)], input=commands, text=True,
                             capture_output=True, check=True, timeout=30)
    actual = np.array([np.fromstring(line, sep=" ") for line in process.stdout.splitlines()])
    assert expected.shape == (741, 135), expected.shape
    assert actual.shape == expected.shape, (actual.shape, expected.shape, process.stderr)
    np.testing.assert_array_equal(actual[:, :2], expected[:, :2])
    np.testing.assert_allclose(actual[:, 2:], expected[:, 2:], rtol=2e-9, atol=2e-11)
    print("Matched 741 frozen operations: 11 states, 121 covariance entries, NIS and gating")


if __name__ == "__main__":
    check_golden(sys.argv[1])
