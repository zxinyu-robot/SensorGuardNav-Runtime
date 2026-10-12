import math

import numpy as np
import pytest


@pytest.fixture
def healthy_cloud() -> np.ndarray:
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    ranges = 5.0 + 2.0 * np.sin(3.0 * angles) + 0.5 * np.cos(7.0 * angles)
    return np.column_stack(
        (ranges * np.cos(angles), ranges * np.sin(angles), np.zeros(360), np.ones(360))
    )
