import numpy as np
import pytest

from sensor_guard_nav.faults import (
    FrameDelay,
    FrameFreeze,
    gaussian_noise,
    intensity_attenuation,
    nan_corruption,
    point_dropout,
    range_bias,
    sector_occlusion,
)


def test_dropout_is_deterministic(healthy_cloud: np.ndarray) -> None:
    first = point_dropout(healthy_cloud, 0.4, seed=7)
    second = point_dropout(healthy_cloud, 0.4, seed=7)
    assert np.array_equal(first, second)
    assert len(first) == 216


def test_sector_occlusion_removes_requested_region(healthy_cloud: np.ndarray) -> None:
    result = sector_occlusion(healthy_cloud, -0.5, 0.5)
    angles = np.arctan2(result[:, 1], result[:, 0])
    assert not np.any((angles >= -0.5) & (angles <= 0.5))


def test_noise_is_deterministic_and_non_mutating(healthy_cloud: np.ndarray) -> None:
    original = healthy_cloud.copy()
    first = gaussian_noise(healthy_cloud, 0.1, seed=9)
    second = gaussian_noise(healthy_cloud, 0.1, seed=9)
    assert np.array_equal(first, second)
    assert np.array_equal(healthy_cloud, original)
    assert not np.array_equal(first, original)


def test_range_bias_changes_radius(healthy_cloud: np.ndarray) -> None:
    before = np.linalg.norm(healthy_cloud[:, :3], axis=1)
    after = np.linalg.norm(range_bias(healthy_cloud, 0.4)[:, :3], axis=1)
    assert np.allclose(after, before + 0.4)


def test_intensity_and_nan_faults(healthy_cloud: np.ndarray) -> None:
    attenuated = intensity_attenuation(healthy_cloud, 0.2)
    assert np.allclose(attenuated[:, 3], 0.2)
    first = nan_corruption(healthy_cloud, 0.1, seed=3)
    second = nan_corruption(healthy_cloud, 0.1, seed=3)
    assert np.array_equal(first, second, equal_nan=True)
    assert np.isnan(first).all(axis=1).sum() == 36


def test_frame_freeze_repeats_first_enabled_frame() -> None:
    freeze = FrameFreeze[np.ndarray](enabled=True)
    assert np.array_equal(freeze.apply(np.array([1])), np.array([1]))
    assert np.array_equal(freeze.apply(np.array([2])), np.array([1]))
    freeze.enabled = False
    assert np.array_equal(freeze.apply(np.array([3])), np.array([3]))


def test_frame_delay_and_validation() -> None:
    delay = FrameDelay[int](2)
    assert [delay.apply(value) for value in (1, 2, 3, 4)] == [1, 1, 1, 2]
    with pytest.raises(ValueError):
        FrameDelay[int](-1)
