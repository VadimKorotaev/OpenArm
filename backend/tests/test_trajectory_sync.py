import pytest

from openarm_skills.trajectory_sync import duration, sample_at, sample_times


def ramp() -> list[tuple[float, tuple[float, ...]]]:
    return [(0.0, (0.0, 1.0)), (1.0, (1.0, 1.0)), (3.0, (1.0, 3.0))]


def test_sample_interpolates_between_surrounding_points() -> None:
    assert sample_at(ramp(), 0.25) == pytest.approx((0.25, 1.0))
    assert sample_at(ramp(), 2.0) == pytest.approx((1.0, 2.0))


def test_sample_holds_the_final_pose_past_the_end() -> None:
    # An arm that already stopped is still standing in the way of the other.
    assert sample_at(ramp(), 9.0) == pytest.approx((1.0, 3.0))
    assert sample_at(ramp(), -1.0) == pytest.approx((0.0, 1.0))


def test_sample_of_an_empty_trajectory_is_an_error() -> None:
    with pytest.raises(ValueError):
        sample_at([], 0.0)


def test_duration_is_the_last_point_time() -> None:
    assert duration(ramp()) == pytest.approx(3.0)
    assert duration([]) == pytest.approx(0.0)


def test_sample_times_span_the_whole_trajectory() -> None:
    times = sample_times(1.0, step=0.1)
    assert times[0] == pytest.approx(0.0)
    assert times[-1] == pytest.approx(1.0)
    assert len(times) == 11


def test_long_trajectories_are_coarsened_not_truncated() -> None:
    times = sample_times(60.0, step=0.1, max_samples=30)
    assert len(times) == 31
    assert times[-1] == pytest.approx(60.0)


def test_instant_trajectory_still_yields_one_check() -> None:
    assert sample_times(0.0) == [0.0]
