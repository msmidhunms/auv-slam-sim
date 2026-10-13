import os

from auv_control.allocation import allocation_matrix, ThrusterAllocator
import numpy as np
import pytest
import yaml

LAYOUT_FILE = os.path.join(
    os.path.dirname(__file__), '..', '..', 'auv_description', 'config', 'thrusters.yaml')


@pytest.fixture
def layout():
    with open(LAYOUT_FILE) as f:
        return yaml.safe_load(f)


@pytest.fixture
def allocator(layout):
    return ThrusterAllocator(layout['thrusters'], layout['max_thrust'])


def test_matrix_shape_and_rank(layout):
    matrix = allocation_matrix(layout['thrusters'])
    assert matrix.shape == (6, 8)
    assert np.linalg.matrix_rank(matrix) == 6


@pytest.mark.parametrize('axis', range(6))
def test_each_axis_is_reproduced(allocator, axis):
    wrench = np.zeros(6)
    wrench[axis] = 2.0
    thrusts = allocator.allocate(wrench)
    assert allocator.wrench(thrusts) == pytest.approx(wrench, abs=1e-4)


def test_surge_uses_only_horizontal_thrusters(allocator):
    thrusts = allocator.allocate([10.0, 0, 0, 0, 0, 0])
    assert thrusts[:4] == pytest.approx([10.0 / (4 * np.cos(np.pi / 4))] * 4, rel=1e-3)
    assert thrusts[4:] == pytest.approx([0.0] * 4, abs=1e-3)


def test_heave_uses_only_vertical_thrusters(allocator):
    thrusts = allocator.allocate([0, 0, -20.0, 0, 0, 0])
    assert thrusts[:4] == pytest.approx([0.0] * 4, abs=1e-3)
    assert thrusts[4:] == pytest.approx([-5.0] * 4, rel=1e-3)


def test_combined_wrench(allocator):
    wrench = np.array([5.0, -3.0, 8.0, 0.5, -0.4, 1.0])
    assert allocator.wrench(allocator.allocate(wrench)) == pytest.approx(wrench, abs=1e-4)


def test_saturation_keeps_direction(allocator, layout):
    wrench = np.array([500.0, 0, 0, 0, 0, 0])
    thrusts = allocator.allocate(wrench)
    assert np.max(np.abs(thrusts)) == pytest.approx(layout['max_thrust'])
    produced = allocator.wrench(thrusts)
    assert produced[0] > 0
    assert produced[1:] == pytest.approx(np.zeros(5), abs=1e-3)


def test_rank_deficient_layout_is_rejected():
    thrusters = [{'name': f't{i}', 'xyz': [0.1 * i, 0, 0], 'rpy': [0, 0, 0]} for i in range(8)]
    with pytest.raises(ValueError):
        ThrusterAllocator(thrusters, 40.0)
