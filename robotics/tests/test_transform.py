import pytest
import robotics
import numpy as np

@pytest.fixture
def transform_fixture_one():
    x, y, z, theta, phi, psi = 0.3, 0.5, 0.8, 0, 0, 0
    outT = robotics.Transform(x, y, z, theta, phi, psi)
    out = [outT, x, y, z, theta, phi, psi]
    return out

def test_pose_one(transform_fixture_one):
    x, y, z, theta, phi, psi = transform_fixture_one[1:]
    assert np.array_equal(transform_fixture_one[0].pose(), np.array([x, y, z, theta, phi, psi]).reshape(-1, 1))

def test_inv_one(transform_fixture_one):
    mat = transform_fixture_one[0]
    matInv = mat.inv()
    result = mat * matInv
    assert np.array_equal(result.transform, np.eye(4))
