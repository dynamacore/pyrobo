import numpy as np
from robotics import Quaternion


def test_quaternion_operators():
    test = Quaternion(1, 1, 1, 1) * 0.5
    compare = Quaternion(0.5, 0.5, 0.5, 0.5)
    assert test == compare

    test = 0.5 * Quaternion(1, 1, 1, 1)
    compare = Quaternion(0.5, 0.5, 0.5, 0.5)
    assert test == compare

    test = Quaternion(1, 1, 1, 1) / 2
    compare = Quaternion(0.5, 0.5, 0.5, 0.5)
    assert test == compare


def test_quaternion_basics():
    q = Quaternion(1.23, 4.3, 0.32, -0.23)
    assert np.isclose(q.norm(), 4.49, rtol=1e-3)

    assert q.inv().approx(Quaternion(0.06107, -0.2133, -0.0158, 0.0114), 1e-4)

    assert (q * q.inv()).approx(Quaternion(1.0, 0, 0, 0), 1e-16)


def test_two_quaternion_operations():
    q1 = Quaternion(1, -2, 3, 4)
    q2 = Quaternion(1, 2, 3, 4)
    assert q1 + q2 == Quaternion(2, 0, 6, 8)
    assert q1 - q2 == Quaternion(0, -4, 0, 0)
    assert q1 * q2 == Quaternion(-20, 0, 22, -4)

    q1 = Quaternion(0.89, -0.45, 4.1, 0.11)
    q2 = Quaternion(0.23, 2, 9, 5.243)
    assert (q1 + q2).approx(Quaternion(1.12, 1.55, 13.1, 5.353), atol=1e-4)
    assert (q1 - q2).approx(Quaternion(0.66, -2.45, -4.9, -5.133), atol=1e-4)
    assert (q1 * q2).approx(Quaternion(-36.3720, 22.1828, 11.5323, -7.5584), atol=1e-4)
    # testing inverses again
    assert (q1 * q1.inv()).approx(Quaternion(1, 0, 0, 0), atol=1e-12)
    assert (q2 * q2.inv()).approx(Quaternion(1, 0, 0, 0), atol=1e-12)


def test_rotations():
    q = Quaternion(1, 0, 0, 0)
    assert q.rotation()
    # rotation of euler angles XYZ = (30deg, 20deg, -10deg)
    q = Quaternion(0.9515485, 0.2392983, 0.1893079, -0.0381346)
    assert q.rotation(atol=1e-7)
    # rotation of 30deg around z-axis, which results in a 60 degree quaternion rotation
    q = Quaternion(0.9659258, 0, 0, 0.258819)
    assert q.rotation(atol=1e-7)

    #  rotate the zaxis, should not move
    p = np.array([0, 0, 1])
    assert np.allclose(q * p, np.array([0, 0, 1]))

    # rotate the x-axis
    p = np.array([1, 0, 0])
    assert np.allclose(q * p, np.array([0.8660254, 0.5, 0]))
