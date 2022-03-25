import pytest
import robotics
import numpy as np


@pytest.fixture
def transform_fixture_one():
    """
    Prepare fixture for the pose tests
    """
    x, y, z, theta, phi, psi = 0.3, 0.5, 0.8, 0, 0, 0
    outT = robotics.Transform(x, y, z, theta, phi, psi)
    out = [outT, x, y, z, theta, phi, psi]
    return out


def test_inv_one(transform_fixture_one):
    """
    Testing inverse function for the fixture
    """
    mat = transform_fixture_one[0]
    matInv = mat.inv()
    result = mat * matInv
    assert np.array_equal(result.transform, np.eye(4))


def test_pose_one(transform_fixture_one):
    """
    Testing that pose() method works
    """
    x, y, z, theta, phi, psi = transform_fixture_one[1:]
    assert np.array_equal(
        transform_fixture_one[0].pose(),
        np.array([x, y, z, theta, phi, psi]).reshape(-1, 1),
    )


def test_inverse_pose_no_rotation(transform_fixture_one):
    """
    Testing the inverse pose return method works
    """
    pose = transform_fixture_one[1:]
    compare = transform_fixture_one[0].inverse_pose()
    assert np.allclose(compare, pose)


def test_inverse_pose_full_rotation():
    """
    Testing inverse pose for full rotation
    """
    pose = [0.23, 1.5, 0.89, 1.5, 0.5, 0.4]
    x, y, z, theta, phi, psi = pose
    outT = robotics.Transform(x, y, z, theta, phi, psi)
    out = outT.inverse_pose()
    assert np.allclose(out, pose)


def test_inverse_pose_full_rotation_neg():
    """
    Testing inverse pose for negative rotations
    """
    pose = [0.23, 1.5, 0.89, -1.5, 0.5, -0.4]
    x, y, z, theta, phi, psi = pose
    outT = robotics.Transform(x, y, z, theta, phi, psi)
    out = outT.inverse_pose()
    assert np.allclose(out, pose)


def test_inv_one_angle():
    """
    Testing inverse function when the transform has one angle
    """
    mat = robotics.Transform(
        x=66,
        y=93,
        z=0.567,
        theta=0.342,
        phi=0.0,
        psi=0.0,
    )
    matInv = mat.inv()
    result = mat * matInv
    assert np.allclose(result.transform, np.eye(4))


def test_inv_all_angles():
    """
    Testing inverse function with all 3 angles
    """
    mat = robotics.Transform(
        x=66,
        y=93,
        z=0.567,
        theta=0.15,
        phi=-0.9,
        psi=0.3,
    )
    matInv = mat.inv()
    result = mat * matInv
    assert np.allclose(result.transform, np.eye(4))


def test_inverse_pose_singularity():
    """
    There is a singularity at theta=pi/2, so test to make sure the function can still return inverse pose
    """
    pose = [2.5, 1.5, 0.89, np.pi / 2, 0.5, -0.4]
    x, y, z, theta, phi, psi = pose
    outT = robotics.Transform(x, y, z, theta, phi, psi)
    out = outT.inverse_pose()
    assert np.allclose(out, pose)


def test_equality():
    """
    Test equality overload
    """
    one = robotics.Transform(x=0.5)
    two = robotics.Transform(x=0.5)
    assert one == two


def test_equality_name():
    """
    Test equality overload with different names
    """
    one = robotics.Transform(x=0.5, name="one")
    two = robotics.Transform(x=0.5)
    assert not one == two


def test_equality_nodes():
    """
    Test equaltity operator for different parents, children, and names
    """
    one = robotics.Transform(
        0.32,
        0.67,
        0.123,
        0.56,
        np.pi / 2,
        23.4,
        parent="base",
        child="nase",
        name="base2nase",
    )
    two = robotics.Transform(
        0.32,
        0.67,
        0.123,
        0.56,
        np.pi / 2,
        23.4,
        parent="base",
        child="nase",
        name="base2nase",
    )
    assert one == two
    # change parent name
    two.parent = "flase"
    assert not one == two
    # back to equal
    two.parent = "base"
    assert one == two
    # change name
    two.name = "space"
    assert not one == two
    # change child
    two.name = "base"
    two.child = "space"
    assert not one == two


def test_multiply():
    """
    Test multiplication overload
    """
    one = robotics.Transform(x=0.5)
    two = robotics.Transform(x=0.5)
    compare = one.transform @ two.transform
    assert np.allclose(compare, (one * two).transform)


@pytest.fixture
def frames():
    """
    Prepares set of frames to multiply together
    """
    base = robotics.Transform(parent="base", child="base")
    one = robotics.Transform(parent="base", child="one")
    two = robotics.Transform(parent="one", child="two")
    three = robotics.Transform(parent="two", child="three")
    four = robotics.Transform(parent="two", child="four")
    return [base, one, two, three, four]


def test_multiply_frames(frames):
    """
    Tests coordinate transform algebra
    """
    base, one, two, three, four = frames
    # source = one, dest = base
    new = base * one
    assert new.parent == base.parent
    assert new.child == one.child


def test_multiply_many_frames(frames):
    base, one, two, three, four = frames
    # source = three, dest = base
    new = base * one * two * three
    assert new.parent == base.parent
    assert new.child == three.child


def test_multiply_inverses_frames(frames):
    base, one, two, three, four = frames
    inv = two.inv()
    new = two * inv
    assert new.parent == new.child


def test_multiply_non_matching_frames(frames):
    base, one, two, three, four = frames
    new = four * one
    assert new.parent == new.child is None


def test_updating_one_variable():
    test = robotics.Transform()
    test.update_transform(x=1)
    assert test.x == 1

    test.update_transform(phi=-1)
    assert test.phi == -1

    test.update_transform(psi=10)
    assert test.psi == 10


if __name__ == "__main__":
    one = robotics.Transform(x=0.5)
    two = robotics.Transform(x=0.5)
    print(one == two)
