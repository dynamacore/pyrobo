import pytest
import robotics
import numpy as np


@pytest.fixture
def fixture_collection():
    collection = robotics.RigidCollection()
    return collection


def test_add_transform(fixture_collection):
    """
    Tests adding a transform to the rigid collection
    """
    transform = robotics.Transform()
    fixture_collection.add(transform)
    assert len(fixture_collection.collection) == 1
