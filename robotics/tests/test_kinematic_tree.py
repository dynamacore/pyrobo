import robotics
import pytest
import numpy as np

@pytest.fixture
def fixture():
	baseLink = robotics.Transform(name="base_link")
	link1 = robotics.Transform(0.1, 0.5, 0, 0, 0, parent="base_link", child="link1", name="bTo1")
	link2 = robotics.Transform(0.0, 0.0, 0.5, 0, 0, parent="base_link", child="link2", name="bTo2")
	link3 = robotics.Transform(0.0, 0.0, -0.2, np.pi, 0, 0, parent="link1", child="link3", name="3To1")
	link4 = robotics.Transform(0.0, 0.2, 0.0, 0.0, 0, 0, parent="link3", child="link4", name="4To3")
	link5 = robotics.Transform(0.0, 0.0, 0.0, 0.0, 0, 0, parent="link3", child="link5", name="5To3")
	tree = robotics.KinematicTree([baseLink, link1, link2, link3, link4, link5])
	return tree

def test_forward_and_back_get(fixture):
	'''
	Tests that the tree finds a path between two nodes, and that swapping the order returns the inverse
	'''
	forward = fixture.get('link2', 'link5')
	backward = fixture.get('link5', 'link2')
	inv = forward * backward
	assert np.allclose(inv.transform, np.eye(4))

def test_simple_path():
	'''
	Test a simple path is returning the correct transform
	'''
	base_link = robotics.Transform(name="base_link")
	hand = robotics.Transform(x=0.5, parent="base_link", child="hand", name="hand")
	box = robotics.Transform(x=1.0, z=0.5, parent="hand", child="box", name="box")
	tree = robotics.KinematicTree([base_link, hand, box])
	path = tree.get('base_link', 'box')
	compare = robotics.Transform(x=1.5, z=0.5, parent='base_link', child='box')
	assert compare == path
	path = tree.get('box', 'base_link')
	compare = robotics.Transform(x=-1.5, z=-0.5, parent='box', child='base_link')
	assert compare == path

def test_root(fixture):
	# get everything in the root frame
	grounded = fixture.root()
	assert len(grounded) == len(fixture.tree_rep)
	for g in grounded:
		assert grounded[g].parent == 'base_link'

def test_root_comparison(fixture):
	'''
	The manually computed transform, rooted transform, and BFS search transform should all be the same
	'''
	grounded = fixture.root()
	assert grounded['link5'] == robotics.Transform(0.1, 0.5, -0.2, np.pi, 0, 0, parent='base_link', child='link5') == fixture.get('base_link', 'link5')

def test_root_arbitrary_frame(fixture):
	grounded = fixture.root(destination='link2')
	assert grounded['link5'] == robotics.Transform(0.1, 0.5, -0.7, np.pi, 0, 0, parent='link2', child='link5') == fixture.get('link2', 'link5')

@pytest.fixture
def new_name_fixture():
	'''
	For testing a tree with a base link named something new
	'''
	base_link = robotics.Transform(name="rock")
	hand = robotics.Transform(x=0.5, parent="rock", child="hand", name="hand")
	box = robotics.Transform(x=1.0, z=0.5, parent="hand", child="box", name="box")
	tree = robotics.KinematicTree([base_link, hand, box], root_name="rock")
	return tree

def test_new_name(new_name_fixture):
	grounded = new_name_fixture.root()
	assert len(grounded) == len(new_name_fixture.tree_rep)
	for g in grounded:
		assert grounded[g].parent == 'rock'

def test_new_name_compare(new_name_fixture):
	path = new_name_fixture.get('rock', 'box')
	compare = robotics.Transform(x=1.5, z=0.5, parent='rock', child='box')
	assert compare == path
	path = new_name_fixture.get('box', 'rock')
	compare = robotics.Transform(x=-1.5, z=-0.5, parent='box', child='rock')
	assert compare == path

@pytest.fixture
def complicated():
	chassis = robotics.Transform(name="chassis")
	link1 = robotics.Transform(0.125, -0.5, -0.5, 0, 0, np.pi/3, parent="chassis", child="link1", name="bTo1")
	link2 = robotics.Transform(0.125, 0.5, -0.5, 0, 0, -np.pi/3, parent="chassis", child="link2", name="bTo2")
	link3 = robotics.Transform(0.0, 0.0, -0.2, np.pi, 0.5426, 0.6392, parent="link1", child="link3", name="3To1")
	link4 = robotics.Transform(1.2643, -0.5, -1.0, 0.0, 0.523, 0, parent="link3", child="link4", name="4To3")
	link5 = robotics.Transform(1.2643, 0.5, -1.0, 0.0, 1.532, 0, parent="link3", child="link5", name="5To3")
	tree = robotics.KinematicTree([chassis, link1, link2, link3, link4, link5], root_name='chassis')
	return {'tree': tree, 'link1' : link1, 'link2' : link2, 'link3' : link3, 'link4':link4, 'link5':link5}

def test_complex_traversal(complicated):
	forward = complicated['tree'].get('link5', 'link2')
	backward = complicated['tree'].get('link2', 'link5')
	inv = forward * backward
	assert np.allclose(inv.transform, np.eye(4))

def test_complex_root(complicated):
	grounded = complicated['tree'].root()
	for g in grounded:
		assert grounded[g].parent == 'chassis'
	
	test = grounded['link5']
	multiply = complicated['link1'] * complicated['link3'] * complicated['link5']
	get = complicated['tree'].get('chassis', 'link5')
	assert test == get == multiply
