import robotics
import numpy as np

if __name__ == '__main__':
	baseLink = robotics.Transform(name="base_link")
	link1 = robotics.Transform(0.1, 0.5, 0, 0, 0, parent="base_link", child="link1", name="bTo1")
	link2 = robotics.Transform(0.0, 0.0, 0.5, 0, 0, parent="base_link", child="link2", name="bTo2")
	link3 = robotics.Transform(0.0, 0.0, -0.2, np.pi, 0, 0, parent="link1", child="link3", name="3To1")
	link4 = robotics.Transform(0.0, 0.2, 0.0, 0.0, 0, 0, parent="link3", child="link4", name="4To3")
	link5 = robotics.Transform(0.0, 0.0, 0.0, 0.0, 0, 0, parent="link3", child="link5", name="5To3")
	tree = robotics.KinematicTree([baseLink, link1, link2, link3, link4, link5])
	tree.plot()