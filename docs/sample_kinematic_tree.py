import robotics

if __name__ == '__main__':
	baseLink = robotics.Transform(name="base_link")
	link1 = robotics.Transform(0.1, 0.5, 0, 0, 0, parent="base_link", name="link1")
	link2 = robotics.Transform(0.0, 0.0, 0.5, 0, 0, parent="base_link", name="link2")
	tree = robotics.KinematicTree([baseLink, link1, link2])
	tree.plot()