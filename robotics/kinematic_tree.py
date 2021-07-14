from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import robotics as r
import matplotlib.pyplot as plt

class KinematicTree:
	""" 
	An ordered collection of transforms, most useful for creating a full robot
	"""
	def __init__(self, collection, root_index=0, root_name=None):
		self.rigid_collection = r.RigidCollection(collection)
		self.__assemble_tree(root_name=root_name, root_index=root_index)

	def __assemble_tree(self, root_index, root_name):
		'''
		Assembles the collection of transforms into a single tree representation, enabling each transform to only need their parent and child when passed to the tree object.
		'''
		# we are given the edges with each parent and child
		tree_dict = {}
		# get the root from the index
		root = self.rigid_collection.collection[root_index]
		# get the root from the name
		if root_name is not None:
			root = self.rigid_collection.lookup(root_name)
		tree_dict[root.name] = []
		# traverse the edges first
		for edge in self.rigid_collection.collection:
			# if the edge's parent is in the tree dictionary, add it
			parent = edge.parent
			if parent is not None:
				if edge.parent in tree_dict:
					tree_dict[edge.parent].append(edge)
				else:
					# otherwise create a list starting with the edge
					tree_dict[edge.parent] = [edge]
		
		self.tree_dict = tree_dict


	def __traverse_tree(self, root_index=0):
		'''
		Traverse the tree given the collection of transforms with the "parent" and "child" fields filled out.
		Each transform passed to the tree's collection is an edge on the graph. The nodes are the described "parent" and "child" frames. 
		'''
		root = self.rigid_collection.collection[root_index]

		# perform depth first search traversal of the tree
		for t in self.rigid_collection.collection:
			print(t.child)
	
	def plot(self, display_arrows=True, axes_lim=5, scale_factor=1.0, rgb_xyz=['r', 'g', 'b'], detached=False, axis_obj=None):
		'''
		Plots the kinematic trees with or without arrows
		'''
		if display_arrows:
			self.__plot_arrows(axes_lim=axes_lim, scale_factor=scale_factor, rgb_xyz=rgb_xyz, detached=detached, axis_obj=axis_obj)

		else:
			self.rigid_collection.plot(axes_lim=axes_lim, scale_factor=scale_factor, rgb_xyz=rgb_xyz, detached=detached, axisObj=axis_obj)

	def __plot_arrows(self, axes_lim, scale_factor, rgb_xyz, detached, axis_obj):
		'''
		Plots the kinematic tree with arrows connecting each frame to its parent
		'''
		if not detached:
			fig = plt.figure()
			axis_obj = plt.subplot(111, projection='3d')

		# plot the transforms
		for transform in self.rigid_collection.collection:
			transform.plot(detached=True, axis_obj=axis_obj, scale_factor=scale_factor, rgb_xyz=rgb_xyz)

		# adjust adxes scaling
		axis_obj.set_xlim3d(-axes_lim, axes_lim)
		axis_obj.set_ylim3d(-axes_lim, axes_lim)
		axis_obj.set_zlim3d(-axes_lim, axes_lim)

		if not detached:
			plt.show()

