from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import robotics as r
import matplotlib.pyplot as plt

class KinematicTree:
	""" 
	An ordered collection of transforms, most useful for creating a full robot
	"""
	def __init__(self, collection, root_index=0, root_name=None, debug=False):
		self.debug = debug
		self.rigid_collection = r.RigidCollection(collection)
		self.__tree_rep = 'incidence_list'
		self.__assemble_tree(root_name=root_name, root_index=root_index, tree_rep=self.__tree_rep)

	def set_tree_rep(self, rep_type):
		self.__tree_rep = rep_type

	def __assemble_tree(self, root_index, root_name, tree_rep):
		'''
		Assembles the collection of transforms into a single tree representation, enabling each transform to only need their parent and child when passed to the tree object.
		'''
		if tree_rep == 'incidence_list':
			self.tree_rep = self.__assemble_incidence_list(root_index, root_name)
			return True
		else:
			return False
	
	def __assemble_incidence_list(self, root_index, root_name):
		'''
		Assemble an incidence representation of the tree, where each node contains the transform edge in the dictionary
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
				if parent in tree_dict:
					tree_dict[parent].append(edge)
				else:
					# otherwise create a list starting with the edge
					tree_dict[parent] = [edge]
			# now process the children, adding inverse edges
			child = edge.child
			if child is not None:
				if child in tree_dict:
					tree_dict[child].append(edge.inv())
				else:
					tree_dict[child] = [edge.inv()]
		
		return tree_dict
	
	def __assemble_adjacency_list(self, root_index, root_name):
		'''
		Assemble an adjacency list representation of the tree
		'''
		# get the root from the index
		root = self.rigid_collection.collection[root_index]
		# get the root from the name
		if root_name is not None:
			root = self.rigid_collection.lookup(root_name)

		adjacency_dict = {}
		for edge in self.rigid_collection.collection:
			parent = edge.parent
			child = edge.child
			if parent is not None:
				if parent in adjacency_dict:
					adjacency_dict[parent].append(child)
				if parent not in adjacency_dict:
					adjacency_dict[parent] = [child]
		
		return adjacency_dict
	
	def __assemble_adjacency_matrix(self):
		'''
		Assemble an adjacency matrix representation of the tree
		'''
		# need to get all the node names first
		nodes  = []
		for edge in self.rigid_collection.collection:
			parent = edge.parent
			child = edge.child
			if parent is not None:
				if parent not in nodes:
					nodes.append(parent)
			if child is not None:
				if child not in nodes:
					nodes.append(child)
		
		# now we can assemble an adjacency matrix in O(n*e) time where n is node number and e is edge number
		adj_matrix = np.zeros((len(nodes), len(nodes)))
		for edge in self.rigid_collection.collection:
			parent = edge.parent
			child = edge.child

			if self.debug: print("{0} -> {1}".format(parent, child))
			# ignore the root
			if parent is not None:
				p_idx = nodes.index(parent)
				c_idx = nodes.index(child)
				adj_matrix[p_idx, c_idx] = 1
				adj_matrix[c_idx, p_idx] = -1

		return adj_matrix


	def get(self, start_frame, end_frame):
		'''
		Retrieves the transform between two frames on the tree
		'''
		# find a path if one exists
		success, edges_traversed = self.__breadth_first_search(start_frame, end_frame)
		if success:
			# start from the goal
			edge = edges_traversed[end_frame]
			path_complete = False
			path = []
			# cycle through the back pointers to get the path
			while not path_complete:
				back_pointer = edge.parent
				if back_pointer is not None:
					path.append(edge)
					if back_pointer == start_frame:
						path_complete = True
					edge = edges_traversed[back_pointer]

			# reverse the path to start from the start frame
			path.reverse()
			if self.debug: [print(x.name) for x in path]
		# return the path multiplied together
		return np.prod(path)

	def __breadth_first_search(self, start_frame, end_frame):
		'''
		Find a path between the start and goal transform using breadth first search 
		'''
		# define expansion function in this context
		def expand_node_BFS(node):
			edges= self.tree_rep[node]
			for edge in edges:
				if edge.child == end_frame: 
					backpointers[edge.child] = edge
					return True
				elif edge.child not in open_nodes and edge.child not in closed_nodes:
					backpointers[edge.child] = edge
					open_nodes.append(edge.child)
			
			return False

		# utilize open and closed node lists
		open_nodes = [start_frame]
		closed_nodes = []
		# dict for saving backpointers
		backpointers = {start_frame: None}
		while len(open_nodes) != 0:
			node = open_nodes.pop(0)
			closed_nodes.append(node)

			if expand_node_BFS(node): return True, backpointers
		return False, backpointers
	
	
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
	
	def root(self, root_name='base_link'):
		'''
		Rearranges the tree such that all frames are expressed in terms of the given root node. No recursion because recursion is wack. 
		'''
		# initialize stack
		open_nodes = [root_name]
		while len(open_nodes) > 0:
			node = open_nodes.pop(0)
			edge = self.tree_rep[node]
			print(edge.child)

