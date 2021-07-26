import numpy as np
import robotics as r
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class KinematicTree:
	""" 
	An ordered collection of transforms, most useful for creating a full robot
	"""
	def __init__(self, collection, root_index=0, root_name='base_link', debug=False):
		self.debug = debug
		self.rigid_collection = r.RigidCollection(collection)
		self.__root_name = root_name
		self.__tree_rep = 'incidence_list'
		self.__assemble_tree(root_name=root_name, root_index=root_index, tree_rep=self.__tree_rep)
		self.__root(root_name=root_name)

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
				# set edge as a backward edge
				edge_inv = edge.inv()
				edge_inv.set_backward()
				if child in tree_dict:
					tree_dict[child].append(edge_inv)
				else:
					tree_dict[child] = [edge_inv]
		
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
			if self.debug:
				print("[GET]")
				[print("\t", x.name) for x in path]
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
	
	
	def plot(self, display_arrows=True, axes_lim=5, scale_factor=1.0, rgb_xyz=['r', 'g', 'b'], detached=False, axis_obj=None, plot_frame=None, view_angle=None, transform_colors=None):
		'''
		Plots the kinematic trees with or without arrows
		'''
		# if display_arrows:
		# 	self.__plot_arrows(axes_lim=axes_lim, scale_factor=scale_factor, rgb_xyz=rgb_xyz, detached=detached, axis_obj=axis_obj)

		# else:
		# plot the tree in a certain frame
		if plot_frame is None:
			plot_frame = self.__root_name
		self.__plot_frames(axes_lim, scale_factor, rgb_xyz, detached, axis_obj, plot_frame, view_angle, transform_colors)

	def __plot_frames(self, axes_lim, scale_factor, rgb_xyz, detached, axis_obj, plot_frame, view_angle, transform_colors):
		'''
		Plots the tree within a certain frame. Note that RGB needs to be a dictionary with each frame name
		'''
		if not detached:
			fig = plt.figure()
			axis_obj = plt.subplot(111, projection='3d')
		frames = self.root(destination=plot_frame, root_name=self.__root_name)
		for frame in frames:
			transform = frames[frame]
			# selects the given color for a transform
			# transform colors should be a dictionary since this isn't a list
			if transform_colors is not None:
				rgb_xyz = transform_colors[frame]
				if isinstance(rgb_xyz, str):
					# to not get confused, dot lines
					rgb_xyz = [rgb_xyz, rgb_xyz + ':', rgb_xyz + '--']

			transform.plot(detached=True, axis_obj=axis_obj, scale_factor=scale_factor, rgb_xyz=rgb_xyz)
		
		axis_obj.set_xlim3d(-axes_lim, axes_lim)
		axis_obj.set_ylim3d(-axes_lim, axes_lim)
		axis_obj.set_zlim3d(-axes_lim, axes_lim)

		axis_obj.set_xlabel("$x$", fontsize=24)
		axis_obj.set_ylabel("$y$", fontsize=24)
		axis_obj.set_zlabel("$z$", fontsize=24)

		if view_angle is not None:
			axis_obj.view_init(view_angle[0], view_angle[1])

		if not detached:
			plt.show()
	
	def root(self, destination='base_link', root_name='base_link'):
		'''
		Transforms all frames into specified frame. If destination is not root, takes O(n) time to transform
		'''
		if destination == root_name:
			# __rooted frames is already in root 
			return self.__rooted_frames
		else:
			if self.debug: print("[ROOT] Transforming frames into '{0}' from {1}".format(destination, root_name))
			# obtain the transform to shift the entire tree to destination frame
			branched_frames = {}
			operator = self.__rooted_frames[destination].inv()
			for t in self.__rooted_frames:
				branched_frames[t] = operator * self.__rooted_frames[t]
			
			return branched_frames

	def __root(self, root_name='base_link'):
		'''
		Transforms every frame into the base link
		'''
		def expand(current):
			# expands a node by adding all the valid neighbors to the stack while extracting the transform
			edges = self.tree_rep[current]
			for edge in reversed(edges):
				# if we have a forward node
				if edge.get_forward():
					current = edge.child
					parent = edge.parent
					if self.debug: print("\t\t", parent, "->", current)
					if current not in open_nodes and current not in closed_nodes:
						open_nodes.append(edge.child) 
						# multiply the current node by the parent transform
						self.__rooted_frames[current] = self.__rooted_frames[parent] * edge

		# initialize the rooted frames list
		self.__rooted_frames = {root_name : r.Transform(parent=root_name, child=root_name)}
		# initialize open and closed lists
		open_nodes = [root_name]
		closed_nodes = []

		if self.debug: print("[ROOT]")
		while open_nodes:
			current = open_nodes.pop()
			if self.debug: print("\t", current)
			closed_nodes.append(current)
			expand(current)
		
		return self.__rooted_frames

