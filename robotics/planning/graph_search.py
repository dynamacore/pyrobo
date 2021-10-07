from typing import List
import numpy as np
import matplotlib.pyplot as plt

class GridCell:
	def __init__(self, x, y, g=np.inf, h=0, f = np.inf) -> None:
		self.x = x
		self.y = y
		self.parent = None
		# intialize the costs of the node for weighted algorithms
		self.cost_to_come = g
		self.cost_to_go = h
		self.total_cost = f
	
	def __eq__(self, o: object) -> bool:
		if (self.x == o.x and self.y == o.y):
			return True
		return False
	
	def __repr__(self) -> str:
		return "[{0}, {1}]".format(self.x, self.y)
	
	def __hash__(self) -> int:
		return hash(self.x) + hash(self.y) + hash(self.cost_to_come) + hash(self.cost_to_go) + hash(self.parent)

class GraphSearch:
	'''
	High level class for graph search algos that all use similar functions
	'''
	def __init__(self, grid, validity_check="occupied", adjacency=4) -> None:
		self.grid = grid
		self.validity_check = validity_check
		self.adjacency = adjacency
		# store number of expansions
		self.expansions = 0

	def neighbors(self, i : int, j : int) -> List[GridCell]:
		'''
		Returns list of neighbors for a grid cell i,j
		'''
		i_low, i_high = 0, self.grid.shape[0]
		j_low, j_high = 0, self.grid.shape[1]
		
		# check if the neighbors will be in bounds
		def in_bounds(point : list):
			i, j = point
			return i < i_high and i >= i_low and j < j_high and j >= j_low

		output = []
		# initialize at i,j even though we will change it
		neighbors = [
			[i, j]
		]

		if self.adjacency == 4:
			# we pick neighbors up, down, left, right
			neighbors = [
				[i, j - 1], 
				[i, j + 1], 
				[i - 1, j], 
				[i + 1, j], 
			]
		elif self.adjacency == 8:
			neighbors = [
				[i     , j - 1] ,
				[i     , j + 1] ,
				[i - 1 , j]     ,
				[i + 1 , j]     ,
				[i - 1 , j - 1] ,
				[i -1  , j + 1] ,
				[i +1  , j - 1] ,
				[i + 1 , j+ 1]
			]
		for point in neighbors:
			if in_bounds(point):
				node = GridCell(point[0], point[1])
				output.append(node)

		return output
			
	def valid(self, node : GridCell) -> bool:
		'''
		Decides whether a node is valid or not, this can take multiple forms
		'''
		if self.validity_check == "occupied":
			return self.grid[node.x, node.y] == 0

	def display_path(self, path : List[GridCell]) -> None:
		'''
		Way to display the path through time
		'''
		display = np.copy(self.grid)
		for node in path:
			display[node.x, node.y] = 5
			plt.imshow(display)
			plt.draw()
			plt.pause(0.01)

		plt.show()