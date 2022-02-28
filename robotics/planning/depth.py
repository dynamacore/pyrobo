from typing import List
from robotics.planning.graph_search import GraphSearch, GridCell
import numpy as np

class DepthFirstSearch(GraphSearch):
	def __init__(self, grid, validity_check = "occupied", adjacency=4) -> None:
		'''
		This is a depth first search algorithm for find paths on grids
		'''
		super().__init__(grid, validity_check=validity_check, adjacency=adjacency)
	
	def search(self, start: GridCell, goal: GridCell) -> List[GridCell]:
		'''
		Finds a path from the start to the goal
		'''
		# this is a stack
		open = [start]
		closed = []
		path_found = False

		# now we use the stack in order to get the most recent item
		while open and not path_found: 
			cur = open.pop()
			closed.append(cur)

			# if we're at the goal then set the parent
			if cur == goal:
				goal.parent = cur.parent
				path_found = True

			for node in self.neighbors(cur.x, cur.y):
				if node not in closed and node not in open and self.valid(node):
					node.parent = cur
					open.append(node)

		# keep track of the number of expansions
		self.expansions = len(closed)
		previous = goal.parent
		path = None
		if path_found:
			path = []
			while previous is not None:
				path.append(previous)
				previous = previous.parent
		
			# return path from start to goal
			path.reverse()
		return path