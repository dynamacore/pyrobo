from typing import List
from robotics.planning.graph_search import GraphSearch, GridCell
import numpy as np

class BestFirstSearch(GraphSearch):
	def __init__(self, grid, validity_check = "occupied", adjacency=4, metric='grid') -> None:
		'''
		Best first search using a heuristic
		'''
		super().__init__(grid, validity_check=validity_check, adjacency=adjacency)
		self.metric = metric
	
	def sort_nodes(self, node: GridCell) -> float:
		'''
		Returns the node's heuristic cost
		'''
		return node.cost_to_go
	
	def edge_cost(self, from_node: GridCell, to_node: GridCell, metric=None) -> float:
		'''
		Returns the cost of a movement using the selected metric to calculate costs
		'''
		# User overridden metric
		if metric is None:
			metric = self.metric

		# Manhattan distance
		if metric == 'manhattan': return abs(to_node.x - from_node.x) + abs(to_node.y - from_node.y)

		# Grid movement
		if metric == 'grid': return self.grid[to_node.x, to_node.y]

		# Euclidean distance
		if metric == 'euclidean': return np.sqrt((from_node.x - to_node.x)**2 + (from_node.y - to_node.y)**2)


	def search(self, start: GridCell, goal: GridCell) -> List[GridCell]:
		'''
		Search through a grid to find a path in a low number of expansions
		'''
		self.start = start
		self.goal = goal

		# priority queue sorted by cost to go
		open = [start]
		closed = []
		start.cost_to_go = 0
		path_found = False

		while open and not path_found:
			open = sorted(open, key=self.sort_nodes)
			cur = open.pop(0)
			# maintain a closed list
			closed.append(cur)

			# if current node is goal break
			if cur == goal:
				goal.parent = cur.parent
				path_found = True

			# check all the neighbors
			for node in self.neighbors(cur.x, cur.y):
				if node not in closed and node not in open:
					# calculate the heuristic from the node to the goal
					node.cost_to_go = self.edge_cost(node, goal, metric='manhattan')
					node.parent = cur
					# add the node to the open list
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