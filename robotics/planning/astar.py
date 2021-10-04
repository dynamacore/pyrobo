from robotics.planning.graph_search import GraphSearch, GridCell
from typing import List
import numpy as np

class AStarSearch(GraphSearch):
	def __init__(self, grid, validity_check = "occupied", adjacency=4, metric='grid') -> None:
		'''
		A star algorithm
		'''
		super().__init__(grid, validity_check=validity_check, adjacency=adjacency)
		self.metric = metric

	def sort_nodes(self, node: GridCell) -> float:
		'''
		Returns a nodes cost
		'''
		return node.total_cost
	
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
		Search the grid to find a path from start to goal
		'''
		self.start = start
		self.goal = goal
		open = [start]
		closed = []
		start.cost_to_come = 0
		start.total_cost = 0
		path_found = False

		while open and not path_found:
			# sort the queue and retrieve the lowest cost node
			open = sorted(open, key=self.sort_nodes)
			cur = open.pop(0)
			# maintain a closed list so we don't backtrack
			closed.append(cur)

			if cur == goal:
				goal.parent = cur.parent
				path_found = True
			
			# in the neighbors
			for node in self.neighbors(cur.x, cur.y):
				# calculate the cost to reach that node
				new_cost = cur.cost_to_come + self.edge_cost(cur, node)
				# if we haven't already seen this node
				if node not in closed and node not in open:
					# append it to the queue
					node.cost_to_come = new_cost
					# calculate the total cost with heuristic
					node.total_cost = new_cost + self.edge_cost(node, goal, metric='manhattan')
					open.append(node)
					node.parent = cur

				# or if we have seen it and new cost is lower
				elif new_cost < node.cost_to_come:
					# update the cost of the node
					node.cost_to_come = new_cost
					node.total_cost = new_cost + self.edge_cost(node, goal, metric='manhattan')
					# add it to the queue
					open.append(node)
					node.parent = cur

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
