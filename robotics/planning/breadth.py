from robotics.planning.graph_search import GraphSearch, GridCell
import numpy 

class BreadthFirstSearch(GraphSearch):
	def __init__(self, grid, validity_check = "occupied", adjacency=4) -> None:
		'''
		This is breadth first search to find a path
		'''
		super().__init__(grid, validity_check=validity_check, adjacency=adjacency)
	
	def search(self, start : GridCell, goal : GridCell):
		'''
		Finds a path from start to goal
		'''
		open = [start]
		closed = []
		path_found = False

		while open and not path_found:
			# this is a queue now, first in last out
			cur = open[0]
			open.pop(0)
			closed.append(cur)

			if cur == goal:
				goal.parent = cur.parent
				path_found = True
			
			# search through the neighbors
			for node in self.neighbors(cur.x, cur.y):
				if node not in open and node not in closed and self.valid(node):
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

