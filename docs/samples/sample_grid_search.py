import robotics.planning as p
import matplotlib.pyplot as plt
import numpy as np

def depth(grid):
	'''
	Depth first search using the robotics package
	'''
	depth = p.DepthFirstSearch(grid, adjacency=8)
	# prints the neighbors of this cell
	print("Neighbors of the cell [3, 6]: ", depth.neighbors(3, 6))

	start = p.GridCell(0, 0)
	goal = p.GridCell(6, 7)
	depth_path = depth.search(start, goal)

	if depth_path:
		print("Path found with Depth!")
		# set the path nodes to high weights to show them
		depth.display_path(depth_path)
	else:
		print("No path to goal found with Depth!")

def breadth(grid):
	'''
	Breadth first search using the robotics package
	'''
	start = p.GridCell(0, 0)
	goal = p.GridCell(4, 3)
	breadth = p.BreadthFirstSearch(grid, adjacency=4)
	breadth_path = breadth.search(start, goal)

	if breadth_path:
		print("Path found with Breadth!")
		breadth.display_path(breadth_path)
	else:
		print("No path to goal found with Breadth!")

def dijkstra(grid):
	'''
	Dijkstra's algorithm to find the least cost path
	'''


if __name__ == '__main__':
	# 8x8 grid to explore
	grid = np.array(
		[ 
			[0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 1, 1, 1, 1, 1, 0],
			[0, 0, 1, 0, 0, 0, 1, 0],
			[0, 0, 1, 0, 1, 1, 1, 0],
			[0, 0, 1, 0, 0, 1, 1, 0],
			[0, 0, 1, 0, 0, 0, 0, 0],
			[0, 0, 1, 0, 1, 1, 0, 0],
			[0, 0, 1, 0, 1, 1, 0, 0],
		]
	)

	depth(grid)
	breadth(grid)