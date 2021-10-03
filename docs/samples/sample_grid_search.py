import robotics.planning as p
import matplotlib.pyplot as plt
import numpy as np

from robotics.planning.dijkstra import DijkstraSearch

def depth(grid, start, goal):
	'''
	Depth first search using the robotics package
	'''
	depth = p.DepthFirstSearch(grid, adjacency=8)
	# prints the neighbors of this cell
	print("Neighbors of the cell [3, 6]: ", depth.neighbors(3, 6))

	start = p.GridCell(start[0], start[1])
	goal = p.GridCell(goal[0], goal[1])
	depth_path = depth.search(start, goal)

	if depth_path:
		print("Path found with Depth! \n\tLength: ", len(depth_path), ". Expanded: ", depth.expansions)
		# set the path nodes to high weights to show them
		depth.display_path(depth_path)
	else:
		print("No path to goal found with Depth!")

def breadth(grid, start, goal):
	'''
	Breadth first search using the robotics package
	'''
	start = p.GridCell(start[0], start[1])
	goal = p.GridCell(goal[0], goal[1])
	breadth = p.BreadthFirstSearch(grid, adjacency=8)
	breadth_path = breadth.search(start, goal)

	if breadth_path:
		print("Path found with Breadth! \n\tLength: ", len(breadth_path), ". Expanded: ", breadth.expansions)
		breadth.display_path(breadth_path)
	else:
		print("No path to goal found with Breadth!")

def dijkstra(grid, start, goal):
	'''
	Dijkstra's algorithm to find the least cost path
	'''
	start = p.GridCell(start[0], start[1])
	goal = p.GridCell(goal[0], goal[1])
	dijk = DijkstraSearch(grid, adjacency=8)
	dijk_path = dijk.search(start, goal)

	if dijk_path:
		cost = 0
		for x in dijk_path:
			cost += x.cost_to_come
		print("Path found with cost {0} with Dijkstra's! \n\tLength:".format(cost), len(dijk_path), ". Expanded: ", dijk.expansions)
		dijk.display_path(dijk_path)
	else:
		print("No path found with Dijkstra's!")

def generate_grid(n, seed, show=False):
	grid = np.zeros((n, n))
	# obstacles
	np.random.seed(seed)
	for i in range(n//10):
		i, j = np.random.randint(0, n), np.random.randint(0, n)
		i2, j2 = np.random.randint(0, n), np.random.randint(0, n)
		if grid[i, j] != 1 and grid[i2, j2] != 1:
			grid[i:i2, j:j2] = 1

	# generate individual obstacles
	for i in range(2*n):
		i, j = np.random.randint(0, n), np.random.randint(0, n)
		if grid[i, j] != 1:
			grid[i, j] = 1
	if show: 
		plt.imshow(grid)
		plt.show()
	return grid

if __name__ == '__main__':
	# 8x8 grid to explore
	# grid = np.array(
	# 	[ 
	# 		[0, 0, 1, 0, 1, 0, 0, 0],
	# 		[0, 0, 0, 0, 0, 0, 0, 0],
	# 		[0, 0, 1, 1, 1, 0, 1, 0],
	# 		[0, 0, 1, 0, 0, 0, 0, 0],
	# 		[0, 0, 1, 0, 1, 1, 0, 0],
	# 		[0, 1, 1, 0, 0, 1, 1, 0],
	# 		[0, 0, 0, 0, 0, 0, 0, 0],
	# 		[0, 0, 0, 0, 1, 1, 0, 0],
	# 		[0, 0, 0, 0, 1, 1, 0, 0],
	# 	]
	# )
	grid = generate_grid(50, 1)

	s = [0, 0]
	g = [15, 19]
	depth(grid,s, g)
	# seems like both breadth and dijkstra's return optimal paths
	breadth(grid, s, g)
	dijkstra(grid, s, g)