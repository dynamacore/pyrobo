import robotics.planning as p
import matplotlib.pyplot as plt
import numpy as np

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
	dijk = p.DijkstraSearch(grid, adjacency=8)
	dijk_path = dijk.search(start, goal)

	if dijk_path:
		cost = 0
		for x in dijk_path:
			cost += x.cost_to_come
		print("Path found with cost {0} with Dijkstra's! \n\tLength:".format(cost), len(dijk_path), ". Expanded: ", dijk.expansions)
		dijk.display_path(dijk_path)
	else:
		print("No path found with Dijkstra's!")

def best_first(grid, start, goal):
	'''
	Best first search
	'''
	start = p.GridCell(start[0], start[1])
	goal = p.GridCell(goal[0], goal[1])

	best = p.BestFirstSearch(grid, adjacency=8)
	best_path = best.search(start, goal)
	if best_path:
		cost = 0
		for x in best_path:
			cost += x.cost_to_go
		print("Path found with cost {0} with Best First! \n\tLength:".format(cost), len(best_path), ". Expanded: ", best.expansions)
		best.display_path(best_path)
	else:
		print("No path found with Best First!")


def astar(grid, start, goal):
	'''
	A star grid search
	'''
	start = p.GridCell(start[0], start[1])
	goal = p.GridCell(goal[0], goal[1])

	astar = p.AStarSearch(grid, adjacency=8)
	astar_path = astar.search(start, goal)
	if astar_path:
		cost = 0
		for x in astar_path:
			cost += x.total_cost
		print("Path found with cost {0} with AStar! \n\tLength:".format(cost), len(astar_path), ". Expanded: ", astar.expansions)
		astar.display_path(astar_path)
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
	grid = generate_grid(30, 1)

	s = [0, 0]
	g = [15, 19]
	depth(grid,s, g)
	# Breadth and Dijkstra's both return optimal paths for grids with uniform edge costs
	breadth(grid, s, g)
	dijkstra(grid, s, g)
	best_first(grid, s, g)
	astar(grid, s, g)
