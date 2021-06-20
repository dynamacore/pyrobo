import robotics
import numpy as np

if __name__ == '__main__':
	cloud = np.array([
		[0, 0, 0], 
		[3, 5, 3], 
		[3, 5, 3], 
		[5, 5, 5], 
		[3, 3, 3], 
		[2, 2, 2], 
		[9, 9, 9], 
		[1, 5, 4], 
		[3, 2, 6], 
	])
	cloudObj = robotics.PointCloud(cloud=cloud)

	print("Cloud has points:\n", cloudObj)
	cloudObj.plot()