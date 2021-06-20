import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PointCloud:
	""" 
	A pointcloud data structure implemented with numpy to visualize transforms and clouds
	"""
	def __init__(self, cloud=None, frame=None):
		if cloud is not None:
			self.cloud = cloud
			self.frame = frame

	def __str__(self):
		return str(self.cloud)
	
	def __repr__(self):
		return "PointCloud({0}, {1})".format(self.cloud, self.frame)

	def transform(self, frame_from, frame_to, transform):
		""" 
		Transforms a point cloud according to the transform matrix and updates the frame member
		"""
		pass

	def plot(self, color='b'):
		"""
		Plots the cloud for visualization
		"""
		fig = plt.figure()
		ax = plt.subplot(111, projection='3d')

		# plot the cloud
		x = self.cloud[:, 0]
		y = self.cloud[:, 1]
		z = self.cloud[:, 2]
		ax.scatter(x, y, z, color=color)
		plt.show()


