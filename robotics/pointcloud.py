import numpy as np
import robotics
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PointCloud:
	""" 
	A pointcloud data structure implemented with numpy to visualize transforms and clouds

	Args:

		cloud - Nx3 array of 3D points
		frame - frame of reference that the pointcloud coordinates are in
	"""
	def __init__(self, cloud=None, frame=None):
		if cloud is not None:
			self.cloud = cloud
			self.frame = frame

	def __str__(self):
		return "Frame: {0}\n".format(self.frame) + str(self.cloud)
	
	def __repr__(self):
		return "PointCloud({0}, {1})".format(self.cloud, self.frame)

	def transform(self, frame_from, frame_to, transform):
		""" 
		Transforms a point cloud according to the transform matrix and updates the frame member
		"""
		if isinstance(transform, robotics.Transform):
			operator = transform.transform
		else:
			operator = transform

		# add the homogenous coord
		print(self.cloud.shape)
		homog = np.hstack((self.cloud, np.ones((self.cloud.shape[0], 1)))).T
		print(homog.shape)
		newCloud = (operator @ homog)[:-1, :].T
		return PointCloud(cloud=newCloud, frame=frame_to)

	def plot(self, color='b', axes_lim=3, global_frame=False):
		"""
		Plots the cloud for visualization in its frame
		"""
		fig = plt.figure()
		ax = plt.subplot(111, projection='3d')

		# plot the cloud
		x = self.cloud[:, 0]
		y = self.cloud[:, 1]
		z = self.cloud[:, 2]

		# adjust adxes scaling
		ax.set_xlim3d(-axes_lim, axes_lim)
		ax.set_ylim3d(-axes_lim, axes_lim)
		ax.set_zlim3d(-axes_lim, axes_lim)

		ax.scatter(x, y, z, color=color)
		plt.show()


