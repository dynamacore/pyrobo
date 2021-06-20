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

	def transform(self, frame_from, frame_to, transform=None, transformation_matrix=None):
		""" 
		Transforms a point cloud according to the transform matrix and updates the frame member
		"""
		pass

	def plot(self):
		pass

