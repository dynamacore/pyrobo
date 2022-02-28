import matplotlib.pyplot as plt
import numpy as np
from _pyrobo import Quaternion

def plot(self, detached=False, axis_obj=None, rgb_xyz=['r', 'g', 'b'], xlim=[-2, 2], ylim=[-2, 2], zlim=[-2, 2], scale_factor=1.0, view_angle=None):
	if detached:
		return __plot_detached(self, axis_obj, rgb_xyz=rgb_xyz, scale_factor=scale_factor)
	else:
		return __plot_attached(self, xlim=xlim, ylim=ylim, zlim=zlim, rgb_xyz=rgb_xyz, scale_factor=scale_factor, view_angle=view_angle)

def __plot_attached(self, xlim, ylim, zlim, rgb_xyz, scale_factor, view_angle):
	"""
	Plots the quaternion on internally provided matplotlib axes
	"""
	axis_obj = plt.axes(projection='3d')

	__plot_axes(self, axis_obj, rgb_xyz, scale_factor)

	axis_obj.set_xlim3d(xlim[0], xlim[1])
	axis_obj.set_ylim3d(ylim[0], ylim[1])
	axis_obj.set_zlim3d(zlim[0], zlim[1])
	axis_obj.set_xlabel("x")
	axis_obj.set_ylabel("y")
	axis_obj.set_zlabel("z")
	if view_angle is not None:
		axis_obj.view_init(view_angle[0], view_angle[1])

	plt.show()

def __plot_detached(self, axis_obj, rgb_xyz, scale_factor):
	"""
	Plots the axes on externally provided matplotlib axes
	"""
	__plot_axes(self, axis_obj, rgb_xyz, scale_factor)

def __plot_axes(self, axis_obj, rgb_xyz, scale_factor):
	"""
	Plot the 3 basis vectors in 3D space rotated by the quaternion
	"""
	origin = np.array([0, 0, 0])
	x_axis = np.array([1, 0, 0])
	y_axis = np.array([0, 1, 0])
	z_axis = np.array([0, 0, 1])
	try:
		# rotate all the axes
		x_axis = self*(scale_factor * x_axis ) + origin
		y_axis = self*(scale_factor * y_axis ) + origin
		z_axis = self*(scale_factor * z_axis ) + origin

		# collect plot values
		# i unit vectors
		iX, iY, iZ  = x_axis[0], x_axis[1], x_axis[2]
		# j unit vector
		jX, jY, jZ = y_axis[0], y_axis[1], y_axis[2]
		# k unit vector
		kX, kY, kZ = z_axis[0], z_axis[1], z_axis[2]
		# origin
		oX, oY, oZ = origin[0], origin[1], origin[2]
				
		axis_obj.plot([oX, iX], [oY, iY], [oZ, iZ], rgb_xyz[0])
		axis_obj.plot([oX, jX], [oY, jY], [oZ, jZ], rgb_xyz[1])
		axis_obj.plot([oX, kX], [oY, kY], [oZ, kZ], rgb_xyz[2])

	except AttributeError:
		raise AttributeError("axis_obj is None")

# Bind the method to the quaternion class as a pure python implementation
Quaternion.plot = plot