import robotics as r
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
	# plot the transform on externally provided axes; good for specifying one's own plot parameters
	fig = plt.figure()
	ax = plt.subplot(111, projection='3d')
	detachedPlot = r.Transform()
	detachedPlot.plot(detached=True, axisObj=ax)
	# need to include the show outside of the function
	plt.show()

	# plot the transform in attached mode, which just shows the transform in it's parent frame
	attachedPlot = r.Transform()
	attachedPlot.plot()