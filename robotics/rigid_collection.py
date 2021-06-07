import numpy as np
from .transform import Transform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class RigidCollection():
    """ 
    A collection of transforms, but unordered as opposed to an ordered kinematic tree
    """

    def __init__(self, collection=None):
        self.collection = []

        if collection is not None:
            self.collection = collection

    def add(self, transform):
        """ 
        Adds transform to the collection
        """ 
        self.collection.append(transform)
        return self.collection

    def plot(self, axes_lim=5):
        """
        plot the collection
        """
        fig = plt.figure()
        ax = plt.subplot(111, projection='3d')

        # plot the transforms
        for transform in self.collection:
            transform.plot(ax)

        # adjust adxes scaling
        ax.set_xlim3d(-axes_lim, axes_lim)
        ax.set_ylim3d(-axes_lim, axes_lim)
        ax.set_zlim3d(-axes_lim, axes_lim)

        plt.show()