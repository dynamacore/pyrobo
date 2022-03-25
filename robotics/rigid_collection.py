import numpy as np
from .transform import Transform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class RigidCollection:
    """
    A collection of transforms, but unordered as opposed to an ordered kinematic tree
    """

    def __init__(self, collection=None, name=None):
        self.collection = []
        self.name = None

        if collection is not None:
            self.collection = collection
            self.__assemble_collection_dict()

        if name is not None:
            self.name = name

    def add(self, *transforms):
        """
        Adds transforms to the collection
        """
        for t in transforms:
            # check to see if there's a duplicate already in the collection
            # for example if naming a transform already added to the collection
            if t not in self.collection:
                self.collection.append(t)
            # reassemble the collection dictionary
            self.__assemble_collection_dict()
        return self.collection

    def plot(
        self,
        axes_lim=5,
        scale_factor=1.0,
        rgb_xyz=["r", "g", "b"],
        detached=False,
        axis_obj=None,
        view_angle=None,
        transform_colors=None,
        text=False,
        fontsize=16,
    ):
        """
        plot the collection in either attached or detached mode
        """
        if not detached:
            fig = plt.figure()
            axis_obj = plt.subplot(111, projection="3d")

        # plot the transforms
        for t_idx in range(len(self.collection)):
            transform = self.collection[t_idx]
            # selects the given color for a transform
            if transform_colors is not None:
                rgb_xyz = transform_colors[t_idx]
                if isinstance(rgb_xyz, str):
                    # to not get confused, dot lines
                    rgb_xyz = [rgb_xyz, rgb_xyz + ":", rgb_xyz + "--"]

            transform.plot(
                detached=True,
                axis_obj=axis_obj,
                scale_factor=scale_factor,
                rgb_xyz=rgb_xyz,
            )
            if text:
                axis_obj.text(
                    transform.x + transform.x / 50,
                    transform.y + transform.y / 50,
                    transform.z + transform.z / 50,
                    transform.name,
                    fontsize=fontsize,
                )

        # adjust adxes scaling
        axis_obj.set_xlim3d(-axes_lim, axes_lim)
        axis_obj.set_ylim3d(-axes_lim, axes_lim)
        axis_obj.set_zlim3d(-axes_lim, axes_lim)

        axis_obj.set_xlabel("$x$", fontsize=fontsize)
        axis_obj.set_ylabel("$y$", fontsize=fontsize)
        axis_obj.set_zlabel("$z$", fontsize=fontsize)

        if view_angle is not None:
            axis_obj.view_init(view_angle[0], view_angle[1])

        if not detached:
            plt.show()

    def lookup(self, name):
        """
        Uses the hash table to lookup a certain transform based on name
        """
        if len(self.coll_dict.keys()) == 0:
            print("No transforms have been named in the RigidCollection!")

        else:
            try:
                return self.coll_dict[name]
            # raise a key error
            except KeyError:
                raise KeyError("{0} is not in the RigidCollection!".format(name))

    def __assemble_collection_dict(self):
        """
        Assembles a hash table of transforms that uses the transform's name as the key for quick searches
        """
        self.coll_dict = {}
        # traverse the list and create a dictionary
        for t in self.collection:
            name = t.name
            if name is not None:
                self.coll_dict[t.name] = t

    def status(self):
        print("_" * 10)

        if self.name is not None:
            print("Rigid Collection: ", self.name)

        print("Rigid Collection contains {0} transforms".format(len(self.collection)))
        print("Transforms in this collection: \n", self.collection)

        print("_" * 10)
