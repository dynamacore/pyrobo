import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Transform:
    """
    Transform class for rigid transforms, theta, phi, psi = roll, pitch, yaw
    """

    def __str__(self):
        return str(self.transform)

    def __repr__(self):
        return "Transform(x={0}, y={1}, z={2}, roll={3}, pitch={4}, yaw={5}, parent={6}, child={7}, name={8})".format(self.x, self.y, self.z, self.theta, self.phi, self.psi, self.parent, self.child, self.name)

    def __init__(self, x=0, y=0, z=0, theta=0, phi=0, psi=0, child=None, parent=None, name=None, transform=None):
        self.child, self.parent, self.name = child, parent, name

        if transform is None:
            # create the transform
            self.transform_first = False
            self.update_transform(x, y, z, theta, phi, psi)
        else:
            self.transform_first = True
            self.transform = transform
            self.x_axis = self.transform[:3, 0]
            self.y_axis = self.transform[:3, 1]
            self.z_axis = self.transform[:3, 2]
            self.origin = self.transform[:3, 3]
            self.x, self.y, self.z, self.theta, self.phi, self.psi = self.inverse_pose()
    
    # overload multiplication
    def __mul__(self, other):
        result = self.transform @ other.transform
        result_tran = Transform(transform=result)
        # calculate pose from transform
        output = result_tran.inverse_pose()
        x, y, z, theta, phi, psi = output
        return Transform(x, y, z, theta, phi, psi)
    
    def __eq__(self, other):
        # overload equal sign to work with other transform objects and numpy arrays 
        if isinstance(other, Transform):
            return np.allclose(self.transform, other.transform) and self.name == other.name and self.parent == other.parent and self.child == other.child
        elif isinstance(other, np.ndarray):
            return np.allclose(self.transform, other)
    
    def update_transform(self, x, y, z, theta, phi, psi):
        """
        updates the transform according to the entered values
        """
        # translation vector
        self.x, self.y, self.z, self.theta, self.phi, self.psi = x, y, z, theta, phi, psi

        self.tran = np.array([
            [1, 0, 0, x], 
            [0, 1, 0, y], 
            [0, 0, 1, z], 
            [0, 0, 0, 1]
        ])

        # roll rotation about x axis
        self.rotX = np.array([
            [1  , 0              , 0              , 0]  ,
            [0  , np.cos(theta)  , -np.sin(theta) , 0]  ,
            [0  , np.sin(theta)  , np.cos(theta)  , 0]  ,
            [0  , 0              , 0              , 1]
        ])

        # pitch rotation about y axis
        self.rotY = np.array([
            [np.cos(phi)   , 0  , np.sin(phi) , 0 ]  ,
            [0             , 1  , 0           , 0 ]  ,
            [-np.sin(phi)  , 0  , np.cos(phi) , 0 ]  ,
            [0             , 0  , 0           , 1]
        ])

        # yaw rotation about z axis
        self.rotZ = np.array([
            [np.cos(psi)  , -np.sin(psi)  , 0  , 0]  ,
            [np.sin(psi)  , np.cos(psi)   , 0  , 0]  ,
            [0            , 0             , 1  , 0]  ,
            [0            , 0             , 0  , 1]
        ])

        self.rot = (self.rotZ @ self.rotY @ self.rotX)[:3, :3]
        self.transform = self.tran @ self.rotZ @ self.rotY @ self.rotX

        # recover the axes for plotting
        self.x_axis = self.transform[:3, 0]
        self.y_axis = self.transform[:3, 1]
        self.z_axis = self.transform[:3, 2]
        self.origin = self.transform[:3, 3]

    def inverse_pose(self, transform=None):
        """ 
        recovers x,y,z,r,p,y from a given transformation
        """
        if transform is None:
            transform  = self.transform
            
        assert transform.shape == (4, 4), "Wrong size transformation!"

        rot = transform[:3, :3]
        tran = transform[:3, 3]
        x, y, z = tran

        # get yaw
        try:
            psi = np.arctan2(rot[1, 0], rot[0, 0])
            cp = np.cos(psi)
            sp = np.sin(psi)
        except:
            print("Singularity in transform")
            return

        # get pitch
        phi = np.arctan2(-rot[2, 0], rot[0, 0] * cp + rot[1, 0]*sp)
        ct = np.cos(phi)
        st = np.sin(phi)

        theta = np.arctan2(st * (rot[0, 1] * cp + rot[1, 1] * sp) + rot[2, 1] * ct, -rot[0, 1] * sp + rot[1, 1] * cp)

        return x, y, z, theta, phi, psi

    def pose(self):
        """
        return pose associated with the transform
        """
        return np.array([self.x, self.y, self.z, self.theta, self.phi, self.psi]).reshape(-1, 1)

    def plot(self, detached=False, axis_obj=None, rgb_xyz=['r', 'g', 'b'], xlim=[-2, 2], ylim=[-2, 2], zlim=[-2, 2], scale_factor=1.0, view_angle=None):
        """
        Plots the transform in its parent frame
        """
        if detached:
            return self.__plot_detached(axis_obj, rgb_xyz=rgb_xyz, scale_factor=scale_factor)
        else:
            return self.__plot_attached(xlim=xlim, ylim=ylim, zlim=zlim, rgb_xyz=rgb_xyz, scale_factor=scale_factor, view_angle=view_angle)

    def __plot_attached(self, xlim, ylim, zlim, rgb_xyz, scale_factor, view_angle):
        """
        Plots the transform on internally provided matplotlib axes 
        """
        fig = plt.figure()
        axis_obj = plt.subplot(111, projection='3d')

        self.__plot_axes(axis_obj, rgb_xyz, scale_factor)

        axis_obj.set_xlim3d(xlim[0], xlim[1])
        axis_obj.set_ylim3d(ylim[0], ylim[1])
        axis_obj.set_zlim3d(zlim[0], zlim[1])
        if view_angle is not None:
            axis_obj.view_init(view_angle[0], view_angle[1])

        plt.show()

    def __plot_detached(self, axis_obj, rgb_xyz, scale_factor):
        """
        Plots the transform on externally provided matplotlib axes
        """
        self.__plot_axes(axis_obj, rgb_xyz, scale_factor)

    def __plot_axes(self, axis_obj, rgb_xyz, scale_factor):
        """ 
        Plots the axes of the transform on a mattplotlib axis
        """
        try:
            # normalize all axes
            x_axis = (scale_factor * self.x_axis ) / np.linalg.norm(self.x_axis) + self.origin
            y_axis = (scale_factor * self.y_axis ) / np.linalg.norm(self.y_axis) + self.origin
            z_axis = (scale_factor * self.z_axis ) / np.linalg.norm(self.z_axis) + self.origin

            # collect plot values
            # i unit vectors
            iX, iY, iZ  = x_axis[0], x_axis[1], x_axis[2]
            # j unit vector
            jX, jY, jZ = y_axis[0], y_axis[1], y_axis[2]
            # k unit vector
            kX, kY, kZ = z_axis[0], z_axis[1], z_axis[2]
            # origin
            oX, oY, oZ = self.origin[0], self.origin[1], self.origin[2]
                    
            axis_obj.plot([oX, iX], [oY, iY], [oZ, iZ], rgb_xyz[0])
            axis_obj.plot([oX, jX], [oY, jY], [oZ, jZ], rgb_xyz[1])
            axis_obj.plot([oX, kX], [oY, kY], [oZ, kZ], rgb_xyz[2])

        except AttributeError:
            raise AttributeError("axis_obj is None")

    
    # inverse transform
    def inv(self):
        """
        compute the inverse of the transform
        """
        rot = self.transform[:3, :3]
        tran = self.transform[:3, 3]

        # compute new translation
        new_tran = -rot.T @ tran
        new_rot = rot.T

        # assemble the inverse transform and assign it to a transform object
        inv_transform = np.eye(4)
        inv_transform[:3, :3] = new_rot
        inv_transform[:3, 3] = new_tran

        if self.name is not None:
            # initialize new transform as inverse
            return Transform(transform=inv_transform, parent=self.child, child=self.parent, name=self.name + "_inv")
        else:
            return Transform(transform=inv_transform, parent=self.child, child=self.parent, name=self.name)