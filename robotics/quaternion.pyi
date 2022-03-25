from typing import overload, Union, Tuple
from numpy import ndarray
from matplotlib.pyplot import Axes

class Quaternion:
    """
    Quaternion class that holds a quaternion [w, xi, yj, zk] and various operations for quaternions
    """

    @overload
    def __init__(self, w: float, x: float, y: float, z: float) -> None:
        """
        Construct a quaternion class from the four elements [w, xi, yj, zk]
        """
        ...
    @overload
    def __init__(self, p: ndarray) -> None:
        """
        Construct a quaternion from a numpy array vector
        """
        ...
    def w(self) -> float:
        """
        Returns the quaternion scalar-element
        """
        ...
    def x(self) -> float:
        """
        Returns the quaternion i-element
        """
        ...
    def y(self) -> float:
        """
        Returns the quaternion j-element
        """
        ...
    def z(self) -> float:
        """
        Returns the quaternion k-element
        """
        ...
    def data(self) -> ndarray:
        """
        Returns an array of the four quaternion elements
        """
        ...
    def inv(self) -> Quaternion:
        """
        Computes the inverse of the quaternion
        """
        ...
    def norm(self) -> float:
        """
        Computes the L2 norm of the quaternion
        """
        ...
    def adj(self) -> Quaternion:
        """
        Computes
        """
        ...
    def rotation(self, atol: float = 1e-12) -> bool:
        """
        Is this quaternion a rotation?
        """
        ...
    def plot(
        self,
        detached: bool = False,
        axis_obj: Axes = None,
        rgb_xyz: Tuple[str, str, str] = ["r", "g", "b"],
        xlim: Tuple[int, int] = [-2, 2],
        ylim: Tuple[int, int] = [-2, 2],
        zlim: Tuple[int, int] = [-2, 2],
        scale_factor: float = 1.0,
        view_angle: Tuple[float, float] = None,
    ) -> None:
        """
        Plot a quaternion in 3D graph. This will rotate the 3 basis axes by the quaternion in order to visualize.

        ### Args
         * `axis_obj` - `matplotlib.pyplot.Axes` axis to perform the plotting on
         * `detached` - whether or not to plot on an externally provided matplotlib axis
         * `rgb_xyz` - colors of each axis in the visualization
         * `xlim` - graph x limits
         * `ylim` - graph y limits
         * `zlim` - graph z limits
         * `scale_factor` - scale the axes up or down by this value
         * `view_angle` - tuple specifying the (elevation, azimuth) of the view angle
        """
        ...
