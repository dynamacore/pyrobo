import numpy as np
from robotics import Transform, RigidCollection

if __name__ == '__main__':
    # produce 3 transforms
    rToBlade = Transform(-1, -0.1, -0.3, 0, 0, -np.pi/4)
    wToBlade = Transform(4, 1.9, 0.7, 0, 0, -np.pi/4)
    world = Transform(0, 0, 0, 0, 0, 0)

    # find the world to rover transform
    wToRover = wToBlade * rToBlade.inv()
    print("World To Rover")
    print(wToRover)

    # assembled the rigid collection
    transforms = [wToBlade, wToRover, world]

    # make the plot
    collection = RigidCollection(transforms)
    collection.plot()
