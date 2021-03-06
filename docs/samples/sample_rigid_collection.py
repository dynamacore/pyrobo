import numpy as np
from robotics import Transform, RigidCollection

if __name__ == "__main__":
    # produce 3 transforms
    rToBlade = Transform(-1, -0.1, -0.3, 0, 0, -np.pi / 4)
    wToBlade = Transform(4, 1.9, 0.7, 0, 0, -np.pi / 4)
    world = Transform(0, 0, 0, 0, 0, 0)

    # find the world to rover transform
    wToRover = wToBlade * rToBlade.inv()
    print("World To Rover\n", wToRover)

    # assembled the rigid collection
    transforms = [wToBlade, wToRover, world]

    # make the plot
    collection = RigidCollection(transforms)
    collection.plot()

    # You can use the lookup function to find a particular transform
    collection.lookup("world")
    # However, to find a transform, we need to pass a name to a transform in order to use this function
    collection.add(Transform(name="world"))
    # comment out line 22
    print("Lookup 'world': \n", collection.lookup("world"))

    # if we try to look up a transform that isn't in the collection we get an error so lets add more transforms with names
    wToBlade.name = "wToBlade"
    wToRover.name = "wToRover"

    collection.add(wToBlade, wToRover)
    print("Lookup 'wToBlade': \n", collection.lookup("wToBlade"))
    print("Lookup 'wToRover': \n", collection.lookup("wToRover"))

    # notice that the collection now contains 4 transforms instead of three, why do you think that is?
    print("Number of collections in transform: ", len(collection.collection))
    # hint: try calling collection.status
    # collection.status()
