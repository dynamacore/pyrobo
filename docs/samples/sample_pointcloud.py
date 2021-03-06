import robotics
import numpy as np

if __name__ == "__main__":
    # define the point cloud
    cloud = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2], [5, 3, 2]])

    cloudObj = robotics.PointCloud(cloud=cloud)

    # display the cloud
    print("Cloud has points:\n", cloudObj)
    cloudObj.plot()

    # define a transform to perform on the pointcloud
    world = robotics.Transform(name="world")
    frame1 = robotics.Transform(1, 1, 1, 0, 0, 0, name="frame1")

    # display the new cloud
    new = cloudObj.transform("frame2", "frame1", frame1)
    print("Transformed cloud:\n", new)
    new.plot()
