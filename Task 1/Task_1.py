import numpy as np
import matplotlib.pyplot as plt
###########################Assumptions ##############################
# Assuming data is in the form of (x, y, z) coordinates.
# Assuming the Robot is moving along the Y axis before deviation.
#######################################################################

# Load & Read the Point Cloud Data
def read_pointcloud(file):
    """
    file: [string]: npz file to be processed
    return: pointcloud in numpy array
    """
    row_np_array = np.load(file)
    row_pointcloud = row_np_array['arr_0']
    

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(azim=0, elev=-180)
    ax.scatter(row_pointcloud[:, 2], row_pointcloud[:, 0], row_pointcloud[:, 1])
    plt.show()

    return row_pointcloud

# Calculate the angle the point cloud has rotated from the XYZ plane
def calculate_angle(points):

    # Calculate the centroid of the point cloud
    centroid = np.mean(points, axis=0)
    
    # Calculate the angle between the original plane and the rotated plane
    theta = np.arctan2(centroid[0], centroid[2])
    # Convert radians to degrees
    theta = np.degrees(theta)
    return theta


if __name__ == "__main__":
    pointcloud = read_pointcloud("1.npz")
    print("Angle :", calculate_angle(pointcloud))
