import numpy as np
import math
import cv2
import matplotlib.pyplot as plt

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


def row_end_detection(point_cloud):
    
    # Extract the x and y coordinates from the point cloud
    x = point_cloud[:, 0]
    y = point_cloud[:, 1]
    end_flag=0
    # Creating a blank image to represent the point cloud
    img = np.zeros((500, 500), dtype=np.uint8)

    # Map the point cloud onto the image
    scaled_x = ((x - np.min(x)) / (np.max(x) - np.min(x)) * (img.shape[1] - 1)).astype(int)
    scaled_y = ((y - np.min(y)) / (np.max(y) - np.min(y)) * (img.shape[0] - 1)).astype(int)
    img[scaled_y, scaled_x] = 255

    # Cropping to avoid horizontal lines
    img= img[:400,:500]

    # Canny edge detection to detect the edges
    edges = cv2.Canny(img, 50, 150)

    # ROI for left side and right sides
    height, width = edges.shape
    roi_left = edges[:, :width // 2]
    roi_right = edges[:, width // 2:]

    # Applying Hough line transformation for the left side
    lines_left = cv2.HoughLines(roi_left, 1, np.pi / 180, threshold=100)

    # Applying Hough line transformation for the right side
    lines_right = cv2.HoughLines(roi_right, 1, np.pi / 180, threshold=100)


    #checking if the there are any lines detected on the left and ride side of the row
    if lines_left is None and lines_right is  None:
        end_flag=1

    return end_flag    
    
       
        

if __name__ == "__main__":
    pointcloud = read_pointcloud("4.npz")
    #sample_data = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    print(f"End of the Row Detected : {row_end_detection(read_pointcloud)}")
    




