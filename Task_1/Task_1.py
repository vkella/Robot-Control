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


# Calculate the angle of inclination 
def calculate_angle(lines):

    x1,y1=lines[:2]
    x2,y2=lines[2:]
    angle = math.atan2(y2 - y1, x2 - x1)
    
    # Convert radians to degrees
    angle_degrees = np.degrees(angle)
    return angle_degrees

   

# calculate the dominant line for a set of lines
def calculate_dominant_line(lines):
    if lines is not None:
        angles = []
        for line in lines:
            rho, theta = line[0]
            angles.append(theta)
        median_angle = np.median(angles)

        # Calculate the dominant line parameters
        a = np.cos(median_angle)
        b = np.sin(median_angle)
        rho = np.median([line[0] for line in lines])

        # Calculate the points in the line
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))

        return x1, y1, x2, y2

    return None

def line_detection(point_cloud):
    
    # Extract the x and y coordinates from the point cloud
    x = point_cloud[:, 0]
    y = point_cloud[:, 1]

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


    # Calculate the dominant line for the left side
    dominant_line_left = calculate_dominant_line(lines_left)

    # Calculate the dominant line for the right side
    dominant_line_right = calculate_dominant_line(lines_right)

    #Deciding the dominant line based on the side with the larger number of lines
    if lines_left is not None and lines_right is not None:
        if len(lines_left) > len(lines_right):
            dominant_line = dominant_line_left

        else:
            
            dominant_line = dominant_line_right
    else:
        dominant_line = dominant_line_left if lines_left is not None else dominant_line_right

    return dominant_line    
    
       
        

if __name__ == "__main__":
    pointcloud = read_pointcloud("4.npz")
    lines = line_detection(pointcloud)
    angle=calculate_angle(lines)

    if angle >0 :
        correction = -90+angle
        print(f"Robot needs to rotate to left by : {correction} degrees")
    else:

        correction= 90+angle
        print(f"Robot needs to rotate to right by : {correction} degrees")
    




