# imports libraries
import os, cv2, time
import numpy as np
import pandas as pd

# defines a function for preprocessing image
def process_img(img, threshold = 75, maxvalue = 255):
    # gets shape of image
    h, w = img.shape[0], img.shape[1]//2
    
    # gets left, right image
    left_img  = img[:, :w, :]
    right_img = img[:, w:, :]
    
    # gets channels in each image
    _, _, r_left    = cv2.split(left_img)
    _, _, r_right = cv2.split(right_img)
    
    red_thresh_l = cv2.threshold(r_left, threshold, maxvalue, cv2.THRESH_TOZERO)
    red_thresh_r = cv2.threshold(r_right, threshold, maxvalue, cv2.THRESH_TOZERO)
    
    sparse    = np.zeros((h, w))
    rgb_left  = np.dstack((sparse, sparse, red_thresh_l[1]))
    rgb_right = np.dstack((sparse, sparse, red_thresh_r[1]))
    
    h_L, w_L = np.mgrid[0:h, 0:w]
    mask_left = (rgb_left[:,:,0] > 25) | (rgb_left[:,:,1] > 25) | (rgb_left[:,:,2] > 25)
    final_h_L = list(h_L[mask_left]) # height
    final_w_L = list(w_L[mask_left]) # width
    
    h_R, w_R = np.mgrid[0:h, 0:w]
    mask_right = (rgb_right[:,:,0] > 25) | (rgb_right[:,:,1] > 25) | (rgb_right[:,:,2] > 25)
    final_h_R = list(h_R[mask_right]) # height
    final_w_R = list(w_R[mask_right]) # width
    
    return final_h_L, final_w_L, final_h_R, final_w_R

# sets local path
local_path = "SidebySide/"

# sets output path
output_path = "output"
if not os.path.exists(output_path):
    os.makedirs(output_path)

# defines camera parameter file
cam_params = "CalibrationData.txt"
# gets camera parameter path
cam_params_path = os.path.join(local_path, cam_params)
# gets data in camera parameter.txt
camera_data = pd.read_csv(cam_params_path, delimiter = "\t", header=None).values.tolist()

# gets left and right intrinsic, extrinsic and fundamental matrix
K_left, Rt_left, K_right, Rt_right, F_matrix = [], [], [], [], []
for i, line in enumerate(camera_data):
    # Gets left camera Intrinsic matrix
    if line[0].startswith("#Left Camera K"):
        for j in range(1, 4):
            value = camera_data[i+j][0].strip().split(" ")
            value = [float(element) for element in value]
            K_left.append(value)
    # Gets left camera Extrinsic matrix
    elif line[0].startswith("#Left Camera RT"):
        for j in range(1, 4):
            value = camera_data[i+j][0].strip().split(" ")
            value = [float(element) for element in value]
            Rt_left.append(value)
    # Gets right camera Intrinsic matrix
    elif line[0].startswith("#Right Camera K"):
        for j in range(1, 4):
            value = camera_data[i+j][0].strip().split(" ")
            value = [float(element) for element in value]
            K_right.append(value)
    # Gets right camera Extrinsic matrix
    elif line[0].startswith("#Right Camera RT"):
        for j in range(1, 4):
            value = camera_data[i+j][0].strip().split(" ")
            value = [float(element) for element in value]
            Rt_right.append(value)
    # Gets Fundamental matrix
    elif line[0].startswith("#Fundamental Matrix"):
        for j in range(1, 4):
            value = camera_data[i+j][0].strip().split(" ")
            value = [float(element) for element in value]
            F_matrix.append(value)
            
# Converts to array
K_left = np.array(K_left, dtype=float)
Rt_left = np.array(Rt_left, dtype=float)
K_right = np.array(K_right, dtype=float)
Rt_right = np.array(Rt_right, dtype=float)
F_matrix = np.array(F_matrix, dtype=float)

# Gets Projection matrix
P_left = K_left.dot(Rt_left)
P_right = K_right.dot(Rt_right)

# Defines threshold for removing outliers
epsilon = 0.001 # TODO: change to 0.01 to have better result (but long executing time)

# Sets *.xyz output file
output_xyz = f"3D_epsilon_{epsilon}.xyz"
# Sets *.xyz output path file
output_xyz_path = os.path.join(output_path, output_xyz)
# Opens file to write
f = open(output_xyz_path, "w+")

# Creates list to append all the X, Y, Z calculated points
list_3D_points = []
original_start = time.time()
for img_file in os.listdir(local_path):
    if img_file.endswith(".jpg"):
        print(f"[INFO]...Processing {img_file}")
        # Starts counting time
        start = time.time()
        # Gets image path
        img_path = os.path.join(local_path, img_file)
        # Reads image
        img = cv2.imread(img_path)
        # Gets bright pixels in left and right image
        final_h_L, final_w_L, final_h_R, final_w_R = process_img(img)
        # Searchs in left image
        for i in range(len(final_h_L)):
            x, y = final_w_L[i], final_h_L[i]
            left_point = np.array([x, y, 1])
            left_point_trans = left_point.T
            # Gets projection point
            left = F_matrix.dot(left_point)
            a, b, c = left[0], left[1], left[2]
            # Searchs in right image
            for j in range(len(final_h_R)):
                m, n = final_w_R[j], final_h_R[j]
                # removes outliers
                if abs(a*m+b*n+c) < epsilon:
                    right_point = np.array([m, n, 1])
                    # Computes to equation
                    A1 =  left_point[0] * P_left[2, :] - P_left[0, :]
                    A2 =  left_point[1] * P_left[2, :] - P_left[1, :]
                    A3 = right_point[0] * P_right[2, :] - P_right[0, :]
                    A4 = right_point[1] * P_right[2, :] - P_right[1, :]
                    A  = np.vstack((A1, A2, A3, A4))
                    # Uses SVD to solve the equation
                    U, S, V = np.linalg.svd(A)
                    V = V.T
                    P = (V[:,-1] / V[-1,-1])
                    X, Y, Z = P[:3]
                    # Filters by the height dimension of 3D model
                    if abs(Y) > 145: # (145mm is the height dimension of 3D model)
                        continue
                    elif [X, Y, Z] in list_3D_points:
                        continue
                    else:
                        f.write(f"{X} {Y} {Z}\n")
                        list_3D_points.append([X, Y, Z])
        # stops counting time
        end = time.time()
        # executes time
        print("\tProcessing time: ", round(end-start, 2))
        
f.close()
original_end = time.time()
print(f"With defined epsilon={epsilon}")
print("Total Processing time is: ", round(original_end-original_start, 2))