<h3 align="center">
  Reconstruct 3D from stereoscopic Side by side images
</h3>

## Content
Program for reconstructing 3D points from stereoscopic images and output a 3D xyz file

## Dataset
https://drive.google.com/drive/folders/14E7YnlApD5-0Fy3CTTfXL5q0vR1yBIP2?usp=sharing

## Pipeline
- Step 1: Reading camera calibration file to get:
➢ Left Intrinsic & Right Intrinsic matrix
➢ Left Extrinsic & Right Extrinsic matrix
➢ Fundamental Matrix

- Step 2: Calculating Projection matrix P1 of Left camera and P2 of Right 
camera based on:
          𝑷 = 𝑲[𝑹|𝒕]
          𝐾: 𝐼𝑛𝑡𝑟𝑖𝑛𝑠𝑖𝑐 𝑚𝑎𝑡𝑟𝑖𝑥
          [𝑅|𝑡]: 𝐸𝑥𝑡𝑟𝑖𝑛𝑠𝑖𝑐 𝑚𝑎𝑡𝑟𝑖𝑥

- Step 3: Estimating 3D points using Direct Triangulation method:
➢ Gradually reading input image (2560x720) and separating into left and 
right image (1280x720)
➢ Picking the brightest pixel in each row by using a defined threshold to 
filter pixel values on one channel among R, G, B channels
➢ Scanning through list of filtered pixels in Left image and finding 
corresponding epipolar line in Right image based on Fundamental matrix
➢ Scanning through list of filtered pixels in Right image and getting 
corresponding point after removing possible outliers based on ax + by 
+c ≈ 0 ([x, y]: coordinate of 2D points)
➢ Using 3D estimation Direct Triangulation method and solving the 
result by SVD to get the 3D points

- Step 4: Exporting to XYZ file “3D.xyz” file with format [X Y Z]


## Demo
![image](https://user-images.githubusercontent.com/54583824/127175141-22dfdbfc-d4bd-4cbf-8912-80ef1021a5b2.png)
