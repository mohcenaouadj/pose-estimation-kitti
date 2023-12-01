# pose-estimation-kitti
3D reconstruction and Visual Odometry 


This project consists of 3 main parts : 
1. **Transformation and projection of a cloud of Lidar points on a camera (3D to 2D)** : In this part you will use the transformations provided by the dataset to project a
3D point cloud (Lidar/Velodyne source) onto the image plane of the first color camera (left). This allows you to know the depth of a set of 2d points in the image, but also 
to know the color of the 3D points. 
![3D-to-2D](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/Figure_1.png)

2. **Create a 3D mesh from Lidar data points and camera images** : Merge all 50 LIDAR acquisitions, after performing appropriate transformations, into a single point cloud, use the corresponding image data to color the point cloud. Save the results to a standard 3D file.
![lidar-to-3d](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/street_2.png)

3. **Visual odometry : Estimate the pose of the vihecule** : Our objective is to calculate the trajectory traveled by the vehicle using a visual odometry approach. We will use the stereo image sequence provided by Kitti to calculate odometry. Next, we will compare the calculated trajectory with the data provided by the IMU.
![odometry](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/Figure_3.png) 
