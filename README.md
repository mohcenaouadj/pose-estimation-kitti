# pose-estimation-kitti
3D reconstruction and Visual Odometry 


This project consists of 3 main parts : 
1. **Transformation and projection of a cloud of Lidar points on a camera (3D to 2D)** : In this part you will use the transformations provided by the dataset to project a
3D point cloud (Lidar/Velodyne source) onto the image plane of the first color camera (left). This allows you to know the depth of a set of 2d points in the image, but also 
to know the color of the 3D points. 
![3D-to-2D](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/Figure_1.png)

2. **Create a 3D mesh from Lidar data points and camera images** : Merge all 50 LIDAR acquisitions, after performing appropriate transformations, into a single point cloud, use the corresponding image data to color the point cloud. Save the results to a standard 3D file.
|![lidar-to-3d](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/street_1.png)|![lidar-to-3d](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/street_2.png)|
|----|----|
|![lidar-to-3d](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/street_3.png)|![lidar-to-3d](https://github.com/mohcenaouadj/pose-estimation-kitti/blob/main/Images/street_4.png)
4. 
   
