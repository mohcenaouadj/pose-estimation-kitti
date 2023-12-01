import matplotlib.pyplot as plt
import numpy as np
import cv2
import pykitti
import open3d
#plt.switch_backend('TkAgg')
basedir, date, drive, frame, min_dist = 'D:\IMOVI/TIA/Odometry/KITTI_SAMPLE/RAW', '2011_09_26', '0009', 0, 5
dataset = pykitti.raw(basedir, date, drive, frames=range(0, 440, 1))
img = dataset.get_cam2(0)
w, h = img.size
velo = dataset.get_velo(0)
velo[:,3] = 1
velo_ = velo[velo[:,0] > 5]
k =  np.diag([0, 0, 0, 1])
k[:3,:3] = dataset.calib.K_cam2
P = k @ dataset.calib.T_cam2_velo
velo_transforme = P @ velo_.T
velo_transforme_n = velo_transforme / velo_transforme[2,:]
mask = np.logical_and.reduce((velo_transforme_n[0,:] < w,
                              velo_transforme_n[0,:] > 0,
                              velo_transforme_n[1,:] < h,
                              velo_transforme_n[1,:] > 0))
points = np.vstack((velo_transforme_n[0,:][mask], velo_transforme_n[1,:][mask])).astype('int')

plt.figure(figsize= (16, 7))
plt.imshow(img)
plt.scatter(x = points[0,:], y = points[1,:], c = 1/ velo_transforme[2][mask])
plt.show()

# Part 2
# Lidar : 3d reconstruction 
fused_points= []
for i in range(0, 50):
    lidar = dataset.get_velo(i)
    image = np.array(dataset.get_cam2(i))
    lidar[:, 3] = 1
    lidar = lidar[lidar[:,0] > 5]
    lidar_ = (P @ lidar.T).transpose()
    lidar_2D = lidar_[:, :2] / lidar_[:,2][:, None]
    mask = np.where((lidar_2D[:,0] > 0) * (lidar_2D[:,0] < w) * (lidar_2D[:,1] > 0) * (lidar_2D[:,1] < h))

    pixels = lidar_2D[mask].astype('uint16')
    colors = image[pixels[:, 1], pixels[:, 0]]

    points = (dataset.oxts[i].T_w_imu @ np.linalg.inv(dataset.calib.T_velo_imu) @ lidar[mask].T).transpose()
    fused_points.append(np.hstack((points[:, :3] / points[:, 3][:, None], colors)))

fused_points = np.concatenate(fused_points, axis = 0)
point_cloud = open3d.geometry.PointCloud()
point_cloud.points = open3d.utility.Vector3dVector(fused_points[:,:3])
point_cloud.colors = open3d.utility.Vector3dVector(fused_points[:,3:] / 255)
open3d.visualization.draw_geometries([point_cloud])

# Part 3 : Visual Odometry
# Trajectoire
traject = np.array([i[1][:3, -1] for i in dataset.oxts])
plt.figure(figsize=(12, 12))
plt.scatter(traject[:,0], traject[:,1])
plt.axis('equal')
plt.show()

def get_scale(i, k, sift, bf):
    I0 = [np.array(dataset.get_cam2(i)), np.array(dataset.get_cam3(i))]
    I1 = [np.array(dataset.get_cam2(i+1)), np.array(dataset.get_cam3(i+1))]
    M1, M2, M3 = sift.detectAndCompute(I0[0], None), sift.detectAndCompute(I0[1], None), sift.detectAndCompute(I1[0],None)
    M21, M13, M23 = bf.match(M1[1], M2[1]), bf.match(M3[1], M1[1]), bf.match(M3[1], M2[1])
    M21, M13, M23 = sorted(M21, key=lambda x: x.distance), sorted(M13, key=lambda x: x.distance), sorted(M23, key=lambda x: x.distance)
    kp21_1, kp21_2 = np.float32([M1[0][i.queryIdx].pt for i in M21]), np.float32([M2[0][i.trainIdx].pt for i in M21])
    kp13_3, kp13_1 = np.float32([M3[0][i.queryIdx].pt for i in M13]), np.float32([M1[0][i.trainIdx].pt for i in M13])
    kp23_3, kp23_2 = np.float32([M3[0][i.queryIdx].pt for i in M23]), np.float32([M2[0][i.trainIdx].pt for i in M23])
    E13, mask13 = cv2.findEssentialMat(kp13_3, kp13_1, k, cv2.FM_8POINT + cv2.FM_RANSAC)
    E23, mask23 = cv2.findEssentialMat(kp23_3, kp23_2, k, cv2.FM_8POINT + cv2.FM_RANSAC)
    P13 = cv2.recoverPose(E13, kp13_3, kp13_1, k, mask13)
    P23 = cv2.recoverPose(E23, kp23_3, kp23_2, k, mask23)
    t21 =dataset.calib.T_cam2_imu @ np.linalg.inv(dataset.calib.T_cam3_imu)
    S = np.linalg.pinv(np.hstack([P23[2], -P13[2]])) @ t21[:-1, -1]
    return S[1] * P13[2].flatten()

k = dataset.calib.K_cam2 # Camera 2 and 3 have the same k
sift = cv2.SIFT_create()
bf = cv2.BFMatcher(crossCheck=True, normType=cv2.NORM_L2)
trajectory = np.zeros(3)
for i in range(0, 49):
    trajectory = np.vstack((trajectory, trajectory[-1] + get_scale(i, k, sift, bf)))
ref_transformation = np.zeros((4, 4))
ref_transformation[:2, :2], ref_transformation[-1, -1] = dataset.oxts[0].T_w_imu[:2, :2], 1
trajectory_ = np.hstack((-trajectory[:, 2].reshape(-1, 1), -trajectory[:, 1].reshape(-1, 1), -trajectory[:, 0].reshape(-1, 1)))
trajectory_ = np.hstack((trajectory_, np.ones((trajectory_.shape[0], 1))))
for i in range(trajectory.shape[0]):
    trajectory[i] = np.dot(ref_transformation, trajectory_[i, :])[:-1]

ground_truth = np.array([i[1][:3, -1] for i in dataset.oxts])
plt.figure(figsize=(12, 12))
plt.scatter(ground_truth[:,0], ground_truth[:, 1], c  = 'b', label = 'Ground Truth')
plt.scatter(trajectory[:,0], trajectory[:,1], c = 'r', label = 'Estimated path')
plt.legend()
plt.show()
