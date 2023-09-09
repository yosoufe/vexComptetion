import open3d as o3d
from cortano import RemoteInterface
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

prev_rgb3_image = None
odo_init = np.identity(4)
option = o3d.pipelines.odometry.OdometryOption()
option.depth_min = 0.05
option.depth_max = 4.0
print(option)

fx = 458.495727539062
fy = 458.299774169922
cx = 327.941131591797
cy = 182.700592041016

total_transform = np.identity(4)
total_trans = np.zeros_like(total_transform[:3, 3])

pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)

if __name__ == "__main__":
    robot = RemoteInterface(Config.ip)

    while True:
        robot.update()
        color, depth, sensors = robot.read()

        depth = depth.astype(np.float32) / 1000

        o3d_color = o3d.geometry.Image(color)
        o3d_depth = o3d.geometry.Image(depth)

        o3d_rgbImage = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, o3d_depth, 
            depth_scale = 1, depth_trunc = option.depth_max,
            convert_rgb_to_intensity = False
        )

        if prev_rgb3_image is None:
            prev_rgb3_image = o3d_rgbImage
        else:

            [success, transformation,
                info] = o3d.pipelines.odometry.compute_rgbd_odometry(
                    o3d_rgbImage, prev_rgb3_image , pinhole_camera_intrinsic, odo_init,
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)

            prev_rgb3_image = o3d_rgbImage
            total_transform = total_transform @ transformation
            total_trans = total_trans + transformation[:3, 3]

            print(total_transform[:3, 3], np.linalg.norm(total_transform[:3, 3]))
            print()

