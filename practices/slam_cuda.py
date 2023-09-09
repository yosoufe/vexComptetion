# not working

import open3d as o3d
import open3d.core as o3c
from cortano import RemoteInterface
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

prev_rgb3_image = None
prev_o3d_color = None
prev_o3d_depth = None

odo_init = np.identity(4)
option = o3d.pipelines.odometry.OdometryOption()
option.depth_min = 0.05
option.depth_max = 2.0
print(option)

# color
# fx = 458.495727539062
# fy = 458.299774169922
# cx = 327.941131591797
# cy = 182.700592041016

# depth
fx = 450.062347412109
fy = 450.062347412109
cx = 315.441986083984
cy = 190.89762878418

# pinhole_camera_intrinsic = np.array(
#     [[fx, 0, cx],[0,fy,cy],[0,0,1]], dtype = float
# )

total_transform = np.identity(4)
total_trans = np.zeros_like(total_transform[:3, 3])

pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)

pinhole_camera_intrinsic = o3d.core.Tensor(pinhole_camera_intrinsic.intrinsic_matrix)

criteria_list = [
    o3d.t.pipelines.odometry.OdometryConvergenceCriteria(20),
    o3d.t.pipelines.odometry.OdometryConvergenceCriteria(10),
    o3d.t.pipelines.odometry.OdometryConvergenceCriteria(5)
]
method = o3d.t.pipelines.odometry.Method.Hybrid

if __name__ == "__main__":
    robot = RemoteInterface(Config.ip)

    while True:
        robot.update()
        color, depth, sensors = robot.read()

        # depth = depth.astype(np.float32) / 1000
        # color = color.astype(np.float32)

        # o3d_color = o3d.t.geometry.Image(color).cuda(0)
        # o3d_depth = o3d.t.geometry.Image(depth).cuda(0)

        color = color.astype(np.float32)
        depth = depth.astype(np.float32) / 1000

        o3d_color = o3d.t.geometry.Image(color).cuda(0).rgb_to_gray()
        o3d_depth = o3d.t.geometry.Image(depth).cuda(0)

        # o3d_rgbImage = o3d.t.geometry.RGBDImage(
        #     o3d_color,
        #     o3d_depth
        # )

        if prev_o3d_color is None:
            prev_o3d_color = o3d_color.clone()
            prev_o3d_depth = o3d_depth.clone()
        else:
            try:
                # odom_results = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
                #         o3d_rgbImage, prev_rgb3_image,
                #         pinhole_camera_intrinsic, o3c.Tensor(np.eye(4)),
                #         depth_scale = 1, depth_max = option.depth_max,
                #         criteria_list= criteria_list, method = method)

                prev_o3d_color_dx, prev_o3d_color_dy = prev_o3d_color.filter_sobel()
                prev_o3d_depth_dx, prev_o3d_depth_dy = prev_o3d_depth.filter_sobel()

                odom_results = o3d.t.pipelines.odometry.compute_odometry_result_hybrid(
                    source_depth = o3d_depth.as_tensor(), target_depth = prev_o3d_depth.as_tensor(),
                    source_intensity = o3d_color.as_tensor(), target_intensity = prev_o3d_color.as_tensor(),
                    target_depth_dx = prev_o3d_depth_dx.as_tensor(), target_depth_dy = prev_o3d_depth_dy.as_tensor(),
                    target_intensity_dx = prev_o3d_color_dx.as_tensor(), target_intensity_dy = prev_o3d_color_dy.as_tensor(),
                    source_vertex_map = o3d_color.create_vertex_map(pinhole_camera_intrinsic).as_tensor(),
                    intrinsics = pinhole_camera_intrinsic.cuda(0),
                    init_source_to_target = o3c.Tensor(np.eye(4)).cuda(0),
                    depth_outlier_trunc = 4.0,
                    depth_huber_delta = 4.0,
                    intensity_huber_delta = 4.0
                )

                # prev_rgb3_image = o3d_rgbImage.clone()

                prev_o3d_color = o3d_color.clone()
                prev_o3d_depth = o3d_depth.clone()
                
                transformation = odom_results.transformation.cpu().numpy()
                # print(transformation.shape)
                total_transform = total_transform @ transformation

                # print(total_transform[:3, 3], np.linalg.norm(total_transform[:3, 3]))
                # print(odom_results.inlier_rmse, odom_results.fitness)
            except Exception as e:
                print(e)
                pass
            
            break
            print()

