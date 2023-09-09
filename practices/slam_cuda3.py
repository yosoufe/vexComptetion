import numpy as np
import cupoch as cph
from cortano import RemoteInterface
import cv2
import numpy as np
from datetime import datetime


cph.initialize_allocator(cph.PoolAllocation, 1000000000)


# depth
fx = 450.062347412109
fy = 450.062347412109
cx = 315.441986083984
cy = 190.89762878418


prev_rgbd_image = None

if __name__ == "__main__":
    robot = RemoteInterface(Config.ip)
    # robot = RemoteInterface("192.168.55.1")

    camera_intrinsics = cph.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)
    option = cph.odometry.OdometryOption()
    option.min_depth = 0.01
    option.max_depth = 4
    # print(dir(cph.odometry.OdometryOption))
    cur_trans = np.identity(4)

    # markers = []

    while True:
        dt = datetime.now()
        robot.update()
        c, d, sensors = robot.read()
        color = c.copy()
        depth = d.copy()
        del c
        del d
        # print(color)
        # print(depth)

        # color = color.astype(np.float32)
        depth = depth.astype(np.float32)

        color = cph.geometry.Image(color)
        depth = cph.geometry.Image(depth)

        # print(cph.geometry.RGBDImage.create_from_color_and_depth.__doc__)

        # print(1)
        rgbd = cph.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale = 1000
        )
        # print(2)

        if not prev_rgbd_image is None:
            res, odo_trans, _ = cph.odometry.compute_rgbd_odometry(
                prev_rgbd_image,
                rgbd,
                camera_intrinsics,
                np.identity(4),
                cph.odometry.RGBDOdometryJacobianFromHybridTerm(),
                option,
            )
            # print(3)

            if res:
                cur_trans = np.matmul(cur_trans, odo_trans)
                print(cur_trans[:3,3])

        prev_rgbd_image = rgbd
        process_time = datetime.now() - dt
        print("FPS: " + str(1 / process_time.total_seconds()))

