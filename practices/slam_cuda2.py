import numpy as np
import cupoch as cph
from cortano import RemoteInterface
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

# depth
fx = 450.062347412109
fy = 450.062347412109
cx = 315.441986083984
cy = 190.89762878418


prev_frame = None

if __name__ == "__main__":
    robot = RemoteInterface("192.168.68.68")

    camera_intrinsics = cph.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy)
    kop = cph.kinfu.KinfuOption()
    # print(cph.kinfu.KinfuOption.__doc__)
    kop.distance_threshold = 5.0
    kinfu = cph.kinfu.KinfuPipeline(camera_intrinsics, kop)

    # markers = []

    while True:
        robot.update()
        c, d, sensors = robot.read()
        color = c.copy()
        depth = d.copy()
        # print(color)
        # print(depth)

        # color = color.astype(np.float32)
        depth = depth.astype(np.float32)

        color = cph.geometry.Image(color)
        depth = cph.geometry.Image(depth)

        # print(cph.geometry.RGBDImage.create_from_color_and_depth.__doc__)

        rgbd = cph.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale = 1000 , depth_trunc=4.0, convert_rgb_to_intensity=False
        )

        res = kinfu.process_frame(rgbd)

        if res:
            # markers.append(cph.geometry.LineSet.create_camera_marker(camera_intrinsics, np.linalg.inv(kinfu.cur_pose)))
            print(kinfu.cur_pose[:3,3])
            print()

