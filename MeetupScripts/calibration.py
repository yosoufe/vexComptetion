from cortano import RemoteInterface
from pyapriltags import Detector
import cv2
import numpy as np
np.set_printoptions(precision=2)

# ROBOT_IP = "192.168.50.206" # clb wifi
# ROBOT_IP = "192.168.68.68"  # Home
ROBOT_IP = "192.168.128.66"  # Android hotspot

fx = 460.92495728   # FOV(x) -> depth2xyz -> focal length (x)
fy = 460.85058594   # FOV(y) -> depth2xyz -> focal length (y)
cx = 315.10949707   # 640 (width) 320
cy = 176.72598267   # 360 (height) 180
camera_params = (fx, fy, cx, cy)
INTERESTING_TAG_ID = 4

tag_size = 3 * 0.0254


def camera2RobotT(at_to_cam_r_calib, at_to_cam_p_calib):
    AT2Camera = np.eye(4, dtype=float)
    AT2Camera[:3, :3] = at_to_cam_r_calib
    AT2Camera[:3, 3] = at_to_cam_p_calib.squeeze()
    CalibrationAT2Robot = np.array([[0, -1, 0, 0],
                                   [-1, 0, 0, 0],
                                   [0, 0, -1, 0],
                                   [0, 0, 0, 1]], dtype=float)
    cam2RobotT = CalibrationAT2Robot @ np.linalg.inv(AT2Camera)
    return cam2RobotT


def finalCam2RobotT():
    at_to_cam_r_calib = np.array([[0.00186484, -0.06014257, -0.99818806],
                                  [0.97443102,  0.2243821, -0.01169895],
                                  [0.22467914, -0.97264359,  0.05902322]])
    at_to_cam_p_calib = np.array([[-0.16024738],
                                  [0.04109039],
                                  [0.35074568]])

    final = camera2RobotT(at_to_cam_r_calib, at_to_cam_p_calib)
    return final


if __name__ == "__main__":
    robot = RemoteInterface(ROBOT_IP)

    while True:
        robot.update()  # must never be forgotten
        color, depth, sensors = robot.read()

        detector = Detector(families='tag16h5',
                            nthreads=4,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            #  decode_sharpening=0.25,
                            #  decode_sharpening=0.9,
                            decode_sharpening=1.2,
                            debug=0)
        tags = detector.detect(
            img=cv2.cvtColor(color, cv2.COLOR_BGR2GRAY),
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size)

        # print([tag.tag_id for tag in tags])

        tags = [tag for tag in tags if tag.decision_margin > 100 and
                tag.tag_id in [INTERESTING_TAG_ID]
                and abs(tag.pose_err) < 1e-6]

        if len(tags) > 0:
            R = tags[0].pose_R
            P = tags[0].pose_t
            print("---------- For Calibration ---------")
            print(repr(R))
            print(repr(P))
            print(repr(camera2RobotT(R, P)))

            print("---------- For Object Localization ---------- ")
            cam2Robot = finalCam2RobotT()
            at2Cam = np.eye(4, dtype=float)
            at2Cam[:3, :3] = R
            at2Cam[:3, 3] = P.squeeze()
            at2Robot = cam2Robot @ at2Cam
            print("at2Robot\n", repr(at2Robot))