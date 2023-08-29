import numpy as np

points = np.array(
  [
    [-198.94553972,   80.2606243,   433.52995246],
    [-325.67734512,   -7.79099682,  606.59626719],
    [245.26729325,  67.92113081, 432.67514405],
    [104.10242179,  83.60890827, 406.62416107],
    [-91.04901322,  96.95636528, 376.51435667],
    [-153.08950212,-182.97287402,  973.06603023],
    [-172.13841292,  -50.17415276,  694.17569659],
    [-161.02047569,   92.07982404,  396.95278299],
    [-16.82964262,  55.23321105, 471.71938306],
    [223.24264933,  56.49451308, 483.16323851],
    [-120.06557033, -184.54291377,  978.01182033],
    [ 462.3452559,  -107.22211983,  867.60859329],
    [  87.19297782, -345.4787612,  1332.37892095]
  ],
  dtype=float
)

num_samples = points.shape[0]
A = np.ones((num_samples, 3), dtype=float)
B = np.zeros((num_samples, 1), dtype=float)

A[:, :2] = points[:, :2]
B[:, 0] = points[:, 2]

A_t = A.T
normal = np.linalg.inv(A_t @ A) @ A_t @ B
normal = normal / np.linalg.norm(normal)
if normal[2] > 0:
  normal = -1 * normal

print(normal.squeeze())


p1 = points[0]
p2 = points[1]
p3 = points[2]

n = np.cross(p2-p1, p3 - p1)
if (n[2] > 0):
  n = -1 * n
print(n / np.linalg.norm(n))

# Best answer
# https://math.stackexchange.com/questions/3869/what-is-the-intuitive-relationship-between-svd-and-pca/3871#3871
# https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
ps = points.T
print(ps.shape)
svd = np.linalg.svd(ps - np.mean(ps, axis=1, keepdims=True))
left = svd[0]
normal = left[:, -1]
if (normal[2] >0):
  normal = -1 * normal
print(normal)


pose = np.array(
  [
    [ 0.04183374 ,-0.99828073 , 0.03691244  ,0.        ],
    [-0.40959374 ,-0.05146286 ,-0.90393521  ,0.        ],
    [ 0.91130835 , 0.02269589 ,-0.42607349  ,0.        ],
    [ 0.         , 0.         , 0.          ,1.        ],
  ]
)

ball = np.array([107.61909812,  85.28468999, 401.61486486],float)/1000

pose[:3, 3] = ball
camera_to_robot = np.identity(4, dtype=float)
camera_to_robot[:3,:3] = pose[:3, :3].T
camera_to_robot[:3, 3] = (-pose[:3, :3].T @ pose[:3, [3]]).squeeze()
print("camera to robot transformation: \n", np.linalg.inv(pose))
print("camera to robot transformation: \n", camera_to_robot)
print("robot to camera transformation: \n", pose)