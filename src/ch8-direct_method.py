import cv2
import numpy as np
from scipy.linalg import inv
from scipy.sparse import lil_matrix
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from multiprocessing import Pool, cpu_count
from functools import partial

# 相机内参
fx = 718.856
fy = 718.856
cx = 607.1928
cy = 185.2157
baseline = 0.573

# 文件路径
left_file = "../assets/ch8-left.png"
disparity_file = "../assets/ch8-disparity.png"

# 加载图像
left_img = cv2.imread(left_file, 0)
disparity_img = cv2.imread(disparity_file, 0)

# 随机选择第一张图像中的像素点，并生成一些第一张图像坐标系中的3D点
nPoints = 2000
boarder = 20
pixels_ref = []
depth_ref = []

rng = np.random.RandomState(42)
for i in range(nPoints):
    x = rng.uniform(boarder, left_img.shape[1] - boarder)
    y = rng.uniform(boarder, left_img.shape[0] - boarder)
    disparity = disparity_img[int(y), int(x)]
    depth = fx * baseline / disparity
    depth_ref.append(depth)
    pixels_ref.append(np.array([x, y]))

pixels_ref = np.array(pixels_ref)
depth_ref = np.array(depth_ref)

# 定义用于姿态估计的SE3类
class SE3:
    def __init__(self, R, t):
        self.R = R
        self.t = t

    def __mul__(self, other):
        R = self.R @ other.R
        t = self.R @ other.t + self.t
        return SE3(R, t)

    def exp(update):
        # 将李代数转换为李群
        w = update[:3]
        u = update[3:]
        theta = np.linalg.norm(w)
        if theta < 1e-6:
            return SE3(np.eye(3), u)
        w_hat = np.array([[0, -w[2], w[1]],
                          [w[2], 0, -w[0]],
                          [-w[1], w[0], 0]])
        A = np.sin(theta) / theta
        B = (1 - np.cos(theta)) / theta**2
        R = np.eye(3) + A * w_hat + B * w_hat @ w_hat
        V = np.eye(3) + B * w_hat + (1 - A) / theta**2 * w_hat @ w_hat
        t = V @ u
        return SE3(R, t)

    def matrix(self):
        mat = np.eye(4)
        mat[:3, :3] = self.R
        mat[:3, 3] = self.t
        return mat

# 双线性插值
def get_pixel_value(img, x, y):
    if x < 0: x = 0
    if y < 0: y = 0
    if x >= img.shape[1]: x = img.shape[1] - 1
    if y >= img.shape[0]: y = img.shape[0] - 1
    x0, y0 = int(x), int(y)
    x1, y1 = x0 + 1, y0 + 1
    dx, dy = x - x0, y - y0
    return (1 - dx) * (1 - dy) * img[y0, x0] + dx * (1 - dy) * img[y0, x1] + (1 - dx) * dy * img[y1, x0] + dx * dy * img[y1, x1]

# 雅可比累加器
class JacobianAccumulator:
    def __init__(self, img1, img2, px_ref, depth_ref, T21):
        self.img1 = img1
        self.img2 = img2
        self.px_ref = px_ref
        self.depth_ref = depth_ref
        self.T21 = T21
        self.projection = np.zeros_like(px_ref)
        self.H = np.zeros((6, 6))
        self.b = np.zeros(6)
        self.cost = 0

    def accumulate_jacobian(self, range_tuple):
        start, end = range_tuple
        half_patch_size = 1
        cnt_good = 0
        hessian = np.zeros((6, 6))
        bias = np.zeros(6)
        cost_tmp = 0

        for i in range(start, end):
            point_ref = depth_ref[i] * np.array([(px_ref[i, 0] - cx) / fx, (px_ref[i, 1] - cy) / fy, 1])
            point_cur = self.T21.R @ point_ref + self.T21.t
            if point_cur[2] < 0:
                continue

            u = fx * point_cur[0] / point_cur[2] + cx
            v = fy * point_cur[1] / point_cur[2] + cy
            if u < half_patch_size or u > self.img2.shape[1] - half_patch_size or v < half_patch_size or v > self.img2.shape[0] - half_patch_size:
                continue

            self.projection[i] = np.array([u, v])
            X, Y, Z = point_cur
            Z2 = Z * Z
            Z_inv = 1.0 / Z
            Z2_inv = Z_inv * Z_inv
            cnt_good += 1

            for x in range(-half_patch_size, half_patch_size + 1):
                for y in range(-half_patch_size, half_patch_size + 1):
                    error = get_pixel_value(self.img1, px_ref[i, 0] + x, px_ref[i, 1] + y) - get_pixel_value(self.img2, u + x, v + y)
                    J_pixel_xi = np.zeros((2, 6))
                    J_img_pixel = np.zeros(2)

                    J_pixel_xi[0, 0] = fx * Z_inv
                    J_pixel_xi[0, 2] = -fx * X * Z2_inv
                    J_pixel_xi[0, 3] = -fx * X * Y * Z2_inv
                    J_pixel_xi[0, 4] = fx + fx * X * X * Z2_inv
                    J_pixel_xi[0, 5] = -fx * Y * Z_inv

                    J_pixel_xi[1, 1] = fy * Z_inv
                    J_pixel_xi[1, 2] = -fy * Y * Z2_inv
                    J_pixel_xi[1, 3] = -fy - fy * Y * Y * Z2_inv
                    J_pixel_xi[1, 4] = fy * X * Y * Z2_inv
                    J_pixel_xi[1, 5] = fy * X * Z_inv

                    J_img_pixel[0] = 0.5 * (get_pixel_value(self.img2, u + 1 + x, v + y) - get_pixel_value(self.img2, u - 1 + x, v + y))
                    J_img_pixel[1] = 0.5 * (get_pixel_value(self.img2, u + x, v + 1 + y) - get_pixel_value(self.img2, u + x, v - 1 + y))

                    J = -1.0 * (J_img_pixel @ J_pixel_xi)

                    hessian += J[:, None] @ J[None, :]
                    bias += -error * J
                    cost_tmp += error * error

        if cnt_good:
            self.H += hessian
            self.b += bias
            self.cost += cost_tmp / cnt_good

# 单层直接法姿态估计
def direct_pose_estimation_single_layer(img1, img2, px_ref, depth_ref, T21):
    iterations = 10
    cost = 0
    lastCost = 0
    jaco_accu = JacobianAccumulator(img1, img2, px_ref, depth_ref, T21)

    for iter in range(iterations):
        jaco_accu.H = np.zeros((6, 6))
        jaco_accu.b = np.zeros(6)
        jaco_accu.cost = 0

        pool = Pool(cpu_count())
        ranges = [(i * len(px_ref) // cpu_count(), (i + 1) * len(px_ref) // cpu_count()) for i in range(cpu_count())]
        pool.map(jaco_accu.accumulate_jacobian, ranges)
        pool.close()
        pool.join()

        H = jaco_accu.H
        b = jaco_accu.b

        update = np.linalg.solve(H, b)
        T21 = SE3.exp(update) * T21
        cost = jaco_accu.cost

        if np.isnan(update[0]):
            print("update is nan")
            break
        if iter > 0 and cost > lastCost:
            print(f"cost increased: {cost}, {lastCost}")
            break
        if np.linalg.norm(update) < 1e-3:
            break

        lastCost = cost
        print(f"iteration: {iter}, cost: {cost}")

    print(f"T21 = \n{T21.matrix()}")

    # 绘制投影的像素点
    img2_show = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    projection = jaco_accu.projection
    for i in range(len(px_ref)):
        p_ref = px_ref[i]
        p_cur = projection[i]
        if p_cur[0] > 0 and p_cur[1] > 0:
            cv2.circle(img2_show, (int(p_cur[0]), int(p_cur[1])), 2, (0, 250, 0), 2)
            cv2.line(img2_show, (int(p_ref[0]), int(p_ref[1])), (int(p_cur[0]), int(p_cur[1])), (0, 250, 0))

    cv2.imshow("current", img2_show)
    cv2.waitKey(0)

# 多层直接法姿态估计
def direct_pose_estimation_multi_layer(img1, img2, px_ref, depth_ref, T21):
    pyramids = 4
    pyramid_scale = 0.5
    scales = [1.0, 0.5, 0.25, 0.125]

    # 创建金字塔
    pyr1 = [img1]
    pyr2 = [img2]
    for i in range(1, pyramids):
        img1_pyr = cv2.resize(pyr1[i - 1], (int(pyr1[i - 1].shape[1] * pyramid_scale), int(pyr1[i - 1].shape[0] * pyramid_scale)))
        img2_pyr = cv2.resize(pyr2[i - 1], (int(pyr2[i - 1].shape[1] * pyramid_scale), int(pyr2[i - 1].shape[0] * pyramid_scale)))
        pyr1.append(img1_pyr)
        pyr2.append(img2_pyr)
    # global fx,fy,cx,cy
    # 相机内参
    fx = 718.856
    fy = 718.856
    cx = 607.1928
    cy = 185.2157
    baseline = 0.573
    fxG, fyG, cxG, cyG = fx, fy, cx, cy  # 备份旧值
    for level in range(pyramids - 1, -1, -1):
        px_ref_pyr = px_ref * scales[level]
        fx = fxG * scales[level]
        fy = fyG * scales[level]
        cx = cxG * scales[level]
        cy = cyG * scales[level]
        direct_pose_estimation_single_layer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21)

# 主函数
if __name__ == "__main__":
    T_cur_ref = SE3(np.eye(3), np.zeros(3))
    for i in range(1, 6):
        img = cv2.imread(f"../assets/ch8-{i:06d}.png", 0)
        direct_pose_estimation_multi_layer(left_img, img, pixels_ref, depth_ref, T_cur_ref)