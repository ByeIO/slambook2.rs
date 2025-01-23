import cv2
import numpy as np
import time
from typing import List, Tuple
import g2o
from sophus import SE3

def find_feature_matches(img1, img2):
    # 初始化ORB检测器和描述子提取器
    orb = cv2.ORB_create()

    # 检测关键点并计算描述子
    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

    # 使用BFMatcher进行匹配
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)

    # 筛选匹配点
    min_dist = min(matches, key=lambda x: x.distance).distance
    max_dist = max(matches, key=lambda x: x.distance).distance
    print(f"-- Max dist: {max_dist}")
    print(f"-- Min dist: {min_dist}")

    # 筛选出距离小于两倍最小距离的匹配点
    good_matches = [m for m in matches if m.distance <= max(2 * min_dist, 30.0)]
    print(f"一共找到了 {len(good_matches)} 组匹配点")

    return keypoints1, keypoints2, good_matches

def pixel2cam(p, K):
    # 将像素坐标转换为相机归一化坐标
    return np.array([
        (p[0] - K[0, 2]) / K[0, 0],
        (p[1] - K[1, 2]) / K[1, 1]
    ])

def bundle_adjustment_gauss_newton(points_3d, points_2d, K, pose):
    iterations = 10
    cost = 0
    last_cost = 0
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    for iter in range(iterations):
        H = np.zeros((6, 6))
        b = np.zeros(6)

        cost = 0
        # 计算cost
        for i in range(len(points_3d)):
            pc = pose * points_3d[i]
            inv_z = 1.0 / pc[2]
            inv_z2 = inv_z * inv_z
            proj = np.array([
                fx * pc[0] / pc[2] + cx,
                fy * pc[1] / pc[2] + cy
            ])

            e = points_2d[i] - proj
            cost += e.T @ e

            J = np.array([
                [-fx * inv_z, 0, fx * pc[0] * inv_z2, fx * pc[0] * pc[1] * inv_z2, -fx - fx * pc[0] * pc[0] * inv_z2, fx * pc[1] * inv_z],
                [0, -fy * inv_z, fy * pc[1] * inv_z2, fy + fy * pc[1] * pc[1] * inv_z2, -fy * pc[0] * pc[1] * inv_z2, -fy * pc[0] * inv_z]
            ])

            H += J.T @ J
            b += -J.T @ e

        dx = np.linalg.solve(H, b)

        if np.isnan(dx).any():
            print("result is nan!")
            break

        if iter > 0 and cost >= last_cost:
            print(f"cost: {cost}, last cost: {last_cost}")
            break

        # 更新估计
        pose = SE3.exp(dx) * pose
        last_cost = cost

        print(f"iteration {iter} cost={cost}")
        if np.linalg.norm(dx) < 1e-6:
            break

    print(f"pose by g-n: \n{pose.matrix()}")

class VertexPose(g2o.BaseVertex):
    def __init__(self):
        super().__init__()
        self._estimate = SE3()

    def set_to_origin_impl(self):
        self._estimate = SE3()

    def oplus_impl(self, update):
        update_eigen = np.array(update)
        self._estimate = SE3.exp(update_eigen) * self._estimate

class EdgeProjection(g2o.BaseUnaryEdge):
    def __init__(self, pos3d, K):
        super().__init__()
        self._pos3d = pos3d
        self._K = K

    def compute_error(self):
        v = self.vertex(0)
        T = v.estimate()
        pos_pixel = self._K @ (T @ self._pos3d)
        pos_pixel /= pos_pixel[2]
        self._error = self._measurement - pos_pixel[:2]

    def linearize_oplus(self):
        v = self.vertex(0)
        T = v.estimate()
        pos_cam = T @ self._pos3d
        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]
        X = pos_cam[0]
        Y = pos_cam[1]
        Z = pos_cam[2]
        Z2 = Z * Z
        self._jacobianOplusXi = np.array([
            [-fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z],
            [0, -fy / Z, fy * Y / Z2, fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z]
        ])

def bundle_adjustment_g2o(points_3d, points_2d, K, pose):
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverDenseSE3())
    algorithm = g2o.OptimizationAlgorithmGaussNewton(solver)
    optimizer.set_algorithm(algorithm)
    optimizer.set_verbose(True)

    vertex_pose = VertexPose()
    vertex_pose.set_id(0)
    vertex_pose.set_estimate(pose)
    optimizer.add_vertex(vertex_pose)

    K_eigen = np.array(K)

    for i in range(len(points_2d)):
        p2d = points_2d[i]
        p3d = points_3d[i]
        edge = EdgeProjection(p3d, K_eigen)
        edge.set_id(i + 1)
        edge.set_vertex(0, vertex_pose)
        edge.set_measurement(p2d)
        edge.set_information(np.eye(2))
        optimizer.add_edge(edge)

    t1 = time.time()
    optimizer.initialize_optimization()
    optimizer.optimize(10)
    t2 = time.time()
    print(f"optimization costs time: {t2 - t1} seconds.")
    print(f"pose estimated by g2o =\n{vertex_pose.estimate().matrix()}")
    pose = vertex_pose.estimate()

def main(img1_path, img2_path, depth1_path, depth2_path):
    # 读取图像
    img1 = cv2.imread(img1_path, cv2.IMREAD_COLOR)
    img2 = cv2.imread(img2_path, cv2.IMREAD_COLOR)
    assert img1 is not None and img2 is not None, "无法加载图像！"

    # 找到特征匹配点
    keypoints1, keypoints2, matches = find_feature_matches(img1, img2)

    # 建立3D点
    d1 = cv2.imread(depth1_path, cv2.IMREAD_UNCHANGED)
    K = np.array([[520.9, 0, 325.1],
                  [0, 521.0, 249.7],
                  [0, 0, 1]])
    pts_3d = []
    pts_2d = []
    for m in matches:
        d = d1[int(keypoints1[m.queryIdx].pt[1]), int(keypoints1[m.queryIdx].pt[0])]
        if d == 0:  # bad depth
            continue
        dd = d / 5000.0
        p1 = pixel2cam(keypoints1[m.queryIdx].pt, K)
        pts_3d.append(np.array([p1[0] * dd, p1[1] * dd, dd]))
        pts_2d.append(keypoints2[m.trainIdx].pt)

    print(f"3d-2d pairs: {len(pts_3d)}")

    t1 = time.time()
    _, rvec, tvec, _ = cv2.solvePnPRansac(np.array(pts_3d), np.array(pts_2d), K, None)
    R, _ = cv2.Rodrigues(rvec)
    t2 = time.time()
    print(f"solve pnp in opencv cost time: {t2 - t1} seconds.")
    print(f"R=\n{R}")
    print(f"t=\n{tvec}")

    pts_3d_eigen = [np.array([pt[0], pt[1], pt[2]]) for pt in pts_3d]
    pts_2d_eigen = [np.array([pt[0], pt[1]]) for pt in pts_2d]

    print("calling bundle adjustment by gauss newton")
    pose_gn = SE3()
    t1 = time.time()
    bundle_adjustment_gauss_newton(pts_3d_eigen, pts_2d_eigen, K, pose_gn)
    t2 = time.time()
    print(f"solve pnp by gauss newton cost time: {t2 - t1} seconds.")

    print("calling bundle adjustment by g2o")
    pose_g2o = SE3()
    t1 = time.time()
    bundle_adjustment_g2o(pts_3d_eigen, pts_2d_eigen, K, pose_g2o)
    t2 = time.time()
    print(f"solve pnp by g2o cost time: {t2 - t1} seconds.")

if __name__ == "__main__":
    main("../assets/ch7-1.png", "../assets/ch7-2.png", "../assets/ch7-1_depth.png", "../assets/ch7-2_depth.png")