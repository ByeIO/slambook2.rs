import cv2
import numpy as np
from typing import List, Tuple

# 找到特征匹配点
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

# 估计两张图像间运动
def pose_estimation_2d2d(keypoints1, keypoints2, matches):
    # 相机内参矩阵
    global K
    K = np.array([[520.9, 0, 325.1],
                  [0, 521.0, 249.7],
                  [0, 0, 1]])

    # 将匹配点转换为Point2f形式
    points1 = np.array([keypoints1[m.queryIdx].pt for m in matches])
    points2 = np.array([keypoints2[m.trainIdx].pt for m in matches])

    # 计算本质矩阵
    focal_length = K[0, 0]
    principal_point = (K[0, 2], K[1, 2])
    essential_matrix, _ = cv2.findEssentialMat(points1, points2, focal_length, principal_point, cv2.RANSAC, 0.999, 1.0)

    # 从本质矩阵中恢复旋转和平移信息
    _, R, t, _ = cv2.recoverPose(essential_matrix, points1, points2, K)
    return R, t

# 三角测量
def triangulation(keypoints1, keypoints2, matches, R, t):
    # 相机内参矩阵
    K = np.array([[520.9, 0, 325.1],
                  [0, 521.0, 249.7],
                  [0, 0, 1]])

    # 构造投影矩阵 P1 和 P2
    P1 = np.hstack((np.eye(3, 3), np.zeros((3, 1))))
    P2 = np.hstack((R, t))

    # 将像素坐标转换为相机归一化坐标
    pts1 = np.array([pixel2cam(keypoints1[m.queryIdx].pt, K) for m in matches])
    pts2 = np.array([pixel2cam(keypoints2[m.trainIdx].pt, K) for m in matches])

    # 三角化
    pts4d_hom = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
    pts4d = pts4d_hom / pts4d_hom[3]  # 转换为非齐次坐标

    # 提取三维点
    points3d = pts4d[:3].T
    return points3d

# 将像素坐标转换为相机归一化坐标
def pixel2cam(p, K):
    return np.array([
        (p[0] - K[0, 2]) / K[0, 0],
        (p[1] - K[1, 2]) / K[1, 1]
    ])

# 根据深度获取颜色
def get_color(depth):
    up_th = 50
    low_th = 10
    th_range = up_th - low_th
    if depth > up_th:
        depth = up_th
    if depth < low_th:
        depth = low_th
    return (255 * depth / th_range, 0, 255 * (1 - depth / th_range))

def main(img1_path, img2_path):
    # 读取图像
    img1 = cv2.imread(img1_path, cv2.IMREAD_COLOR)
    img2 = cv2.imread(img2_path, cv2.IMREAD_COLOR)
    assert img1 is not None and img2 is not None, "无法加载图像！"

    # 找到特征匹配点
    keypoints1, keypoints2, matches = find_feature_matches(img1, img2)

    # 估计两张图像间运动
    R, t = pose_estimation_2d2d(keypoints1, keypoints2, matches)

    # 三角化
    points3d = triangulation(keypoints1, keypoints2, matches, R, t)

    # 验证三角化点与特征点的重投影关系
    img1_plot = img1.copy()
    img2_plot = img2.copy()
    for i, m in enumerate(matches):
        # 第一个图
        depth1 = points3d[i, 2]
        print(f"depth: {depth1}")
        pt1_cam = pixel2cam(keypoints1[m.queryIdx].pt, K)
        cv2.circle(img1_plot, tuple(map(int, keypoints1[m.queryIdx].pt)), 2, get_color(depth1), 2)

        # 第二个图
        pt2_trans = R @ points3d[i].reshape(3, 1) + t
        depth2 = pt2_trans[2, 0]
        cv2.circle(img2_plot, tuple(map(int, keypoints2[m.trainIdx].pt)), 2, get_color(depth2), 2)

    # 保存图像
    cv2.imwrite("ch7-triangulation-img-1.png", img1_plot)
    cv2.imwrite("ch7-triangulation-img-2.png", img2_plot)

if __name__ == "__main__":
    main("../assets/ch7-1.png", "../assets/ch7-2.png")