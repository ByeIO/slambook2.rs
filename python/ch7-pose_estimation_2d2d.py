import cv2
import numpy as np
import time
from typing import List, Tuple
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['Smiley Sans']  # 指定默认字体为得意黑
rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题

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

def pose_estimation_2d2d(keypoints1, keypoints2, matches, K):
    # 将匹配点转换为Point2f形式
    points1 = np.array([keypoints1[m.queryIdx].pt for m in matches])
    points2 = np.array([keypoints2[m.trainIdx].pt for m in matches])

    # 计算基础矩阵
    fundamental_matrix, _ = cv2.findFundamentalMat(points1, points2, cv2.FM_8POINT)
    print("Fundamental matrix is:")
    print(fundamental_matrix)

    # 计算本质矩阵
    focal_length = K[0, 0]
    principal_point = (K[0, 2], K[1, 2])
    essential_matrix, _ = cv2.findEssentialMat(points1, points2, focal_length, principal_point, cv2.RANSAC, 0.999, 1.0)
    print("Essential matrix is:")
    print(essential_matrix)

    # 计算单应矩阵（本例中场景不是平面，单应矩阵意义不大）
    homography_matrix, _ = cv2.findHomography(points1, points2, cv2.RANSAC, 3)
    print("Homography matrix is:")
    print(homography_matrix)

    # 从本质矩阵中恢复旋转和平移信息
    _, R, t, _ = cv2.recoverPose(essential_matrix, points1, points2, K)
    print("Rotation matrix R is:")
    print(R)
    print("Translation vector t is:")
    print(t)

    # 确保 t 是一个 3x1 的列向量
    t = t.reshape(3, 1)

    return R, t

def main(img1_path, img2_path):
    # 读取图像
    img1 = cv2.imread(img1_path, cv2.IMREAD_COLOR)
    img2 = cv2.imread(img2_path, cv2.IMREAD_COLOR)
    assert img1 is not None and img2 is not None, "无法加载图像！"

    # 找到特征匹配点
    keypoints1, keypoints2, matches = find_feature_matches(img1, img2)

    # 相机内参矩阵
    K = np.array([[520.9, 0, 325.1],
                  [0, 521.0, 249.7],
                  [0, 0, 1]])

    # 估计相机运动
    R, t = pose_estimation_2d2d(keypoints1, keypoints2, matches, K)

    # 验证对极约束
    for m in matches:
        pt1 = pixel2cam(keypoints1[m.queryIdx].pt, K)
        pt2 = pixel2cam(keypoints2[m.trainIdx].pt, K)

        y1 = np.array([[pt1[0]], [pt1[1]], [1]])  # 3x1 列向量
        y2 = np.array([[pt2[0]], [pt2[1]], [1]])  # 3x1 列向量

         # 创建反对称矩阵 t_x
        t_x = np.array([[0, -t[2, 0], t[1, 0]],
                        [t[2, 0], 0, -t[0, 0]],
                        [-t[1, 0], t[0, 0], 0]])
                        
        # 计算对极约束
        d = y2.T @ t_x @ R @ y1
        print(f"Epipolar constraint = {d}")

if __name__ == "__main__":
    main("../assets/ch7-1.png", "../assets/ch7-2.png")
