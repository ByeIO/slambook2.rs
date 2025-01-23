import cv2
import numpy as np
import time
from typing import List, Tuple
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['Smiley Sans']  # 指定默认字体为得意黑
rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题

# 全局变量
first_file = "../assets/ch7-1.png"
second_file = "../assets/ch7-2.png"

# 描述符类型，32位无符号整数，每个描述符有8个元素，每个元素32位
DescType = List[int]

# ORB模式
ORB_pattern = [
    8, -3, 9, 5, 4, 2, 7, -12, -11, 9, -8, 2, 7, -12, 12, -13,
    2, -13, 2, 12, 1, -7, 1, 6, -2, -10, -2, -4, -13, -13, -11, -8,
    -13, -3, -12, -9, 10, 4, 11, 9, -13, -8, -8, -9, -11, 7, -9, 12,
    7, 7, 12, 6, -4, -5, -3, 0, -13, 2, -12, -3, -9, 0, -7, 5,
    12, -6, 12, -1, -3, 6, -2, 12, -6, -13, -4, -8, 11, -13, 12, -8,
    4, 7, 5, 1, 5, -3, 10, -3, 3, -7, 6, 12, -8, -7, -6, -2,
    -2, 11, -1, -10, -13, 12, -8, 10, -7, 3, -5, -3, -4, 2, -3, 7,
    -10, -12, -6, 11, 5, -12, 6, -7, 5, -6, 7, -1, 1, 0, 4, -5,
    9, 11, 11, -13, 4, 7, 4, 12, 2, -1, 4, 4, -4, -12, -2, 7,
    -8, -5, -7, -10, 4, 11, 9, 12, 0, -8, 1, -13, -13, -2, -8, 2,
    -3, -2, -2, 3, -6, 9, -4, -9, 8, 12, 10, 7, 0, 9, 1, 3,
    7, -5, 11, -10, -13, -6, -11, 0, 10, 7, 12, 1, -6, -3, -6, 12,
    10, -9, 12, -4, -13, 8, -8, -12, -13, 0, -8, -4, 3, 3, 7, 8,
    5, 7, 10, -7, -1, 7, 1, -12, 3, -10, 5, 6, 2, -4, 3, -10,
    -13, 0, -13, 5, -13, -7, -12, 12, -13, 3, -11, 8, -7, 12, -4, 7,
    6, -10, 12, 8, -9, -1, -7, -6, -2, -5, 0, 12, -12, 5, -7, 5,
    3, -10, 8, -13, -7, -7, -4, 5, -3, -2, -1, -7, 2, 9, 5, -11,
    -11, -13, -5, -13, -1, 6, 0, -1, 5, -3, 5, 2, -4, -13, -4, 12,
    -9, -6, -9, 6, -12, -10, -8, -4, 10, 2, 12, -3, 7, 12, 12, 12,
    -7, -13, -6, 5, -4, 9, -3, 4, 7, -1, 12, 2, -7, 6, -5, 1,
    -13, 11, -12, 5, -3, 7, -2, -6, 7, -8, 12, -7, -13, -7, -11, -12,
    1, -3, 12, 12, 2, -6, 3, 0, -4, 3, -2, -13, -1, -13, 1, 9,
    7, 1, 8, -6, 1, -1, 3, 12, 9, 1, 12, 6, -1, -9, -1, 3,
    -13, -13, -10, 5, 7, 7, 10, 12, 12, -5, 12, 9, 6, 3, 7, 11,
    5, -13, 6, 10, 2, -12, 2, 3, 3, 8, 4, -6, 2, 6, 12, -13,
    9, -12, 10, 3, -8, 4, -7, 9, -11, 12, -4, -6, 1, 12, 2, -8,
    6, -9, 7, -4, 2, 3, 3, -2, 6, 3, 11, 0, 3, -3, 8, -8,
    7, 8, 9, 3, -11, -5, -6, -4, -10, 11, -5, 10, -5, -8, -3, 12,
    -10, 5, -9, 0, 8, -1, 12, -6, 4, -6, 6, -11, -10, 12, -8, 7,
    4, -2, 6, 7, -2, 0, -2, 12, -5, -8, -5, 2, 7, -6, 10, 12,
    -9, -13, -8, -8, -5, -13, -5, -2, 8, -8, 9, -13, -9, -11, -9, 0,
    1, -8, 1, -2, 7, -4, 9, 1, -2, 1, -1, -4, 11, -6, 12, -11,
    -12, -9, -6, 4, 3, 7, 7, 12, 5, 5, 10, 8, 0, -4, 2, 8,
    -9, 12, -5, -13, 0, 7, 2, 12, -1, 2, 1, 7, 5, 11, 7, -9,
    3, 5, 6, -8, -13, -4, -8, 9, -5, 9, -3, -3, -4, -7, -3, -12,
    6, 5, 8, 0, -7, 6, -6, 12, -13, 6, -5, -2, 1, -10, 3, 10,
    4, 1, 8, -4, -2, -2, 2, -13, 2, -12, 12, 12, -2, -13, 0, -6,
    4, 1, 9, 3, -6, -10, -3, -5, -3, -13, -1, 1, 7, 5, 12, -11,
    4, -2, 5, -7, -13, 9, -9, -5, 7, 1, 8, 6, 7, -8, 7, 6,
    -7, -4, -7, 1, -8, 11, -7, -8, -13, 6, -12, -8, 2, 4, 3, 9,
    10, -5, 12, 3, -6, -5, -6, 7, 8, -3, 9, -8, 2, -12, 2, 8,
    -11, -2, -10, 3, -12, -13, -7, -9, -11, 0, -10, -5, 5, -3, 11, 8,
    -2, -13, -1, 12, -1, -8, 0, 9, -13, -11, -12, -5, -10, -2, -10, 11,
    -3, 9, -2, -13, 2, -3, 3, 2, -9, -13, -4, 0, -4, 6, -3, -10,
    -4, 12, -2, -7, -6, -11, -4, 9, 6, -3, 6, 11, -13, 11, -5, 5,
    11, 11, 12, 6, 7, -5, 12, -2, -1, 12, 0, 7, -4, -8, -3, -2,
    -7, 1, -6, 7, -13, -12, -8, -13, -7, -2, -6, -8, -8, 5, -6, -9,
    -5, -1, -4, 5, -13, 7, -8, 10, 1, 5, 5, -13, 1, 0, 10, -13,
    9, 12, 10, -1, 5, -8, 10, -9, -1, 11, 1, -13, -9, -3, -6, 2,
    -1, -10, 1, 12, -13, 1, -8, -10, 8, -11, 10, -6, 2, -13, 3, -6,
    7, -13, 12, -9, -10, -10, -5, -7, -10, -8, -8, -13, 4, -6, 8, 5,
    3, 12, 8, -13, -4, 2, -3, -3, 5, -13, 10, -12, 4, -13, 5, -1,
    -9, 9, -4, 3, 0, 3, 3, -9, -12, 1, -6, 1, 3, 2, 4, -8,
    -10, -10, -10, 9, 8, -13, 12, 12, -8, -12, -6, -5, 2, 2, 3, 7,
    10, 6, 11, -8, 6, 8, 8, -12, -7, 10, -6, 5, -3, -9, -3, 9,
    -1, -13, -1, 5, -3, -7, -3, 4, -8, -2, -8, 3, 4, 2, 12, 12,
    2, -5, 3, 11, 6, -9, 11, -13, 3, -1, 7, 12, 11, -1, 12, 4,
    -3, 0, -3, 6, 4, -11, 4, 12, 2, -4, 2, 1, -10, -6, -8, 1,
    -13, 7, -11, 1, -13, 12, -11, -13, 6, 0, 11, -13, 0, -1, 1, 4,
    -13, 3, -9, -2, -9, 8, -6, -3, -13, -6, -8, -2, 5, -9, 8, 10,
    2, 7, 3, -9, -1, -6, -1, -1, 9, 5, 11, -2, 11, -3, 12, -8,
    3, 0, 3, 5, -1, 4, 0, 10, 3, -6, 4, 5, -13, 0, -10, 5,
    5, 8, 12, 11, 8, 9, 9, -6, 7, -4, 8, -12, -10, 4, -10, 9,
    7, 3, 12, 4, 9, -7, 10, -2, 7, 0, 12, -2, -1, -6, 0, -11
]

def compute_orb(img: np.ndarray, keypoints: List[cv2.KeyPoint]) -> List[DescType]:
    half_patch_size = 8
    half_boundary = 16
    bad_points = 0
    descriptors = []

    for kp in keypoints:
        if (kp.pt[0] < half_boundary or kp.pt[1] < half_boundary or
                kp.pt[0] >= img.shape[1] - half_boundary or kp.pt[1] >= img.shape[0] - half_boundary):
            # 超出边界
            bad_points += 1
            descriptors.append([])
            continue

        m01, m10 = 0, 0
        for dx in range(-half_patch_size, half_patch_size):
            for dy in range(-half_patch_size, half_patch_size):
                pixel = img[int(kp.pt[1]) + dy, int(kp.pt[0]) + dx]
                m10 += dx * pixel
                m01 += dy * pixel

        # 计算角度
        m_sqrt = np.sqrt(m01 * m01 + m10 * m10) + 1e-18  # 避免除以零
        sin_theta = m01 / m_sqrt
        cos_theta = m10 / m_sqrt

        # 计算描述符
        desc = [0] * 8
        for i in range(8):
            d = 0
            for k in range(32):
                idx_pq = i * 32 + k
                p = (ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1])
                q = (ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3])

                # 旋转角度
                pp = (cos_theta * p[0] - sin_theta * p[1] + kp.pt[0],
                      sin_theta * p[0] + cos_theta * p[1] + kp.pt[1])
                qq = (cos_theta * q[0] - sin_theta * q[1] + kp.pt[0],
                      sin_theta * q[0] + cos_theta * q[1] + kp.pt[1])

                if img[int(pp[1]), int(pp[0])] < img[int(qq[1]), int(qq[0])]:
                    d |= 1 << k
            desc[i] = d
        descriptors.append(desc)

    print(f"bad/total: {bad_points}/{len(keypoints)}")
    return descriptors

def bf_match(desc1: List[DescType], desc2: List[DescType]) -> List[cv2.DMatch]:
    d_max = 40
    matches = []

    for i1 in range(len(desc1)):
        if not desc1[i1]:
            continue
        m = cv2.DMatch(_queryIdx=i1, _trainIdx=0, _distance=256)
        for i2 in range(len(desc2)):
            if not desc2[i2]:
                continue
            distance = 0
            for k in range(8):
                distance += bin(desc1[i1][k] ^ desc2[i2][k]).count('1')
            if distance < d_max and distance < m.distance:
                m.distance = distance
                m.trainIdx = i2
        if m.distance < d_max:
            matches.append(m)

    return matches

def main():
    # 读取图像
    first_image = cv2.imread(first_file, cv2.IMREAD_GRAYSCALE)
    second_image = cv2.imread(second_file, cv2.IMREAD_GRAYSCALE)
    assert first_image is not None and second_image is not None, "图片读取失败"

    # 检测FAST关键点
    start_time = time.time()
    fast = cv2.FastFeatureDetector_create(threshold=40)
    keypoints1 = fast.detect(first_image, None)
    keypoints2 = fast.detect(second_image, None)

    # 计算ORB描述符
    descriptors1 = compute_orb(first_image, keypoints1)
    descriptors2 = compute_orb(second_image, keypoints2)
    end_time = time.time()
    print(f"提取ORB耗时: {end_time - start_time} 秒")

    # 匹配描述符
    start_time = time.time()
    matches = bf_match(descriptors1, descriptors2)
    end_time = time.time()
    print(f"匹配ORB耗时: {end_time - start_time} 秒")
    print(f"匹配点数量: {len(matches)}")

    # 绘制匹配结果
    img_matches = cv2.drawMatches(first_image, keypoints1, second_image, keypoints2, matches, None)
    cv2.imshow("matches", img_matches)
    cv2.imwrite("matches.png", img_matches)
    cv2.waitKey(0)

    print("完成.")

if __name__ == "__main__":
    main()