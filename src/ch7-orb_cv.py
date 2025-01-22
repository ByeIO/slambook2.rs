import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['Smiley Sans']  # 指定默认字体为得意黑
rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题

def main(img1_path, img2_path):
    # 读取图像
    img1 = cv2.imread(img1_path, cv2.IMREAD_COLOR)
    img2 = cv2.imread(img2_path, cv2.IMREAD_COLOR)
    assert img1 is not None and img2 is not None, "图片读取失败"

    # 初始化ORB检测器
    orb = cv2.ORB_create()

    # 第一步: 检测Oriented FAST角点位置
    start_time = time.time()
    keypoints1 = orb.detect(img1, None)
    keypoints2 = orb.detect(img2, None)

    # 第二步: 计算BRIEF描述子
    keypoints1, descriptors1 = orb.compute(img1, keypoints1)
    keypoints2, descriptors2 = orb.compute(img2, keypoints2)
    end_time = time.time()
    print(f"提取ORB特征点耗时: {end_time - start_time} 秒")

    # 绘制特征点
    outimg1 = cv2.drawKeypoints(img1, keypoints1, None, color=(0, 255, 0), flags=0)
    plt.imshow(outimg1)
    plt.title("ORB特征点")
    plt.show()

    # 第三步: 使用Hamming距离进行匹配
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    start_time = time.time()
    matches = bf.match(descriptors1, descriptors2)
    end_time = time.time()
    print(f"匹配ORB特征点耗时: {end_time - start_time} 秒")

    # 第四步: 匹配点对筛选
    matches = sorted(matches, key=lambda x: x.distance)
    min_dist = matches[0].distance
    max_dist = matches[-1].distance

    print(f"-- 最大距离: {max_dist}")
    print(f"-- 最小距离: {min_dist}")

    # 当描述子之间的距离大于两倍的最小距离时，认为匹配有误。但最小距离可能会非常小，设置一个经验值30作为下限。
    good_matches = [m for m in matches if m.distance <= max(2 * min_dist, 30.0)]

    # 第五步: 绘制匹配结果
    img_matches = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches, None, flags=2)
    img_good_matches = cv2.drawMatches(img1, keypoints1, img2, keypoints2, good_matches, None, flags=2)

    plt.imshow(img_matches)
    plt.title("所有匹配点")
    plt.show()

    plt.imshow(img_good_matches)
    plt.title("筛选后的匹配点")
    plt.show()

if __name__ == "__main__":
    import sys
    main("../assets/ch7-1.png", "../assets/ch7-2.png")