import numpy as np
import cv2
import scipy.linalg as la
from scipy.spatial.transform import Rotation as R
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import sys

# 定义相机内参
cx = 325.5
cy = 253.5
fx = 518.0
fy = 519.0
depth_scale = 1000.0

# 读取pose.txt文件中的位姿信息
def read_poses(file_path):
    poses = []
    with open(file_path, 'r') as fin:
        for line in fin:
            data = list(map(float, line.strip().split()))
            q = R.from_quat([data[6], data[3], data[4], data[5]])
            t = np.array([data[0], data[1], data[2]])
            pose = np.eye(4)
            pose[:3, :3] = q.as_matrix()
            pose[:3, 3] = t
            poses.append(pose)
    return poses

# 生成点云
def generate_point_cloud(color_imgs, depth_imgs, poses):
    pointcloud = []
    for i in range(len(color_imgs)):
        print(f"转换图像中: {i + 1}")
        color = color_imgs[i]
        depth = depth_imgs[i]
        T = poses[i]
        for v in range(color.shape[0]):
            for u in range(color.shape[1]):
                d = depth[v, u]
                if d == 0:
                    continue
                point = np.zeros(3)
                point[2] = d / depth_scale
                point[0] = (u - cx) * point[2] / fx
                point[1] = (v - cy) * point[2] / fy
                point_world = T @ np.append(point, 1)
                p = np.zeros(6)
                p[:3] = point_world[:3]
                p[5] = color[v, u, 0]  # blue
                p[4] = color[v, u, 1]  # green
                p[3] = color[v, u, 2]  # red
                pointcloud.append(p)
    return np.array(pointcloud)

# 显示点云
class PointCloudViewer(QMainWindow):
    def __init__(self, pointcloud):
        super().__init__()
        self.setWindowTitle("Point Cloud Viewer")
        self.setGeometry(100, 100, 1024, 768)
        self.pointcloud = pointcloud

        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        layout = QVBoxLayout(self.main_widget)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Point Cloud')

        self.plot_point_cloud()

    def plot_point_cloud(self):
        if self.pointcloud.size == 0:
            print("Point cloud is empty!")
            return

        colors = self.pointcloud[:, 3:] / 255.0
        self.ax.scatter(self.pointcloud[:, 0], self.pointcloud[:, 1], self.pointcloud[:, 2], c=colors, s=2)
        self.canvas.draw()

def main():
    # 读取图像和位姿
    color_imgs = []
    depth_imgs = []
    for i in range(5):
        color_img = cv2.imread(f"../assets/ch5-color/{i+1}.png")
        depth_img = cv2.imread(f"../assets/ch5-depth/{i+1}.pgm", -1)
        color_imgs.append(color_img)
        depth_imgs.append(depth_img)

    poses = read_poses("../assets/ch5-pose.txt")

    # 生成点云
    pointcloud = generate_point_cloud(color_imgs, depth_imgs, poses)
    print(f"点云共有 {len(pointcloud)} 个点.")

    # 显示点云
    app = QApplication(sys.argv)
    viewer = PointCloudViewer(pointcloud)
    viewer.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()