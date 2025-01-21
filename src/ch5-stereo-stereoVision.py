import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import sys

# 内参
fx = 718.856
fy = 718.856
cx = 607.1928
cy = 185.2157
# 基线
b = 0.573

# 读取图像
left_file = "../assets/ch5-left.png"
right_file = "../assets/ch5-right.png"

left = cv2.imread(left_file, 0)
right = cv2.imread(right_file, 0)

# 使用StereoSGBM计算视差图
sgbm = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=96,
    blockSize=9,
    P1=8 * 9 * 9,
    P2=32 * 9 * 9,
    disp12MaxDiff=1,
    preFilterCap=63,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

disparity_sgbm = sgbm.compute(left, right)
disparity = disparity_sgbm.astype(np.float32) / 16.0

# 生成点云
pointcloud = []

for v in range(left.shape[0]):
    for u in range(left.shape[1]):
        if disparity[v, u] <= 0.0 or disparity[v, u] >= 96.0:
            continue

        # 根据双目模型计算 point 的位置
        x = (u - cx) / fx
        y = (v - cy) / fy
        depth = fx * b / disparity[v, u]

        point = np.zeros(4)
        point[0] = x * depth
        point[1] = y * depth
        point[2] = depth
        point[3] = left[v, u] / 255.0  # 颜色信息

        pointcloud.append(point)

pointcloud = np.array(pointcloud)

# 显示视差图
cv2.imshow("disparity", disparity / 96.0)
cv2.waitKey(0)
cv2.destroyAllWindows()

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

        colors = self.pointcloud[:, 3]
        self.ax.scatter(self.pointcloud[:, 0], self.pointcloud[:, 1], self.pointcloud[:, 2], c=colors, cmap='gray', s=2)
        self.canvas.draw()

def main():
    app = QApplication(sys.argv)
    viewer = PointCloudViewer(pointcloud)
    viewer.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()