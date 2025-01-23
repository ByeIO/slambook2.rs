import cv2
import numpy as np
import time

file_1 = "../assets/ch8-LK1.png"  # first image
file_2 = "../assets/ch8-LK2.png"  # second image

class OpticalFlowTracker:
    def __init__(self, img1, img2, kp1, kp2, success, inverse=True, has_initial=False):
        self.img1 = img1
        self.img2 = img2
        self.kp1 = kp1
        self.kp2 = kp2
        self.success = success
        self.inverse = inverse
        self.has_initial = has_initial

    def calculate_optical_flow(self, range_start, range_end):
        half_patch_size = 4
        iterations = 10
        for i in range(range_start, range_end):
            kp = self.kp1[i]
            dx, dy = 0, 0
            if self.has_initial:
                dx = self.kp2[i][0] - kp[0]
                dy = self.kp2[i][1] - kp[1]

            cost, last_cost = 0, 0
            succ = True

            for iter in range(iterations):
                H = np.zeros((2, 2))
                b = np.zeros(2)

                for x in range(-half_patch_size, half_patch_size):
                    for y in range(-half_patch_size, half_patch_size):
                        error = self.get_pixel_value(self.img1, kp[0] + x, kp[1] + y) - \
                                self.get_pixel_value(self.img2, kp[0] + x + dx, kp[1] + y + dy)
                        if not self.inverse:
                            J = -np.array([
                                0.5 * (self.get_pixel_value(self.img2, kp[0] + dx + x + 1, kp[1] + dy + y) -
                                       self.get_pixel_value(self.img2, kp[0] + dx + x - 1, kp[1] + dy + y)),
                                0.5 * (self.get_pixel_value(self.img2, kp[0] + dx + x, kp[1] + dy + y + 1) -
                                       self.get_pixel_value(self.img2, kp[0] + dx + x, kp[1] + dy + y - 1))
                            ])
                        elif iter == 0:
                            J = -np.array([
                                0.5 * (self.get_pixel_value(self.img1, kp[0] + x + 1, kp[1] + y) -
                                       self.get_pixel_value(self.img1, kp[0] + x - 1, kp[1] + y)),
                                0.5 * (self.get_pixel_value(self.img1, kp[0] + x, kp[1] + y + 1) -
                                       self.get_pixel_value(self.img1, kp[0] + x, kp[1] + y - 1))
                            ])
                        b += -error * J
                        cost += error * error
                        if not self.inverse or iter == 0:
                            H += np.outer(J, J)

                # 加入正则化项, 防止奇异矩阵
                H += np.eye(2) * 1e-6

                update = np.linalg.solve(H, b)
                if np.isnan(update[0]):
                    print("update is nan")
                    succ = False
                    break

                if iter > 0 and cost > last_cost:
                    break

                dx += update[0]
                dy += update[1]
                last_cost = cost
                succ = True

                if np.linalg.norm(update) < 1e-2:
                    break

            self.success[i] = succ
            self.kp2[i] = (kp[0] + dx, kp[1] + dy)

    @staticmethod
    def get_pixel_value(img, x, y):
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if x >= img.shape[1] - 1:
            x = img.shape[1] - 2
        if y >= img.shape[0] - 1:
            y = img.shape[0] - 2

        xx = x - np.floor(x)
        yy = y - np.floor(y)
        x_a1 = min(img.shape[1] - 1, int(x) + 1)
        y_a1 = min(img.shape[0] - 1, int(y) + 1)

        return (1 - xx) * (1 - yy) * img[int(y), int(x)] + \
               xx * (1 - yy) * img[int(y), x_a1] + \
               (1 - xx) * yy * img[y_a1, int(x)] + \
               xx * yy * img[y_a1, x_a1]

def optical_flow_single_level(img1, img2, kp1, kp2, success, inverse=False, has_initial_guess=False):
    kp2.clear()
    success.clear()
    kp2.extend([(0, 0)] * len(kp1))
    success.extend([False] * len(kp1))
    tracker = OpticalFlowTracker(img1, img2, kp1, kp2, success, inverse, has_initial_guess)
    print("Number of keypoints tracked:", len(kp1))
    tracker.calculate_optical_flow(0, len(kp1))

def optical_flow_multi_level(img1, img2, kp1, kp2, success, inverse=False):
    pyramids = 4
    pyramid_scale = 0.5
    scales = [1.0, 0.5, 0.25, 0.125]

    pyr1 = [img1]
    pyr2 = [img2]
    for i in range(1, pyramids):
        pyr1.append(cv2.resize(pyr1[i - 1], (0, 0), fx=pyramid_scale, fy=pyramid_scale))
        pyr2.append(cv2.resize(pyr2[i - 1], (0, 0), fx=pyramid_scale, fy=pyramid_scale))

    kp1_pyr = [(kp[0] * scales[-1], kp[1] * scales[-1]) for kp in kp1]
    kp2_pyr = kp1_pyr.copy()

    for level in range(pyramids - 1, -1, -1):
        success.clear()
        optical_flow_single_level(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, True)

        if level > 0:
            kp1_pyr = [(kp[0] / pyramid_scale, kp[1] / pyramid_scale) for kp in kp1_pyr]
            kp2_pyr = [(kp[0] / pyramid_scale, kp[1] / pyramid_scale) for kp in kp2_pyr]

    kp2.clear()
    kp2.extend(kp2_pyr)

def main():
    img1 = cv2.imread(file_1, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(file_2, cv2.IMREAD_GRAYSCALE)

    detector = cv2.GFTTDetector_create(500, 0.01, 20)
    kp1 = detector.detect(img1)
    kp1 = [kp.pt for kp in kp1]

    kp2_single = []
    success_single = []
    optical_flow_single_level(img1, img2, kp1, kp2_single, success_single)

    kp2_multi = []
    success_multi = []
    start_time = time.time()
    optical_flow_multi_level(img1, img2, kp1, kp2_multi, success_multi, True)
    end_time = time.time()
    print("optical flow by gauss-newton:", end_time - start_time)

    pt1 = np.array([kp for kp in kp1], dtype=np.float32)
    pt2, status, error = cv2.calcOpticalFlowPyrLK(img1, img2, pt1, None)
    print("optical flow by opencv:", time.time() - end_time)

    img2_single = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    for i in range(len(kp2_single)):
        if success_single[i]:
            cv2.circle(img2_single, (int(kp2_single[i][0]), int(kp2_single[i][1])), 2, (0, 250, 0), 2)
            cv2.line(img2_single, (int(kp1[i][0]), int(kp1[i][1])), (int(kp2_single[i][0]), int(kp2_single[i][1])), (0, 250, 0))

    img2_multi = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    for i in range(len(kp2_multi)):
        if success_multi[i]:
            cv2.circle(img2_multi, (int(kp2_multi[i][0]), int(kp2_multi[i][1])), 2, (0, 250, 0), 2)
            cv2.line(img2_multi, (int(kp1[i][0]), int(kp1[i][1])), (int(kp2_multi[i][0]), int(kp2_multi[i][1])), (0, 250, 0))

    img2_CV = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    for i in range(len(pt2)):
        if status[i]:
            cv2.circle(img2_CV, (int(pt2[i][0]), int(pt2[i][1])), 2, (0, 250, 0), 2)
            cv2.line(img2_CV, (int(pt1[i][0]), int(pt1[i][1])), (int(pt2[i][0]), int(pt2[i][1])), (0, 250, 0))

    # 保存单层光流跟踪结果
    cv2.imwrite("ch8-optical_flow-tracked_single_level.png", img2_single)

    # 保存多层光流跟踪结果
    cv2.imwrite("ch8-optical_flow-tracked_multi_level.png", img2_multi)

    # 保存 OpenCV 光流跟踪结果
    cv2.imwrite("ch8-optical_flow-tracked_by_opencv.png", img2_CV)

if __name__ == "__main__":
    main()