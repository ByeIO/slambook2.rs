#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]

//! 针孔相机模型

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};

use nalgebra::{Vector3 as Vec3, Vector2 as Vec2, Matrix3 as Mat33}; // 引入 nalgebra 中的类型

// 定义相机结构体
pub struct Camera {
    // 相机内参
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub baseline: f64,
    // 外参，从立体相机到单相机的变换
    pub pose: SE3,
    // 外参的逆
    pub pose_inv: SE3,
}

impl Camera {
    // 默认构造函数
    pub fn new() -> Self {
        Camera {
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
            baseline: 0.0,
            pose: SE3::identity(),  // 假设 SE3 有 identity 方法
            pose_inv: SE3::identity(),
        }
    }

    // 带参数的构造函数
    pub fn with_params(fx: f64, fy: f64, cx: f64, cy: f64, baseline: f64, pose: SE3) -> Self {
        let pose_inv = pose.inverse();
        Camera {
            fx,
            fy,
            cx,
            cy,
            baseline,
            pose,
            pose_inv,
        }
    }

    // 获取相机的位姿
    pub fn pose(&self) -> SE3 {
        self.pose.clone()
    }

    // 返回相机内参矩阵
    pub fn K(&self) -> Mat33<f64> {
        let mut k = Mat33::zeros();
        k[(0, 0)] = self.fx;
        k[(0, 2)] = self.cx;
        k[(1, 1)] = self.fy;
        k[(1, 2)] = self.cy;
        k[(2, 2)] = 1.0;
        k
    }

    // 世界坐标系到相机坐标系的变换
    pub fn world2camera(&self, p_w: &Vec3<f64>, T_c_w: &SE3) -> Vec3<f64> {
        let T_total = self.pose.compose(T_c_w);
        T_total.apply(p_w.as_view())
    }

    // 相机坐标系到世界坐标系的变换
    pub fn camera2world(&self, p_c: &Vec3<f64>, T_c_w: &SE3) -> Vec3<f64> {
        let T_total = T_c_w.inverse().compose(&self.pose_inv);
        T_total.apply(p_c.as_view())
    }

    // 相机坐标系到像素坐标系的变换
    pub fn camera2pixel(&self, p_c: &Vec3<f64>) -> Vec2<f64> {
        Vec2::new(
            self.fx * p_c[0] / p_c[2] + self.cx,
            self.fy * p_c[1] / p_c[2] + self.cy,
        )
    }

    // 像素坐标系到相机坐标系的变换
    pub fn pixel2camera(&self, p_p: &Vec2<f64>, depth: f64) -> Vec3<f64> {
        Vec3::new(
            (p_p[0] - self.cx) * depth / self.fx,
            (p_p[1] - self.cy) * depth / self.fy,
            depth,
        )
    }

    // 世界坐标系到像素坐标系的变换
    pub fn world2pixel(&self, p_w: &Vec3<f64>, T_c_w: &SE3) -> Vec2<f64> {
        let p_c = self.world2camera(p_w, T_c_w);
        self.camera2pixel(&p_c)
    }

    // 像素坐标系到世界坐标系的变换
    pub fn pixel2world(&self, p_p: &Vec2<f64>, T_c_w: &SE3, depth: f64) -> Vec3<f64> {
        let p_c = self.pixel2camera(p_p, depth);
        self.camera2world(&p_c, T_c_w)
    }
}

#[cfg(test)]
mod tests1 {
    use super::*;
    use nalgebra::{Vector3, Vector2};

    // 比较两个 SE3 实例的 log 映射结果是否相等
    fn se3_approx_eq(a: &SE3, b: &SE3) -> bool {
        let log_a = a.log();
        let log_b = b.log();
        let epsilon = 1e-6;
        log_a.iter().zip(log_b.iter()).all(|(x, y)| (x - y).abs() < epsilon)
    }

    // 测试默认构造函数
    #[test]
    fn test_camera_new() {
        // 创建一个新的相机实例
        let camera = Camera::new();
        // 检查相机的内参是否初始化为 0.0
        assert_eq!(camera.fx, 0.0);
        assert_eq!(camera.fy, 0.0);
        assert_eq!(camera.cx, 0.0);
        assert_eq!(camera.cy, 0.0);
        assert_eq!(camera.baseline, 0.0);
        // 检查相机的位姿是否为单位位姿
        assert!(se3_approx_eq(&camera.pose, &SE3::identity()));
        assert!(se3_approx_eq(&camera.pose_inv, &SE3::identity()));
    }

    // 测试带参数的构造函数
    #[test]
    fn test_camera_with_params() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose.clone());
        // 检查相机的内参是否正确设置
        assert_eq!(camera.fx, fx);
        assert_eq!(camera.fy, fy);
        assert_eq!(camera.cx, cx);
        assert_eq!(camera.cy, cy);
        assert_eq!(camera.baseline, baseline);
        // 检查相机的位姿是否正确设置
        assert!(se3_approx_eq(&camera.pose, &pose));
        // 检查相机位姿的逆是否正确计算
        assert!(se3_approx_eq(&camera.pose_inv, &pose.inverse()));
    }

    // 测试相机内参矩阵的计算
    #[test]
    fn test_camera_K() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 计算相机的内参矩阵
        let k = camera.K();
        // 检查内参矩阵的元素是否正确
        assert_eq!(k[(0, 0)], fx);
        assert_eq!(k[(0, 2)], cx);
        assert_eq!(k[(1, 1)], fy);
        assert_eq!(k[(1, 2)], cy);
        assert_eq!(k[(2, 2)], 1.0);
    }

    // 测试世界坐标系到相机坐标系的变换
    #[test]
    fn test_world2camera() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 定义一个世界坐标系下的点
        let p_w = Vector3::new(1.0, 2.0, 3.0);
        // 定义一个 SE3 变换
        let T_c_w = SE3::identity();
        // 进行世界坐标系到相机坐标系的变换
        let p_c = camera.world2camera(&p_w, &T_c_w);
        // 由于相机的位姿是单位位姿，变换后的点应该和原点点相同
        assert_eq!(p_c, p_w);
    }

    // 测试相机坐标系到世界坐标系的变换
    #[test]
    fn test_camera2world() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 定义一个相机坐标系下的点
        let p_c = Vector3::new(1.0, 2.0, 3.0);
        // 定义一个 SE3 变换
        let T_c_w = SE3::identity();
        // 进行相机坐标系到世界坐标系的变换
        let p_w = camera.camera2world(&p_c, &T_c_w);
        // 由于相机的位姿是单位位姿，变换后的点应该和原点点相同
        assert_eq!(p_w, p_c);
    }

    // 测试相机坐标系到像素坐标系的变换
    #[test]
    fn test_camera2pixel() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 定义一个相机坐标系下的点
        let p_c = Vector3::new(1.0, 2.0, 3.0);
        // 进行相机坐标系到像素坐标系的变换
        let p_p = camera.camera2pixel(&p_c);
        // 手动计算像素坐标
        let u = fx * p_c[0] / p_c[2] + cx;
        let v = fy * p_c[1] / p_c[2] + cy;
        // 检查计算结果是否正确
        assert_eq!(p_p[0], u);
        assert_eq!(p_p[1], v);
    }

    // 测试像素坐标系到相机坐标系的变换
    #[test]
    fn test_pixel2camera() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 定义一个像素坐标系下的点
        let p_p = Vector2::new(320.0, 240.0);
        // 定义深度
        let depth = 3.0;
        // 进行像素坐标系到相机坐标系的变换
        let p_c = camera.pixel2camera(&p_p, depth);
        // 手动计算相机坐标
        let x = (p_p[0] - cx) * depth / fx;
        let y = (p_p[1] - cy) * depth / fy;
        let z = depth;
        // 检查计算结果是否正确
        assert_eq!(p_c[0], x);
        assert_eq!(p_c[1], y);
        assert_eq!(p_c[2], z);
    }

    // 测试世界坐标系到像素坐标系的变换
    #[test]
    fn test_world2pixel() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 定义一个世界坐标系下的点
        let p_w = Vector3::new(1.0, 2.0, 3.0);
        // 定义一个 SE3 变换
        let T_c_w = SE3::identity();
        // 进行世界坐标系到像素坐标系的变换
        let p_p = camera.world2pixel(&p_w, &T_c_w);
        // 先将世界坐标系下的点转换到相机坐标系下
        let p_c = camera.world2camera(&p_w, &T_c_w);
        // 再将相机坐标系下的点转换到像素坐标系下
        let p_p_check = camera.camera2pixel(&p_c);
        // 检查计算结果是否正确
        assert_eq!(p_p, p_p_check);
    }

    // 测试像素坐标系到世界坐标系的变换
    #[test]
    fn test_pixel2world() {
        // 创建一个 SE3 实例，这里假设 SE3 有 identity 方法
        let pose = SE3::identity();
        // 定义相机的内参
        let fx = 100.0;
        let fy = 200.0;
        let cx = 320.0;
        let cy = 240.0;
        let baseline = 0.1;
        // 使用带参数的构造函数创建相机实例
        let camera = Camera::with_params(fx, fy, cx, cy, baseline, pose);
        // 定义一个像素坐标系下的点
        let p_p = Vector2::new(320.0, 240.0);
        // 定义深度
        let depth = 3.0;
        // 定义一个 SE3 变换
        let T_c_w = SE3::identity();
        // 进行像素坐标系到世界坐标系的变换
        let p_w = camera.pixel2world(&p_p, &T_c_w, depth);
        // 先将像素坐标系下的点转换到相机坐标系下
        let p_c = camera.pixel2camera(&p_p, depth);
        // 再将相机坐标系下的点转换到世界坐标系下
        let p_w_check = camera.camera2world(&p_c, &T_c_w);
        // 检查计算结果是否正确
        assert_eq!(p_w, p_w_check);
    }
}
