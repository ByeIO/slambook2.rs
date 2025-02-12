#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]

//! 帧处理

use approx::RelativeEq;

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::sync::{Arc, Mutex, Weak};

// 使用内部库
use super::feature::Feature;
use super::mappoint::MapPoint;
use super::camera::Camera;

/// 帧
/// 每一帧分配独立id，关键帧分配关键帧ID
pub struct Frame {
    /// 此帧的唯一 ID
    pub id_: u64,
    /// 关键帧的 ID
    pub keyframe_id_: u64,
    /// 是否为关键帧
    pub is_keyframe_: bool,
    /// 时间戳，暂不使用
    pub time_stamp_: f64,
    /// Tcw 形式的位姿
    pub pose_: SE3,
    /// Pose 数据锁
    pub pose_mutex_: Mutex<()>,
    /// 左图像
    pub left_img_: OMatrix<i32, Dyn, Dyn>,
    /// 右图像
    pub right_img_: OMatrix<i32, Dyn, Dyn>,
    /// 左图像中提取的特征
    pub features_left_: RefCell<Vec<Arc<Feature>>>,
    /// 右图像中对应的特征，如果没有对应则为 None
    pub features_right_: RefCell<Vec<Option<Arc<Feature>>>>,
}

impl Frame {
    /// 默认构造函数
    pub fn new() -> Self {
        Frame {
            id_: 0,
            keyframe_id_: 0,
            is_keyframe_: false,
            time_stamp_: 0.0,
            pose_: SE3::identity(),
            pose_mutex_: Mutex::new(()),
            // 使用 OMatrix::zeros 初始化矩阵
            left_img_: OMatrix::default(),
            right_img_: OMatrix::default(),
            features_left_: RefCell::new(Vec::new()),
            features_right_: RefCell::new(Vec::new()),
        }
    }

    /// 带参数的构造函数
    pub fn with_params(id: u64, time_stamp: f64, pose: SE3, left: OMatrix<i32, Dyn, Dyn>, right: OMatrix<i32, Dyn, Dyn>) -> Self {
        Frame {
            id_: id,
            keyframe_id_: 0,
            is_keyframe_: false,
            time_stamp_: time_stamp,
            pose_: pose,
            pose_mutex_: Mutex::new(()),
            left_img_: left,
            right_img_: right,
            features_left_: RefCell::new(Vec::new()),
            features_right_: RefCell::new(Vec::new()),
        }
    }

    /// 获取帧的位姿，线程安全
    pub fn Pose(&self) -> SE3 {
        let _lck = self.pose_mutex_.lock().unwrap();
        self.pose_.clone()
    }

    /// 设置帧的位姿，线程安全
    pub fn SetPose(&mut self, pose: &SE3) {
        let _lck = self.pose_mutex_.lock().unwrap();
        self.pose_ = pose.clone();
    }

    /// 设置关键帧并分配关键帧 ID
    pub fn SetKeyFrame(&mut self) {
        static mut KEYFRAME_FACTORY_ID: u64 = 0;
        self.is_keyframe_ = true;
        unsafe {
            self.keyframe_id_ = KEYFRAME_FACTORY_ID;
            KEYFRAME_FACTORY_ID += 1;
        }
    }

    /// 工厂构建模式，分配 ID
    pub fn CreateFrame() -> Arc<Frame> {
        static mut FACTORY_ID: u64 = 0;
        let new_frame = Arc::new(Frame::new());
        unsafe {
            Arc::get_mut(&mut Arc::clone(&new_frame)).unwrap().id_ = FACTORY_ID;
            FACTORY_ID += 1;
        }
        new_frame
    }
}

/// 单元测试模块
#[cfg(test)]
mod tests1 {
    use super::*;
    use factrs::linalg::{Vector3, Matrix3};

    // 辅助函数，用于比较两个 Vector3 是否近似相等
    fn vector3_approx_eq<T: Numeric>(a: &Vector3<T>, b: &Vector3<T>, abs: T, rel: T) -> bool {
        let diff = (a - b).norm();
        let tolerance = rel * a.norm().max(b.norm()) + abs;
        diff <= tolerance
    }

    // 辅助函数，用于比较两个 Matrix3 是否近似相等
    fn matrix3_approx_eq<T: Numeric>(a: &Matrix3<T>, b: &Matrix3<T>, abs: T, rel: T) -> bool {
        let mut diff_sum = T::zero();
        for i in 0..3 {
            for j in 0..3 {
                let diff = (a[(i, j)] - b[(i, j)]).abs();
                let tolerance = rel * a[(i, j)].abs().max(b[(i, j)].abs()) + abs;
                if diff > tolerance {
                    return false;
                }
                diff_sum += diff;
            }
        }
        diff_sum <= rel * a.norm().max(b.norm()) + abs
    }

    // 辅助函数，用于比较两个 SE3 是否近似相等
    fn se3_approx_eq<T: Numeric>(a: &SE3<T>, b: &SE3<T>, abs: T, rel: T) -> bool {
        let rot_eq = matrix3_approx_eq(&a.rot().to_matrix(), &b.rot().to_matrix(), abs, rel);
        let xyz_eq = vector3_approx_eq(&a.xyz().into_owned(), &b.xyz().into_owned(), abs, rel);
        rot_eq && xyz_eq
    }

    /// 测试默认构造函数
    #[test]
    fn test_new() {
        let frame = Frame::new();
        // 验证默认构造的帧的 ID 为 0
        assert_eq!(frame.id_, 0);
        // 验证默认构造的帧不是关键帧
        assert!(!frame.is_keyframe_);
        // 验证默认构造的帧的位姿为单位矩阵
        assert!(se3_approx_eq(&frame.pose_, &SE3::identity(), 1e-6, 1e-6));
    }

    /// 测试带参数的构造函数
    #[test]
    fn test_with_params() {
        let id = 1;
        let time_stamp = 1.0;
        let pose = SE3::identity();
        let left = OMatrix::default();
        let right = OMatrix::default();
        let frame = Frame::with_params(id, time_stamp, pose.clone(), left, right);
        // 验证带参数构造的帧的 ID 与传入的 ID 一致
        assert_eq!(frame.id_, id);
        // 验证带参数构造的帧的时间戳与传入的时间戳一致
        assert_eq!(frame.time_stamp_, time_stamp);
        // 验证带参数构造的帧的位姿与传入的位姿一致
        assert!(se3_approx_eq(&frame.pose_, &pose, 1e-6, 1e-6));
    }

    /// 测试获取帧的位姿
    #[test]
    fn test_pose() {
        let frame = Frame::new();
        let pose = frame.Pose();
        // 验证获取的位姿与帧的初始位姿一致
        assert!(se3_approx_eq(&pose, &SE3::identity(), 1e-6, 1e-6));
    }

    /// 测试设置帧的位姿
    #[test]
    fn test_set_pose() {
        let mut frame = Frame::new();
        let new_pose = SE3::identity();
        frame.SetPose(&new_pose);
        let pose = frame.Pose();
        // 验证设置位姿后，获取的位姿与设置的位姿一致
        assert!(se3_approx_eq(&pose, &new_pose, 1e-6, 1e-6));
    }

    /// 测试设置关键帧
    #[test]
    fn test_set_key_frame() {
        let mut frame = Frame::new();
        frame.SetKeyFrame();
        // 验证设置关键帧后，帧的 is_keyframe_ 标志为 true
        assert!(frame.is_keyframe_);
        // 验证设置关键帧后，帧的关键帧 ID 不为 0
        assert_eq!(!frame.keyframe_id_, 0);
    }

    /// 测试工厂构建模式
    #[test]
    fn test_create_frame() {
        let frame1 = Frame::CreateFrame();
        let frame2 = Frame::CreateFrame();
        // 验证通过工厂模式创建的两个帧的 ID 不同
        assert_ne!(frame1.id_, frame2.id_);
    }
}
