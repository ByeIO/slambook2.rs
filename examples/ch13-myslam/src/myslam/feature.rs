#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]

//! 特征提取

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::sync::{Weak, Arc};

// ORB特征提取
use bye_orb_rs::{orb, fast, common::Matchable};

// akaze特征提取
use cv::feature::akaze::{Akaze, KeyPoint};

// nalgebra库增加导入
use nalgebra::Point2;

// 使用内置库
use super::frame::Frame;
use super::mappoint::MapPoint;

/// 2D 特征点
/// 在三角化之后会被关联一个地图点
#[derive(Debug)]
pub struct Feature {
    /// 持有该feature的frame
    pub frame_: RefCell<Weak<Frame>>,
    /// 2D提取位置
    pub position_: KeyPoint,
    /// 关联地图点
    pub map_point_: RefCell<Weak<MapPoint>>,
    /// 是否为异常点
    pub is_outlier_: bool,
    /// 标识是否提在左图，false为右图
    pub is_on_left_image_: bool,
}

impl Feature {
    /// 默认构造函数
    pub fn new() -> Self {
        Feature {
            frame_: RefCell::new(Weak::new()),
            position_: KeyPoint {
                point: (0.0, 0.0),
                response: 0.0,
                size: 0.0,
                octave: 0,
                class_id: 0,
                angle: 0.0,
            },
            map_point_: RefCell::new(Weak::new()),
            is_outlier_: false,
            is_on_left_image_: true,
        }
    }

    /// 带参数的构造函数
    pub fn with_frame_and_kp(frame: Arc<Frame>, kp: KeyPoint) -> Self {
        Feature {
            frame_: RefCell::new(Arc::downgrade(&frame)),
            position_: kp,
            map_point_: RefCell::new(Weak::new()),
            is_outlier_: false,
            is_on_left_image_: true,
        }
    }

    /// 获取持有该特征的帧
    pub fn get_frame(&self) -> Option<Arc<Frame>> {
        self.frame_.borrow().upgrade()
    }

    /// 获取关联的地图点
    pub fn get_map_point(&self) -> Option<Arc<MapPoint>> {
        self.map_point_.borrow().upgrade()
    }

    /// 设置关联的地图点
    pub fn set_map_point(&self, map_point: Arc<MapPoint>) {
        *self.map_point_.borrow_mut() = Arc::downgrade(&map_point);
    }
}

// 手动实现 Feature 的 PartialEq 特征
impl PartialEq for Feature {
    fn eq(&self, other: &Self) -> bool {
        // 这里假设 Feature 的比较只基于其内存地址
        std::ptr::eq(self, other)
    }
}
