#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_mut)]
#![allow(unused_variables)]

//! 地图点模块

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::sync::{Weak, Arc, Mutex, RwLock};
use std::borrow::{Borrow, BorrowMut};

// 使用内置库
use super::frame::Frame;
use super::feature::Feature;

/// 路标点类
/// 特征点在三角化之后形成路标点
#[derive(Debug)]
pub struct MapPoint {
    /// 地图点的唯一ID
    pub id_: u64,
    /// 是否为异常点
    pub is_outlier_: bool,
    /// 地图点在世界坐标系下的位置
    pub pos_: Vec3,
    /// 用于保护数据的互斥锁
    pub data_mutex_: Mutex<()>,
    /// 被特征匹配算法观测到的次数
    pub observed_times_: u32,
    /// 观测到该地图点的特征列表
    pub observations_: RwLock<Vec<Weak<Feature>>>,
}

impl MapPoint {
    /// 默认构造函数
    pub fn new() -> Self {
        MapPoint {
            id_: 0,
            is_outlier_: false,
            pos_: Vec3::zeros(),
            data_mutex_: Mutex::new(()),
            observed_times_: 0,
            observations_: RwLock::new(Vec::new()),
        }
    }

    /// 带参数的构造函数
    pub fn with_id_and_pos(id: u64, position: Vec3) -> Self {
        MapPoint {
            id_: id,
            is_outlier_: false,
            pos_: position,
            data_mutex_: Mutex::new(()),
            observed_times_: 0,
            observations_: RwLock::new(Vec::new()),
        }
    }

    /// 获取地图点在世界坐标系下的位置
    pub fn pos(&self) -> Vec3 {
        let _lck = self.data_mutex_.lock().unwrap();
        self.pos_
    }

    /// 设置地图点在世界坐标系下的位置
    pub fn set_pos(&mut self, pos: &Vec3) {
        let _lck = self.data_mutex_.lock().unwrap();
        self.pos_ = *pos;
    }

    /// 添加一个对该地图点的观测
    pub fn add_observation(&mut self, feature: Arc<Feature>) {
        let _lck = self.data_mutex_.lock().unwrap();
        let mut obs = self.observations_.write().unwrap();
        obs.push(Arc::downgrade(&feature));
        self.observed_times_ += 1;
    }

    /// 移除一个对该地图点的观测
    pub fn remove_observation(&mut self, feat: Arc<Feature>) {
        let _lck = self.data_mutex_.lock().unwrap();
        let mut obs = self.observations_.write().unwrap();
        if let Some(index) = obs.iter().position(|x| {
            let upgraded = x.upgrade();
            if let Some(upgraded_feat) = upgraded {
                Arc::ptr_eq(&upgraded_feat, &feat)
            } else {
                false
            }
        }) {
            obs.remove(index);
            feat.reset_map_point();
            self.observed_times_ -= 1;
        }
    }

    /// 获取观测到该地图点的特征列表
    pub fn get_obs(&self) -> Vec<Weak<Feature>> {
        let _lck = self.data_mutex_.lock().unwrap();
        let obs = self.observations_.read().unwrap();
        obs.clone()
    }

    /// 工厂函数，用于创建新的地图点
    pub fn create_new_mappoint() -> Arc<MapPoint> {
        static mut FACTORY_ID: u64 = 0;
        let mut new_mappoint = MapPoint::new();
        unsafe {
            new_mappoint.id_ = FACTORY_ID;
            FACTORY_ID += 1;
        }
        Arc::new(new_mappoint)
    }
}
