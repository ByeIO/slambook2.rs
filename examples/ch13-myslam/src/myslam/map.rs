#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_variables)]
#![allow(unused_assignments)]
#![allow(unused_mut)]

//! 地图处理

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
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};
use std::borrow::{Borrow, BorrowMut};

// 使用内部库
use super::frame::Frame;
use super::mappoint::MapPoint;
use super::feature::Feature;

/// 地图
/// 和地图的交互：前端调用 insert_keyframe 和 insert_map_point 插入新帧和地图点，后端维护地图的结构，判定 outlier/剔除等等
#[derive(Debug)]
pub struct Map {
    // 数据互斥锁
    pub data_mutex: Mutex<()>,
    // 所有的地图点
    pub landmarks: RwLock<HashMap<u64, Arc<Mutex<MapPoint>>>>,
    // 激活的地图点
    pub active_landmarks: RwLock<HashMap<u64, Arc<Mutex<MapPoint>>>>,
    // 所有的关键帧
    pub keyframes: RwLock<HashMap<u64, Arc<Frame>>>,
    // 激活的关键帧
    pub active_keyframes: RwLock<HashMap<u64, Arc<Frame>>>,
    // 当前帧
    pub current_frame: Option<Arc<Frame>>,
    // 设置：激活的关键帧数量
    pub num_active_keyframes: i32, 
}

impl Map {
    /// 默认构造函数
    pub fn new() -> Self {
        Map {
            data_mutex: Mutex::new(()),
            landmarks: RwLock::new(HashMap::new()),
            active_landmarks: RwLock::new(HashMap::new()),
            keyframes: RwLock::new(HashMap::new()),
            active_keyframes: RwLock::new(HashMap::new()),
            current_frame: None,
            num_active_keyframes: 7,
        }
    }

    /// 增加一个关键帧
    pub fn insert_keyframe(&mut self, frame: Arc<Frame>) {
        self.current_frame = Some(frame.clone());
        let keyframe_id = frame.keyframe_id_;
        let keyframes_clone = self.keyframes.read().unwrap().clone();
        {
            let mut keyframes = self.keyframes.write().unwrap();
            let mut active_keyframes = self.active_keyframes.write().unwrap();
            if!keyframes_clone.contains_key(&keyframe_id) {
                keyframes.insert(keyframe_id, frame.clone());
                active_keyframes.insert(keyframe_id, frame.clone());
            } else {
                keyframes.insert(keyframe_id, frame.clone());
                active_keyframes.insert(keyframe_id, frame);
            }
        } // 提前释放借用
    
        let active_keyframes = self.active_keyframes.read().unwrap().clone();
        if active_keyframes.len() > self.num_active_keyframes as usize {
            self.remove_old_keyframe();
        }
    }

    /// 增加一个地图顶点
    pub fn insert_map_point(&mut self, map_point: Arc<Mutex<MapPoint>>) {
        let map_point_id = map_point.lock().unwrap().id_;
        let mut landmarks = self.landmarks.write().unwrap();
        let mut active_landmarks = self.active_landmarks.write().unwrap();
        if!landmarks.contains_key(&map_point_id) {
            landmarks.insert(map_point_id, map_point.clone());
            active_landmarks.insert(map_point_id, map_point);
        } else {
            landmarks.insert(map_point_id, map_point.clone());
            active_landmarks.insert(map_point_id, map_point);
        }
    }

    /// 获取所有地图点
    pub fn get_all_map_points(&self) -> HashMap<u64, Arc<Mutex<MapPoint>>> {
        let lck = self.data_mutex.lock().unwrap();
        self.landmarks.read().unwrap().clone()
    }

    /// 获取所有关键帧
    pub fn get_all_key_frames(&self) -> HashMap<u64, Arc<Frame>> {
        let lck = self.data_mutex.lock().unwrap();
        self.keyframes.read().unwrap().clone()
    }

    /// 获取激活地图点
    pub fn get_active_map_points(&self) -> HashMap<u64, Arc<Mutex<MapPoint>>> {
        let lck = self.data_mutex.lock().unwrap();
        self.active_landmarks.read().unwrap().clone()
    }

    /// 获取激活关键帧
    pub fn get_active_key_frames(&self) -> HashMap<u64, Arc<Frame>> {
        let lck = self.data_mutex.lock().unwrap();
        self.active_keyframes.read().unwrap().clone()
    }

    /// 清理 map 中观测数量为零的点
    pub fn clean_map(&mut self) {
        let active_landmarks_clone = self.active_landmarks.read().unwrap().clone();
        let mut active_landmarks = self.active_landmarks.write().unwrap();
        let mut cnt_landmark_removed = 0;
        for (id, _) in active_landmarks_clone.iter() {
            if let Some(map_point) = active_landmarks.get(id) {
                if map_point.lock().unwrap().observed_times_ == 0 {
                    active_landmarks.remove(id);
                    cnt_landmark_removed += 1;
                }
            }
        }
        println!("Removed {} active landmarks", cnt_landmark_removed);
    }

    /// 将旧的关键帧置为不活跃状态
    fn remove_old_keyframe(&mut self) {
        if let Some(current_frame) = self.current_frame.as_ref() {
            let mut max_dis = 0.0;
            let mut min_dis = 9999.0;
            let mut max_kf_id = 0;
            let mut min_kf_id = 0;
            let twc = current_frame.pose().inverse();
            let active_keyframes_clone = self.active_keyframes.read().unwrap().clone();
            for (kf_id, kf) in active_keyframes_clone.iter() {
                if *kf_id == current_frame.id_ {
                    continue;
                }
                let dis = (kf.pose() * twc.clone()).log().norm();
                if dis > max_dis {
                    max_dis = dis;
                    max_kf_id = *kf_id;
                }
                if dis < min_dis {
                    min_dis = dis;
                    min_kf_id = *kf_id;
                }
            }
    
            const MIN_DIS_TH: f64 = 0.2; 
            let mut frame_to_remove: Option<Arc<Frame>> = None;
            let keyframes_clone = self.keyframes.read().unwrap().clone();
            if min_dis < MIN_DIS_TH {
                frame_to_remove = keyframes_clone.get(&min_kf_id).cloned();
            } else {
                frame_to_remove = keyframes_clone.get(&max_kf_id).cloned();
            }
    
            if let Some(frame) = frame_to_remove {
                println!("remove keyframe {}", frame.keyframe_id_);
                {
                    let mut active_keyframes = self.active_keyframes.write().unwrap();
                    active_keyframes.remove(&frame.keyframe_id_);
                } // 提前释放借用
    
                for feat in frame.features_left_.read().unwrap().iter()  {
                    if let Some(mp) = (&*feat).get_map_point() {
                        // 假设 feat.get_map_point 返回的是 Arc<MapPoint>，这里需要通过 landmarks 查找对应的 Arc<Mutex<MapPoint>>
                        let landmarks = self.landmarks.read().unwrap();
                        if let Some(mp_with_mutex) = landmarks.get(&mp.id_) {
                            let mut mp_guard = mp_with_mutex.lock().unwrap();
                            mp_guard.remove_observation(feat.clone());
                        }
                    }
                }
                for feat in frame.features_right_.read().unwrap().iter() {
                    if let Some(feat) = feat.as_ref() {
                        if let Some(mp) = feat.get_map_point() {
                            // 假设 feat.get_map_point 返回的是 Arc<MapPoint>，这里需要通过 landmarks 查找对应的 Arc<Mutex<MapPoint>>
                            let landmarks = self.landmarks.read().unwrap();
                            if let Some(mp_with_mutex) = landmarks.get(&mp.id_) {
                                let mut mp_guard = mp_with_mutex.lock().unwrap();
                                mp_guard.remove_observation(feat.clone());
                            }
                        }
                    }
                }
    
                self.clean_map();
            }
        }
    }
    
}

/// 单元测试
#[cfg(test)]
mod tests1 {
    // 引入需要测试的 Map 结构体所在的模块
    use super::Map;
    // 引入 Arc 类型用于共享所有权
    use std::sync::Arc;
    // 引入 Mutex 类型用于线程安全的互斥访问
    use std::sync::Mutex;
    // 引入 HashMap 类型用于存储键值对
    use std::collections::HashMap;
    // 借用
    use std::borrow::{Borrow, BorrowMut};

    // 引入内部模块中的 Frame 和 MapPoint 类型
    use super::super::frame::Frame;
    use super::super::mappoint::MapPoint;

    // 测试 Map 结构体的默认构造函数
    #[test]
    fn test_map_new() {
        let map = Map::new();
        assert!(map.data_mutex.lock().is_ok());
        assert!(map.landmarks.read().unwrap().is_empty());
        assert!(map.active_landmarks.read().unwrap().is_empty());
        assert!(map.keyframes.read().unwrap().is_empty());
        assert!(map.active_keyframes.read().unwrap().is_empty());
        assert!(map.current_frame.is_none());
        assert_eq!(map.num_active_keyframes, 7);
    }

    // 测试插入关键帧的功能
    #[test]
    fn test_insert_keyframe() {
        let mut map = Map::new();
        let frame = Arc::new(Frame::new());
        map.insert_keyframe(frame.clone());
        assert_eq!(map.current_frame.as_ref().unwrap().id_, frame.id_);
        assert!(map.keyframes.read().unwrap().contains_key(&frame.keyframe_id_));
        assert!(map.active_keyframes.read().unwrap().contains_key(&frame.keyframe_id_));
    }

    // 测试插入地图点的功能
    #[test]
    fn test_insert_map_point() {
        let mut map = Map::new();
        let map_point = Arc::new(Mutex::new(MapPoint::new()));
        map.insert_map_point(map_point.clone());
        let map_point_id = map_point.lock().unwrap().id_;
        assert!(map.landmarks.read().unwrap().contains_key(&map_point_id));
        assert!(map.active_landmarks.read().unwrap().contains_key(&map_point_id));
    }

    // 测试获取所有地图点的功能
    #[test]
    fn test_get_all_map_points() {
        let map = Map::new();
        let all_map_points = map.get_all_map_points();
        assert!(all_map_points.is_empty());
    }

    // 测试获取所有关键帧的功能
    #[test]
    fn test_get_all_key_frames() {
        let map = Map::new();
        let all_key_frames = map.get_all_key_frames();
        assert!(all_key_frames.is_empty());
    }

    // 测试获取激活地图点的功能
    #[test]
    fn test_get_active_map_points() {
        let map = Map::new();
        let active_map_points = map.get_active_map_points();
        assert!(active_map_points.is_empty());
    }

    // 测试获取激活关键帧的功能
    #[test]
    fn test_get_active_key_frames() {
        let map = Map::new();
        let active_key_frames = map.get_active_key_frames();
        assert!(active_key_frames.is_empty());
    }

    // 测试清理地图中观测数量为零的点的功能
    #[test]
    fn test_clean_map() {
        let mut map = Map::new();
        let map_point = Arc::new(Mutex::new(MapPoint::new()));
        {
            let mut mp = map_point.lock().unwrap();
            mp.observed_times_ = 0;
        }
        map.insert_map_point(map_point.clone());
        let map_point_id = map_point.lock().unwrap().id_;
        map.clean_map();
        assert!(!map.active_landmarks.read().unwrap().contains_key(&map_point_id));
    }

    // 测试将旧的关键帧置为不活跃状态的功能
    #[test]
    fn test_remove_old_keyframe() {
        let mut map = Map::new();
        let frame = Arc::new(Frame::new());
        map.insert_keyframe(frame.clone());
        map.num_active_keyframes = 1;
        let new_frame = Arc::new(Frame::new());
        map.insert_keyframe(new_frame.clone());
        assert!(!map.active_keyframes.read().unwrap().contains_key(&frame.keyframe_id_));
    }
    
}
