#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_mut)]
#![allow(unused_variables)]

//! SLAM后端

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::sync::{
    Weak, Arc, Mutex, RwLock, atomic::AtomicBool,
    Condvar, 
};
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};
use std::borrow::{Borrow, BorrowMut};
use std::thread::Thread;

// 内部库
use crate::frame::Frame;
use crate::map::Map;
use crate::algorithm::{
    triangulation, to_vec2,
};
use crate::feature::Feature;
use crate::g2o_types::{
    VertexPose, VertexXYZ, EdgeProjectionPoseOnly, 
    EdgeProjection, 
};
use crate::mappoint::MapPoint;
use crate::camera::Camera;

/// SLAM后端结构体
#[derive(Debug)]
pub struct Backend {
    // 地图的智能指针
    pub map_: Arc<RwLock<Map>>,
    // 后端线程
    pub backend_thread_: Option<Thread>,
    // 数据互斥锁
    pub data_mutex_: Mutex<()>,
    // 地图更新条件变量
    pub map_update_: Condvar,
    // 后端运行标志
    pub backend_running_: AtomicBool,
    // 左相机的智能指针
    pub cam_left_: Option<Arc<Camera>>,
    // 右相机的智能指针
    pub cam_right_: Option<Arc<Camera>>,
}

impl Backend {
    
    /// 构造函数，启动优化线程并挂起
    pub fn new(map: Arc<RwLock<Map>>) -> Self {
        let data_mutex = Mutex::new(());
        let map_update = Condvar::new();
        let backend_running = AtomicBool::new(true);

        let mut backend = Backend {
            map_: map,
            backend_thread_: None,
            data_mutex_: data_mutex.lock().unwrap().clone().into(),
            map_update_: map_update,
            backend_running_: backend_running,
            cam_left_: None,
            cam_right_: None,
        };
        
        // TODO 修复backend_loop()及多线程访问

        // let handle = std::thread::spawn({
        //     let data_mutex = data_mutex.lock().unwrap().clone();
        //     let map_update = map_update;
        //     let backend_running = backend_running;
        //     let map = backend.map_.clone();
        //     move || {
        //         let mut local_backend = Backend {
        //             map_: map,
        //             backend_thread_: None,
        //             data_mutex_: data_mutex.into(),
        //             map_update_: map_update,
        //             backend_running_: backend_running,
        //             cam_left_: None,
        //             cam_right_: None,
        //         };
               
                // local_backend.backend_loop();
        //     }
        // });

        // backend.backend_thread_ = Some(handle.thread().clone());

        // 返回值
        backend
    }

    /// 设置左右相机，用于获得内外参
    pub fn set_cameras(&mut self, left: Arc<Camera>, right: Arc<Camera>) {
        self.cam_left_ = Some(left);
        self.cam_right_ = Some(right);
    }

    /// 设置地图
    pub fn set_map(&mut self, map: Arc<RwLock<Map>>) {
        self.map_ = map;
    }

    /// 触发地图更新，启动优化
    pub fn update_map(&self) {
        let _lock = self.data_mutex_.lock().unwrap();
        self.map_update_.notify_one();
    }

//     /// 关闭后端线程
//     pub fn stop(&self) {
//         self.backend_running_.store(false, Ordering::SeqCst);
//         self.map_update_.notify_one();
//         if let Some(thread) = self.backend_thread_.as_ref() {
//             thread.join().unwrap();
//         }
//     }

//     /// 后端线程
//     fn backend_loop(&mut self) {
//         while self.backend_running_.load(Ordering::SeqCst) {
//             let mut lock = self.data_mutex_.lock().unwrap();
//             lock = self.map_update_.wait(lock).unwrap();

//             // 后端仅优化激活的Frames和Landmarks
//             let map = self.map_.read().unwrap();
//             let active_kfs = map.get_active_key_frames();
//             let active_landmarks = map.get_active_map_points();
//             drop(map);

//             self.optimize(active_kfs, active_landmarks);
//         }
//     }

//     /// 对给定关键帧和路标点进行优化
//     fn optimize(&mut self, keyframes: HashMap<u64, Arc<Frame>>, landmarks: HashMap<u64, Arc<Mutex<MapPoint>>>) {
//         // setup g2o
//         // 这里需要替换为实际的g2o库或者自定义的优化器，暂时省略

//         // pose 顶点，使用Keyframe id
//         let mut vertices: HashMap<u64, Arc<VertexPose>> = HashMap::new();
//         let mut max_kf_id = 0;
//         for (_, kf) in keyframes.iter() {
//             let vertex_pose = Arc::new(VertexPose::new(kf.keyframe_id_ as usize, kf.pose()));
//             if kf.keyframe_id_ > max_kf_id {
//                 max_kf_id = kf.keyframe_id_;
//             }
//             vertices.insert(kf.keyframe_id_, vertex_pose);
//         }

//         // 路标顶点，使用路标id索引
//         let mut vertices_landmarks: HashMap<u64, Arc<VertexXYZ>> = HashMap::new();

//         // K 和左右外参
//         let K = self.cam_left_.as_ref().unwrap().K();
//         let left_ext = self.cam_left_.as_ref().unwrap().pose();
//         let right_ext = self.cam_right_.as_ref().unwrap().pose();

//         // edges
//         let mut index = 1;
//         let chi2_th = 5.991;  // robust kernel 阈值
//         let mut edges_and_features: HashMap<Arc<EdgeProjection>, Arc<Feature>> = HashMap::new();

//         for (_, landmark) in landmarks.iter() {
//             let landmark_guard = landmark.lock().unwrap();
//             if landmark_guard.is_outlier_ {
//                 continue;
//             }
//             let landmark_id = landmark_guard.id_;
//             let observations = landmark_guard.get_obs();
//             drop(landmark_guard);

//             for obs in observations {
//                 if let Some(feat) = obs.upgrade() {
//                     if feat.is_outlier_ || feat.get_frame().is_none() {
//                         continue;
//                     }

//                     let frame = feat.get_frame().unwrap();
//                     let edge = if feat.is_on_left_image_ {
//                         Arc::new(EdgeProjection::new(frame.keyframe_id_ as usize, landmark_id as usize + max_kf_id as usize + 1, to_vec2(feat.position_.point.into()), K, left_ext))
//                     } else {
//                         Arc::new(EdgeProjection::new(frame.keyframe_id_ as usize, landmark_id as usize + max_kf_id as usize + 1, to_vec2(feat.position_.point.into()), K, right_ext))
//                     };

//                     // 如果landmark还没有被加入优化，则新加一个顶点
//                     if!vertices_landmarks.contains_key(&landmark_id) {
//                         let landmark_guard = landmark.lock().unwrap();
//                         let v = Arc::new(VertexXYZ::new(landmark_id as usize + max_kf_id as usize + 1, landmark_guard.pos()));
//                         drop(landmark_guard);
//                         vertices_landmarks.insert(landmark_id, v);
//                     }

//                     if vertices.contains_key(&frame.keyframe_id_) && vertices_landmarks.contains_key(&landmark_id) {
//                         edges_and_features.insert(edge.clone(), feat.clone());
//                         index += 1;
//                     }
//                 }
//             }
//         }

//         // do optimization and eliminate the outliers
//         // 这里需要替换为实际的优化操作，暂时省略

//         let mut cnt_outlier = 0;
//         let mut cnt_inlier = 0;
//         let mut iteration = 0;
//         while iteration < 5 {
//             cnt_outlier = 0;
//             cnt_inlier = 0;
//             // determine if we want to adjust the outlier threshold
//             for (edge, _) in edges_and_features.iter() {
//                 // 这里需要替换为实际的chi2计算，暂时省略
//                 if 0.0 > chi2_th {
//                     cnt_outlier += 1;
//                 } else {
//                     cnt_inlier += 1;
//                 }
//             }
//             let inlier_ratio = cnt_inlier as f64 / (cnt_inlier + cnt_outlier) as f64;
//             if inlier_ratio > 0.5 {
//                 break;
//             } else {
//                 chi2_th *= 2.0;
//                 iteration += 1;
//             }
//         }

//         for (edge, feat) in edges_and_features.iter() {
//             // 这里需要替换为实际的chi2计算，暂时省略
//             if 0.0 > chi2_th {
//                 feat.is_outlier_ = true;
//                 if let Some(map_point) = feat.get_map_point() {
//                     let mut map_point_guard = map_point.lock().unwrap();
//                     map_point_guard.remove_observation(feat.clone());
//                 }
//             } else {
//                 feat.is_outlier_ = false;
//             }
//         }

//         // 打印优化中的异常点和内点数量
//         println!("Outlier/Inlier in optimization: {}/{}", cnt_outlier, cnt_inlier);

//         // Set pose and lanrmark position
//         for (id, vertex) in vertices.iter() {
//             if let Some(kf) = keyframes.get(id) {
//                 let mut kf_mut = Arc::get_mut(&mut Arc::clone(kf)).unwrap();
//                 kf_mut.set_pose(&vertex.estimate.read().unwrap());
//             }
//         }
//         for (id, vertex) in vertices_landmarks.iter() {
//             if let Some(landmark) = landmarks.get(id) {
//                 let mut landmark_guard = landmark.lock().unwrap();
//                 landmark_guard.set_pos(&vertex.estimate.read().unwrap());
//             }
//         }
//     }

}

// #[cfg(test)]
// mod tests1 {
//     use super::*;
//     use crate::camera::Camera;
//     use crate::frame::Frame;
//     use crate::map::Map;
//     use crate::mappoint::MapPoint;
//     use crate::feature::Feature;
//     use std::sync::Arc;
//     use std::collections::HashMap;
//     use factrs::linalg::{Vector3, Matrix3};

//     // 辅助函数，用于比较两个 Vector3 是否近似相等
//     fn vector3_approx_eq<T: Numeric>(a: &Vector3<T>, b: &Vector3<T>, abs: T, rel: T) -> bool {
//         let diff = (a - b).norm();
//         let tolerance = rel * a.norm().max(b.norm()) + abs;
//         diff <= tolerance
//     }

//     // 辅助函数，用于比较两个 Matrix3 是否近似相等
//     fn matrix3_approx_eq<T: Numeric>(a: &Matrix3<T>, b: &Matrix3<T>, abs: T, rel: T) -> bool {
//         let mut diff_sum = T::zero();
//         for i in 0..3 {
//             for j in 0..3 {
//                 let diff = (a[(i, j)] - b[(i, j)]).abs();
//                 let tolerance = rel * a[(i, j)].abs().max(b[(i, j)].abs()) + abs;
//                 if diff > tolerance {
//                     return false;
//                 }
//                 diff_sum += diff;
//             }
//         }
//         diff_sum <= rel * a.norm().max(b.norm()) + abs
//     }

//     // 辅助函数，用于比较两个 SE3 是否近似相等
//     fn se3_approx_eq<T: Numeric>(a: &SE3<T>, b: &SE3<T>, abs: T, rel: T) -> bool {
//         let rot_eq = matrix3_approx_eq(&a.rot().to_matrix(), &b.rot().to_matrix(), abs, rel);
//         let xyz_eq = vector3_approx_eq(&a.xyz().into_owned(), &b.xyz().into_owned(), abs, rel);
//         rot_eq && xyz_eq
//     }

//     /// 测试 Backend 结构体的构造函数
//     #[test]
//     fn test_backend_new() {
//         let map = Arc::new(RwLock::new(Map::new()));
//         let backend = Backend::new(map.clone());
//         // 验证后端线程是否正确启动
//         assert!(backend.backend_thread_.is_some());
//         // 验证后端运行标志是否初始化为 true
//         assert!(backend.backend_running_.load(Ordering::SeqCst));
//     }

//     /// 测试设置相机的功能
//     #[test]
//     fn test_backend_set_cameras() {
//         let map = Arc::new(RwLock::new(Map::new()));
//         let backend = Backend::new(map.clone());
//         let left_camera = Arc::new(Camera::new());
//         let right_camera = Arc::new(Camera::new());
//         backend.set_cameras(left_camera.clone(), right_camera.clone());
//         // 验证左相机是否正确设置
//         assert!(backend.cam_left_.is_some());
//         // 验证右相机是否正确设置
//         assert!(backend.cam_right_.is_some());
//     }

//     /// 测试设置地图的功能
//     #[test]
//     fn test_backend_set_map() {
//         let map = Arc::new(RwLock::new(Map::new()));
//         let backend = Backend::new(map.clone());
//         let new_map = Arc::new(RwLock::new(Map::new()));
//         backend.set_map(new_map.clone());
//         // 验证地图是否正确设置
//         assert!(Arc::ptr_eq(&backend.map_, &new_map));
//     }

//     /// 测试触发地图更新的功能
//     #[test]
//     fn test_backend_update_map() {
//         let map = Arc::new(RwLock::new(Map::new()));
//         let backend = Backend::new(map.clone());
//         // 这里只是调用更新方法，无法直接验证是否触发优化，仅验证调用无异常
//         backend.update_map();
//     }

//     /// 测试关闭后端线程的功能
//     #[test]
//     fn test_backend_stop() {
//         let map = Arc::new(RwLock::new(Map::new()));
//         let backend = Backend::new(map.clone());
//         backend.stop();
//         // 验证后端运行标志是否设置为 false
//         assert!(!backend.backend_running_.load(Ordering::SeqCst));
//     }

//     /// 测试后端优化功能（由于涉及复杂的优化过程，这里仅做简单的流程验证）
//     #[test]
//     fn test_backend_optimize() {
//         let map = Arc::new(RwLock::new(Map::new()));
//         let backend = Backend::new(map.clone());
//         let left_camera = Arc::new(Camera::new());
//         let right_camera = Arc::new(Camera::new());
//         backend.set_cameras(left_camera.clone(), right_camera.clone());

//         let frame = Arc::new(Frame::new());
//         let mut map_write = map.write().unwrap();
//         map_write.insert_keyframe(frame.clone());

//         let map_point = Arc::new(Mutex::new(MapPoint::new()));
//         map_write.insert_map_point(map_point.clone());
//         drop(map_write);

//         let keyframes = map.read().unwrap().get_active_key_frames();
//         let landmarks = map.read().unwrap().get_active_map_points();
//         // 这里只是调用优化方法，无法直接验证优化结果，仅验证调用无异常
//         backend.optimize(keyframes, landmarks);
//     }
// }
