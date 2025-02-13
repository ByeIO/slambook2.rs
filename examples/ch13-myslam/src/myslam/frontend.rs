#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]

//! 前端

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
use crate::viewer::Viewer;
use crate::backend::Backend;

/// 前端状态枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrontendStatus {
    INITING,
    TRACKING_GOOD,
    TRACKING_BAD,
    LOST,
}

/// 前端结构体
#[derive(Debug)]
pub struct Frontend {
    // 前端状态
    pub status_: FrontendStatus,
    // 当前帧
    pub current_frame_: Option<Arc<Frame>>,
    // 上一帧
    pub last_frame_: Option<Arc<Frame>>,
    // 左侧相机
    pub camera_left_: Option<Arc<Camera>>,
    // 右侧相机
    pub camera_right_: Option<Arc<Camera>>,
    // 地图
    pub map_: Option<Arc<Map>>,
    // 后端
    pub backend_: Option<Arc<Backend>>,
    // 查看器
    pub viewer_: Option<Arc<Viewer>>,
    // 当前帧与上一帧的相对运动
    pub relative_motion_: SE3,
    // 内点数量，用于判断是否为新关键帧
    pub tracking_inliers_: usize,
    // 特征点数量参数
    pub num_features_: usize,
    pub num_features_init_: usize,
    pub num_features_tracking_: usize,
    pub num_features_tracking_bad_: usize,
    pub num_features_needed_for_keyframe_: usize,
    // TODO: 这里需要替换为 Rust 中的特征检测器，暂时使用 Option 占位
    pub gftt_: Option<()>,
}

impl Frontend {
    /// 构造函数
    pub fn new() -> Self {
        // TODO: 从配置文件中读取参数
        let num_features = 200;
        let num_features_init = 100;
        let num_features_tracking = 50;
        let num_features_tracking_bad = 20;
        let num_features_needed_for_keyframe = 80;

        Frontend {
            status_: FrontendStatus::INITING,
            current_frame_: None,
            last_frame_: None,
            camera_left_: None,
            camera_right_: None,
            map_: None,
            backend_: None,
            viewer_: None,
            relative_motion_: SE3::identity(),
            tracking_inliers_: 0,
            num_features_: num_features,
            num_features_init_: num_features_init,
            num_features_tracking_: num_features_tracking,
            num_features_tracking_bad_: num_features_tracking_bad,
            num_features_needed_for_keyframe_: num_features_needed_for_keyframe,
            gftt_: None,
        }
    }

//     /// 外部接口，添加一个帧并计算其定位结果
//     pub fn add_frame(&mut self, frame: Arc<Frame>) -> bool {
//         self.current_frame_ = Some(frame);

//         match self.status_ {
//             FrontendStatus::INITING => {
//                 self.stereo_init();
//             }
//             FrontendStatus::TRACKING_GOOD | FrontendStatus::TRACKING_BAD => {
//                 self.track();
//             }
//             FrontendStatus::LOST => {
//                 self.reset();
//             }
//         }

//         self.last_frame_ = self.current_frame_.clone();
//         true
//     }

    /// 设置地图
    pub fn set_map(&mut self, map: Arc<Map>) {
        self.map_ = Some(map);
    }

    /// 设置后端
    pub fn set_backend(&mut self, backend: Arc<Backend>) {
        self.backend_ = Some(backend);
    }

    /// 设置查看器
    pub fn set_viewer(&mut self, viewer: Arc<Viewer>) {
        self.viewer_ = Some(viewer);
    }

    /// 获取前端状态
    pub fn get_status(&self) -> FrontendStatus {
        self.status_
    }

    /// 设置相机
    pub fn set_cameras(&mut self, left: Arc<Camera>, right: Arc<Camera>) {
        self.camera_left_ = Some(left);
        self.camera_right_ = Some(right);
    }

//     /// 正常模式下的跟踪
//     fn track(&mut self) -> bool {
//         if let Some(last_frame) = self.last_frame_.clone() {
//             if let Some(current_frame) = self.current_frame_.clone() {
//                 current_frame.set_pose(&self.relative_motion_ * last_frame.pose());
//             }
//         }

//         let num_track_last = self.track_last_frame();
//         self.tracking_inliers_ = self.estimate_current_pose();

//         if self.tracking_inliers_ > self.num_features_tracking_ {
//             self.status_ = FrontendStatus::TRACKING_GOOD;
//         } else if self.tracking_inliers_ > self.num_features_tracking_bad_ {
//             self.status_ = FrontendStatus::TRACKING_BAD;
//         } else {
//             self.status_ = FrontendStatus::LOST;
//         }

//         self.insert_keyframe();

//         if let (Some(current_frame), Some(last_frame)) = (self.current_frame_.clone(), self.last_frame_.clone()) {
//             self.relative_motion_ = current_frame.pose() * last_frame.pose().inverse();
//         }

//         if let (Some(viewer), Some(current_frame)) = (self.viewer_.clone(), self.current_frame_.clone()) {
//             viewer.add_current_frame(current_frame);
//         }

//         true
//     }

    /// 丢失时的重置
    pub fn reset(&mut self) -> bool {
        println!("Reset is not implemented. ");
        true
    }

//     /// 与上一帧进行跟踪
//     fn track_last_frame(&mut self) -> usize {
//         if let (Some(last_frame), Some(current_frame), Some(camera_left)) = (self.last_frame_.clone(), self.current_frame_.clone(), self.camera_left_.clone()) {
//             let mut kps_last = Vec::new();
//             let mut kps_current = Vec::new();

//             for kp in last_frame.features_left_.read().unwrap().iter() {
//                 if let Some(mp) = kp.get_map_point() {
//                     let px = camera_left.world2pixel(&mp.pos(), &current_frame.pose());
//                     kps_last.push(kp.position_.point);
//                     kps_current.push(image::Point::new(px[0] as i32, px[1] as i32));
//                 } else {
//                     kps_last.push(kp.position_.point);
//                     kps_current.push(kp.position_.point);
//                 }
//             }

//             // TODO: 使用 image 库实现光流跟踪，这里暂时返回 0
//             0
//         } else {
//             0
//         }
//     }

    /// 估计当前帧的位姿
    pub fn estimate_current_pose(&mut self) -> usize {
        // TODO: 实现 g2o 优化，这里暂时返回 0
        0
    }

//     /// 将当前帧设置为关键帧并插入到后端
//     fn insert_keyframe(&mut self) -> bool {
//         if self.tracking_inliers_ >= self.num_features_needed_for_keyframe_ {
//             return false;
//         }

//         if let Some(current_frame) = self.current_frame_.clone() {
//             current_frame.set_key_frame();

//             if let Some(map) = self.map_.clone() {
//                 map.insert_keyframe(current_frame.clone());
//             }

//             println!("Set frame {} as keyframe {}", current_frame.id_, current_frame.keyframe_id_);

//             self.set_observations_for_keyframe();
//             self.detect_features();
//             self.find_features_in_right();
//             self.triangulate_new_points();

//             if let Some(backend) = self.backend_.clone() {
//                 backend.update_map();
//             }

//             if let (Some(viewer), Some(current_frame)) = (self.viewer_.clone(), self.current_frame_.clone()) {
//                 viewer.update_map();
//             }

//             true
//         } else {
//             false
//         }
//     }

//     /// 尝试使用当前帧的立体图像初始化前端
//     fn stereo_init(&mut self) -> bool {
//         let num_features_left = self.detect_features();
//         let num_coor_features = self.find_features_in_right();

//         if num_coor_features < self.num_features_init_ {
//             return false;
//         }

//         let build_map_success = self.build_init_map();

//         if build_map_success {
//             self.status_ = FrontendStatus::TRACKING_GOOD;

//             if let (Some(viewer), Some(current_frame)) = (self.viewer_.clone(), self.current_frame_.clone()) {
//                 viewer.add_current_frame(current_frame);
//                 viewer.update_map();
//             }

//             true
//         } else {
//             false
//         }
//     }

//     /// 在当前帧的左图像中检测特征
//     fn detect_features(&mut self) -> usize {
//         if let Some(current_frame) = self.current_frame_.clone() {
//             // TODO: 使用 image 库实现特征检测，这里暂时返回 0
//             0
//         } else {
//             0
//         }
//     }

//     /// 在当前帧的右图像中找到对应的特征
//     fn find_features_in_right(&mut self) -> usize {
//         if let (Some(current_frame), Some(camera_right)) = (self.current_frame_.clone(), self.camera_right_.clone()) {
//             let mut kps_left = Vec::new();
//             let mut kps_right = Vec::new();

//             for kp in current_frame.features_left_.read().unwrap().iter() {
//                 kps_left.push(kp.position_.point);

//                 if let Some(mp) = kp.get_map_point() {
//                     let px = camera_right.world2pixel(&mp.pos(), &current_frame.pose());
//                     kps_right.push(image::Point::new(px[0] as i32, px[1] as i32));
//                 } else {
//                     kps_right.push(kp.position_.point);
//                 }
//             }

//             // TODO: 使用 image 库实现光流跟踪，这里暂时返回 0
//             0
//         } else {
//             0
//         }
//     }

//     /// 使用单张图像构建初始地图
//     fn build_init_map(&mut self) -> bool {
//         if let (Some(current_frame), Some(camera_left), Some(camera_right), Some(map)) = (self.current_frame_.clone(), self.camera_left_.clone(), self.camera_right_.clone(), self.map_.clone()) {
//             let poses = vec![camera_left.pose(), camera_right.pose()];
//             let mut cnt_init_landmarks = 0;

//             for i in 0..current_frame.features_left_.read().unwrap().len() {
//                 let left_feat = current_frame.features_left_.read().unwrap()[i].clone();
//                 let right_feat = if i < current_frame.features_right_.read().unwrap().len() {
//                     current_frame.features_right_.read().unwrap()[i].clone()
//                 } else {
//                     None
//                 };

//                 if let Some(right_feat) = right_feat {
//                     let points = vec![
//                         camera_left.pixel2camera(&Vec2::new(left_feat.position_.point.x as f64, left_feat.position_.point.y as f64)),
//                         camera_right.pixel2camera(&Vec2::new(right_feat.position_.point.x as f64, right_feat.position_.point.y as f64)),
//                     ];
//                     let mut pworld = Vec3::zeros();

//                     if triangulation(&poses, &points, &mut pworld) && pworld[2] > 0.0 {
//                         let new_map_point = MapPoint::create_new_mappoint();
//                         new_map_point.lock().unwrap().set_pos(&pworld);
//                         new_map_point.lock().unwrap().add_observation(left_feat.clone());
//                         new_map_point.lock().unwrap().add_observation(right_feat.clone());

//                         left_feat.set_map_point(new_map_point.clone());
//                         right_feat.set_map_point(new_map_point.clone());

//                         cnt_init_landmarks += 1;
//                         map.insert_map_point(new_map_point);
//                     }
//                 }
//             }

//             current_frame.set_key_frame();
//             map.insert_keyframe(current_frame);

//             if let Some(backend) = self.backend_.clone() {
//                 backend.update_map();
//             }

//             println!("Initial map created with {} map points", cnt_init_landmarks);

//             true
//         } else {
//             false
//         }
//     }

//     /// 三角化当前帧中的 2D 点
//     pub fn triangulate_new_points(&mut self) -> usize {
//         if let (Some(current_frame), Some(camera_left), Some(camera_right), Some(map)) = (self.current_frame_.clone(), self.camera_left_.clone(), self.camera_right_.clone(), self.map_.clone()) {
//             let poses = vec![camera_left.pose(), camera_right.pose()];
//             let current_pose_Twc = current_frame.pose().inverse();
//             let mut cnt_triangulated_pts = 0;

//             for i in 0..current_frame.features_left_.read().unwrap().len() {
//                 let left_feat = current_frame.features_left_.read().unwrap()[i].clone();
//                 let right_feat = if i < current_frame.features_right_.read().unwrap().len() {
//                     current_frame.features_right_.read().unwrap()[i].clone()
//                 } else {
//                     None
//                 };

//                 if left_feat.get_map_point().is_none() && right_feat.is_some() {
//                     let right_feat = right_feat.unwrap();
//                     let points = vec![
//                         camera_left.pixel2camera(&Vec2::new(left_feat.position_.point.x as f64, left_feat.position_.point.y as f64)),
//                         camera_right.pixel2camera(&Vec2::new(right_feat.position_.point.x as f64, right_feat.position_.point.y as f64)),
//                     ];
//                     let mut pworld = Vec3::zeros();

//                     if triangulation(&poses, &points, &mut pworld) && pworld[2] > 0.0 {
//                         let new_map_point = MapPoint::create_new_mappoint();
//                         pworld = current_pose_Twc * pworld;
//                         new_map_point.lock().unwrap().set_pos(&pworld);
//                         new_map_point.lock().unwrap().add_observation(left_feat.clone());
//                         new_map_point.lock().unwrap().add_observation(right_feat.clone());

//                         left_feat.set_map_point(new_map_point.clone());
//                         right_feat.set_map_point(new_map_point.clone());

//                         map.insert_map_point(new_map_point);
//                         cnt_triangulated_pts += 1;
//                     }
//                 }
//             }

//             println!("new landmarks: {}", cnt_triangulated_pts);
//             cnt_triangulated_pts
//         } else {
//             0
//         }
//     }

//     /// 将关键帧中的特征设置为地图点的新观测
//     fn set_observations_for_keyframe(&mut self) {
//         if let Some(current_frame) = self.current_frame_.clone() {
//             for feat in current_frame.features_left_.read().unwrap().iter() {
//                 if let Some(mp) = feat.get_map_point() {
//                     mp.lock().unwrap().add_observation(feat.clone());
//                 }
//             }
//         }
//     }

}

// #[cfg(test)]
// mod tests2 {
//     use super::*;
//     use crate::frame::Frame;
//     use crate::map::Map;
//     use crate::backend::Backend;
//     use crate::viewer::Viewer;
//     use crate::camera::Camera;
//     use crate::feature::Feature;
//     use crate::mappoint::MapPoint;
//     use std::sync::{Arc, Mutex};
//     use factrs::linalg::{Vec2, Vec3, Mat33, SE3};

//     // 创建一个简单的测试相机
//     fn create_test_camera() -> Arc<Camera> {
//         let k = Mat33::identity();
//         let pose = SE3::identity();
//         Arc::new(Camera::new(k, pose))
//     }

//     // 创建一个简单的测试帧
//     fn create_test_frame() -> Arc<Frame> {
//         let id = 0;
//         let timestamp = 0.0;
//         let left_img = image::GrayImage::new(640, 480);
//         let right_img = image::GrayImage::new(640, 480);
//         let pose = SE3::identity();
//         Arc::new(Frame::new(id, timestamp, left_img, right_img, pose))
//     }

//     // 创建一个简单的测试地图
//     fn create_test_map() -> Arc<Map> {
//         Arc::new(Map::new())
//     }

//     // 创建一个简单的测试后端
//     fn create_test_backend() -> Arc<Backend> {
//         let map = create_test_map();
//         Arc::new(Backend::new(map))
//     }

//     // 创建一个简单的测试查看器
//     fn create_test_viewer() -> Arc<Viewer> {
//         Arc::new(Viewer::new())
//     }

//     /// 测试 Frontend 的构造函数
//     #[test]
//     fn test_frontend_new() {
//         let frontend = Frontend::new();
//         assert_eq!(frontend.status_, FrontendStatus::INITING);
//         assert!(frontend.current_frame_.is_none());
//         assert!(frontend.last_frame_.is_none());
//         assert!(frontend.camera_left_.is_none());
//         assert!(frontend.camera_right_.is_none());
//         assert!(frontend.map_.is_none());
//         assert!(frontend.backend_.is_none());
//         assert!(frontend.viewer_.is_none());
//     }

//     /// 测试 Frontend 的 set_map 方法
//     #[test]
//     fn test_frontend_set_map() {
//         let mut frontend = Frontend::new();
//         let map = create_test_map();
//         frontend.set_map(map.clone());
//         assert!(Arc::ptr_eq(&frontend.map_.unwrap(), &map));
//     }

//     /// 测试 Frontend 的 set_backend 方法
//     #[test]
//     fn test_frontend_set_backend() {
//         let mut frontend = Frontend::new();
//         let backend = create_test_backend();
//         frontend.set_backend(backend.clone());
//         assert!(Arc::ptr_eq(&frontend.backend_.unwrap(), &backend));
//     }

//     /// 测试 Frontend 的 set_viewer 方法
//     #[test]
//     fn test_frontend_set_viewer() {
//         let mut frontend = Frontend::new();
//         let viewer = create_test_viewer();
//         frontend.set_viewer(viewer.clone());
//         assert!(Arc::ptr_eq(&frontend.viewer_.unwrap(), &viewer));
//     }

//     /// 测试 Frontend 的 set_cameras 方法
//     #[test]
//     fn test_frontend_set_cameras() {
//         let mut frontend = Frontend::new();
//         let left_camera = create_test_camera();
//         let right_camera = create_test_camera();
//         frontend.set_cameras(left_camera.clone(), right_camera.clone());
//         assert!(Arc::ptr_eq(&frontend.camera_left_.unwrap(), &left_camera));
//         assert!(Arc::ptr_eq(&frontend.camera_right_.unwrap(), &right_camera));
//     }

//     /// 测试 Frontend 的 add_frame 方法在初始化阶段
//     #[test]
//     fn test_frontend_add_frame_initing() {
//         let mut frontend = Frontend::new();
//         let frame = create_test_frame();
//         let map = create_test_map();
//         let backend = create_test_backend();
//         let viewer = create_test_viewer();
//         let left_camera = create_test_camera();
//         let right_camera = create_test_camera();

//         frontend.set_map(map);
//         frontend.set_backend(backend);
//         frontend.set_viewer(viewer);
//         frontend.set_cameras(left_camera, right_camera);

//         frontend.add_frame(frame.clone());
//         assert!(frontend.current_frame_.is_some());
//         assert!(Arc::ptr_eq(&frontend.current_frame_.unwrap(), &frame));
//     }

//     // 可以继续添加更多的测试用例，例如测试其他方法的功能
//     // 注意：部分方法可能依赖于更复杂的模拟数据和场景，需要进一步完善测试逻辑
// }
