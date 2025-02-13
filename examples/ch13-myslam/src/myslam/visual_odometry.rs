#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]

//! 视觉里程计

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
use crate::frontend::Frontend;
use crate::dataset::Dataset;
use crate::config::Config;

/// 视觉里程计类，对外提供视觉里程计的功能接口
#[derive(Debug)]
pub struct VisualOdometry {
    // 是否已经初始化的标志
    pub inited: bool,
    // 配置文件的路径
    pub config_file_path: String,
    // 前端处理模块
    pub frontend: Option<Arc<Mutex<Frontend>>>,
    // 后端处理模块
    pub backend: Option<Arc<Mutex<Backend>>>,
    // 地图模块
    pub map: Option<Arc<Mutex<Map>>>,
    // 可视化模块
    pub viewer: Option<Arc<Mutex<Viewer>>>,
    // 数据集模块
    pub dataset: Option<Arc<Mutex<Dataset>>>,
}

impl VisualOdometry {
    
    /// 构造函数，接受配置文件路径作为参数
    pub fn new(config_path: String) -> Self {
        VisualOdometry {
            inited: false,
            config_file_path: config_path,
            frontend: None,
            backend: None,
            map: None,
            viewer: None,
            dataset: None,
        }
    }
    
    // /// 初始化函数，在运行视觉里程计之前进行初始化操作
    // /// 返回 true 表示初始化成功，false 表示失败
    // pub fn init(&mut self) -> bool {
    //     // 从配置文件中读取参数
    //     if Config::set_parameter_file(&self.config_file_path) == false {
    //         return false;
    //     }

    //     // 创建数据集实例
    //     let dataset_dir = Config::get::<String>("dataset_dir");
    //     let dataset = Arc::new(Mutex::new(Dataset::new(dataset_dir)));
    //     if dataset.lock().unwrap().init() == false {
    //         return false;
    //     }
    //     self.dataset = Some(dataset.clone());

    //     // 创建各个模块的实例
    //     let frontend = Arc::new(Mutex::new(Frontend::new()));
    //     let backend = Arc::new(Mutex::new(Backend::new()));
    //     let map = Arc::new(Mutex::new(Map::new()));
    //     let viewer = Arc::new(Mutex::new(Viewer::new()));

    //     // 设置各个模块之间的关联
    //     frontend.lock().unwrap().set_backend(backend.clone());
    //     frontend.lock().unwrap().set_map(map.clone());
    //     frontend.lock().unwrap().set_viewer(viewer.clone());
    //     let cam0 = dataset.lock().unwrap().get_camera(0);
    //     let cam1 = dataset.lock().unwrap().get_camera(1);
    //     frontend.lock().unwrap().set_cameras(cam0, cam1);

    //     backend.lock().unwrap().set_map(map.clone());
    //     backend.lock().unwrap().set_cameras(cam0, cam1);

    //     viewer.lock().unwrap().set_map(map.clone());

    //     self.frontend = Some(frontend);
    //     self.backend = Some(backend);
    //     self.map = Some(map);
    //     self.viewer = Some(viewer);

    //     self.inited = true;
    //     true
    // }

    // /// 启动视觉里程计，开始在数据集上运行
    // pub fn run(&mut self) {
    //     if !self.inited {
    //         if !self.init() {
    //             return;
    //         }
    //     }

    //     loop {
    //         println!("VO is running");
    //         if !self.step() {
    //             break;
    //         }
    //     }

    //     if let Some(backend) = &self.backend {
    //         backend.lock().unwrap().stop();
    //     }
    //     if let Some(viewer) = &self.viewer {
    //         viewer.lock().unwrap().close();
    //     }

    //     println!("VO exit");
    // }

    // /// 处理数据集的下一帧
    // /// 返回 true 表示处理成功，false 表示没有更多帧可处理
    // pub fn step(&self) -> bool {
    //     if let Some(dataset) = &self.dataset {
    //         let new_frame = dataset.lock().unwrap().next_frame();
    //         if new_frame.is_none() {
    //             return false;
    //         }
    //         let new_frame = new_frame.unwrap();

    //         let start_time = std::time::Instant::now();
    //         let success = if let Some(frontend) = &self.frontend {
    //             frontend.lock().unwrap().add_frame(new_frame)
    //         } else {
    //             false
    //         };
    //         let end_time = std::time::Instant::now();
    //         let time_used = end_time.duration_since(start_time).as_secs_f64();
    //         println!("VO cost time: {} seconds.", time_used);
    //         success
    //     } else {
    //         false
    //     }
    // }

    // /// 获取前端模块的状态
    // pub fn get_frontend_status(&self) -> FrontendStatus {
    //     if let Some(frontend) = &self.frontend {
    //         frontend.lock().unwrap().get_status()
    //     } else {
    //         FrontendStatus::Initializing
    //     }
    // }


}
