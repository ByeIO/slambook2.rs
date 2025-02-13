#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_mut)]
#![allow(unused_variables)]

//! 可视化

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::fmt;
use std::sync::{Arc, Mutex, RwLock};
use std::any::type_name;
use std::thread;
use std::time::Duration;

// 绘图和界面库
use three_d::{
    Camera, Context, FrameOutput, Gm, InstancedMesh, Instances, Mesh, OrbitControl, PhysicalMaterial,
    Srgba, Viewport, Window, WindowSettings, degrees, vec3, ClearState, 
};
use three_d::egui::{self, SidePanel, Slider, Checkbox, Label};
use env_logger::init;

// 内部库
use crate::frame::Frame;
use crate::map::Map;
use crate::feature::Feature;
use crate::mappoint::MapPoint;

// 引入image库替代opencv库
use image::{
    GenericImageView, ImageBuffer, Rgb, RgbImage
};

// 定义 Viewer 结构体
#[derive(Debug)]
pub struct Viewer {
    current_frame_: Option<Arc<Frame>>,
    map_: Option<Arc<Map>>,
    viewer_thread_: Option<thread::JoinHandle<()>>,
    viewer_running_: bool,
    active_keyframes_: std::collections::HashMap<u64, Arc<Frame>>,
    active_landmarks_: std::collections::HashMap<u64, Arc<Mutex<MapPoint>>>,
    map_updated_: bool,
    viewer_data_mutex_: Mutex<()>,
}

impl Viewer {
    // 构造函数
    pub fn new() -> Self {
        let viewer = Viewer {
            current_frame_: None,
            map_: None,
            viewer_thread_: None,
            viewer_running_: true,
            active_keyframes_: std::collections::HashMap::new(),
            active_landmarks_: std::collections::HashMap::new(),
            map_updated_: false,
            viewer_data_mutex_: Mutex::new(()),
        };
        
        // TODO 修复thread_loop()
        // viewer.viewer_thread_ = Some(thread::spawn(move || viewer.thread_loop()));
        
        // 返回
        viewer
    }

    // 设置地图
    pub fn set_map(&mut self, map: Arc<Map>) {
        self.map_ = Some(map);
    }

    // 关闭Viewer
    pub fn close(&mut self) {
        self.viewer_running_ = false;
        if let Some(thread) = self.viewer_thread_.take() {
            thread.join().unwrap();
        }
    }

    // 增加当前帧
    pub fn add_current_frame(&mut self, current_frame: Arc<Frame>) {
        let mut lck = self.viewer_data_mutex_.lock().unwrap();
        self.current_frame_ = Some(current_frame);
    }

    // 更新地图
    pub fn update_map(&mut self) {
        let mut lck = self.viewer_data_mutex_.lock().unwrap();
        if let Some(map) = self.map_.as_ref() {
            self.active_keyframes_ = map.get_active_key_frames();
            self.active_landmarks_ = map.get_active_map_points();
            self.map_updated_ = true;
        }
    }

//     // 线程循环
//     fn thread_loop(&mut self) {
//         let window = Window::new(WindowSettings {
//             title: "MySLAM Viewer".to_string(),
//             max_size: Some((1024, 768)),
//             ..Default::default()
//         }).unwrap();
//         let context = window.gl();

//         let mut camera = Camera::new_perspective(
//             window.viewport(),
//             vec3(0.0, -5.0, -10.0),
//             vec3(0.0, 0.0, 0.0),
//             vec3(0.0, -1.0, 0.0),
//             degrees(45.0),
//             0.1,
//             1000.0,
//         );
//         let mut control = OrbitControl::new(camera.target(), 1.0, 100.0);

//         let mut gui = three_d::GUI::new(&context);

//         while window.is_open() && self.viewer_running_ {
//             let mut panel_width = 0.0;
//             gui.update(
//                 &mut window.events(),
//                 window.elapsed_time().as_secs_f32(),
//                 window.viewport(),
//                 window.device_pixel_ratio(),
//                 |gui_context| {
//                     // 这里可以添加更多的GUI元素，比如显示当前帧的信息等
//                     panel_width = gui_context.used_rect().width();
//                 },
//             );

//             camera.set_viewport(Viewport {
//                 x: (panel_width * window.device_pixel_ratio()) as i32,
//                 y: 0,
//                 width: window.viewport().width - (panel_width * window.device_pixel_ratio()) as u32,
//                 height: window.viewport().height,
//             });
//             control.handle_events(&mut camera, &mut window.events());

//             let mut screen = window.screen();
//             screen.clear(ClearState::color_and_depth(1.0, 1.0, 1.0, 1.0, 1.0));

//             let mut lck = self.viewer_data_mutex_.lock().unwrap();
//             if let Some(current_frame) = self.current_frame_.as_ref() {
//                 self.draw_frame(&context, &camera, current_frame, &[0.0, 1.0, 0.0]);
//                 self.follow_current_frame(&mut camera, current_frame);

//                 let img = self.plot_frame_image(current_frame);
//                 // 这里可以将img显示在GUI中，暂时先不实现
//             }

//             if let Some(map) = self.map_.as_ref() {
//                 self.draw_map_points(&context, &camera);
//             }

//             screen.write(|| gui.render()).unwrap();
//             window.swap_buffers();
//             thread::sleep(Duration::from_millis(10));
//         }

//         println!("Stop viewer");
//     }

//     // 绘制帧
//     fn draw_frame(&self, context: &Context, camera: &Camera, frame: &Arc<Frame>, color: &[f32; 3]) {
//         let twc = frame.pose().inverse();
//         let sz = 1.0;
//         let fx = 400.0;
//         let fy = 400.0;
//         let cx = 512.0;
//         let cy = 384.0;
//         let width = 1080.0;
//         let height = 768.0;

//         let vertices = vec![
//             three_d::Vec3::new(0.0, 0.0, 0.0),
//             three_d::Vec3::new(sz * (0.0 - cx) / fx, sz * (0.0 - cy) / fy, sz),
//             three_d::Vec3::new(0.0, 0.0, 0.0),
//             three_d::Vec3::new(sz * (0.0 - cx) / fx, sz * (height - 1.0 - cy) / fy, sz),
//             three_d::Vec3::new(0.0, 0.0, 0.0),
//             three_d::Vec3::new(sz * (width - 1.0 - cx) / fx, sz * (height - 1.0 - cy) / fy, sz),
//             three_d::Vec3::new(0.0, 0.0, 0.0),
//             three_d::Vec3::new(sz * (width - 1.0 - cx) / fx, sz * (0.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (width - 1.0 - cx) / fx, sz * (0.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (width - 1.0 - cx) / fx, sz * (height - 1.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (width - 1.0 - cx) / fx, sz * (height - 1.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (0.0 - cx) / fx, sz * (height - 1.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (0.0 - cx) / fx, sz * (height - 1.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (0.0 - cx) / fx, sz * (0.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (0.0 - cx) / fx, sz * (0.0 - cy) / fy, sz),
//             three_d::Vec3::new(sz * (width - 1.0 - cx) / fx, sz * (0.0 - cy) / fy, sz),
//         ];

//         let mesh = Mesh::new(context, &three_d::CpuMesh::from_positions(vertices));
//         let material = PhysicalMaterial::new(
//             context,
//             &three_d::CpuMaterial {
//                 albedo: Srgba::new((color[0] * 255.0) as u8, (color[1] * 255.0) as u8, (color[2] * 255.0) as u8, 255),
//                 ..Default::default()
//             },
//         );
//         let gm = Gm::new(mesh, material);

//         let m = three_d::Mat4::from(twc);
//         gm.set_transformation(m);
//         let screen = frame.screen();
//         screen.render(&camera, &[&gm], &[]);
//     }

//     // 绘制地图点
//     fn draw_map_points(&self, context: &Context, camera: &Camera) {
//         let red = [1.0, 0.0, 0.0];
//         for (_, kf) in self.active_keyframes_.iter() {
//             self.draw_frame(context, camera, kf, &red);
//         }

//         let mut points_vertices = Vec::new();
//         for (_, landmark) in self.active_landmarks_.iter() {
//             let pos = landmark.lock().unwrap().pos();
//             points_vertices.push(three_d::Vec3::new(pos[0] as f32, pos[1] as f32, pos[2] as f32));
//         }

//         let points_mesh = Mesh::new(context, &three_d::CpuMesh::from_positions(points_vertices));
//         let points_material = PhysicalMaterial::new(
//             context,
//             &three_d::CpuMaterial {
//                 albedo: Srgba::new((red[0] * 255.0) as u8, (red[1] * 255.0) as u8, (red[2] * 255.0) as u8, 255),
//                 ..Default::default()
//             },
//         );
//         let points_gm = Gm::new(points_mesh, points_material);
//         let mut screen = context.screen();
//         screen.render(&camera, &[&points_gm], &[]);
//     }

//     // 跟随当前帧
//     fn follow_current_frame(&self, camera: &mut Camera, frame: &Arc<Frame>) {
//         let twc = frame.pose().inverse();

//         // 从 Twc 中提取位置和方向信息
//         let position = twc.translation.vector;
//         let rotation = twc.rotation;

//         // 将位置和方向信息应用到相机上
//         camera.set_position(three_d::Vec3::new(position.x, position.y, position.z));
//         let forward = rotation * three_d::Vec3::new(0.0, 0.0, -1.0);
//         let up = rotation * three_d::Vec3::new(0.0, 1.0, 0.0);
//         camera.set_target(camera.position() + forward);
//         camera.set_up(up);
//     }

//     // 绘制帧图像
//     fn plot_frame_image(&self, frame: &Arc<Frame>) -> RgbImage {
//         let img = frame.left_img_.clone();
//         let mut img_out = ImageBuffer::new(img.width(), img.height());

//         for feat in frame.features_left_.borrow().iter() {
//             if let Some(map_point) = feat.get_map_point() {
//                 let position = feat.position_.point;
//                 let x = position.0 as u32;
//                 let y = position.1 as u32;
//                 img_out.put_pixel(x, y, Rgb([0, 250, 0]));
//             }
//         }

//         img_out
//     }

} // end impl
