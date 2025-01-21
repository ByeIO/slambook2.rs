#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] 
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]
#![allow(unused_must_use)]
#![allow(non_snake_case)]

use std::fs::File;
use std::io::{
    self, BufRead, BufReader
};
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use sophus::lie::groups::isometry3::{
    Isometry3F64
};
// 设置别名防止冲突
use sophus::lie::groups::rotation3::Rotation3 as Rotation3F64;

use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, 
    Isometry3, Translation3, Const,
    Matrix, Vector6, Point3
};

// 绘图
use three_d::*;

// 图像处理
use image::{
    GenericImageView, ImageBuffer, Rgba, RgbaImage, 
    DynamicImage, ImageFormat, imageops::FilterType
};

// 轨迹数据类型,使用nalgebra库的Isometry3
type TrajectoryType = Vec<Isometry3<f64>>;

// 6x1矩阵,使用nalgebra库的Vector6
type Vector6d = Vector6<f64>;

fn main() {
    // 读取相机位姿
    let poses = read_poses("./assets/ch5-pose.txt");

    // 获取图片及深度信息
    let (color_imgs, depth_imgs) = load_images(5);

    // 计算点云
    let pointcloud = compute_pointcloud(&color_imgs, &depth_imgs, &poses);

    // 显示点云
    show_point_cloud(pointcloud);
}

// 读取相机位姿
fn read_poses(path: &str) -> TrajectoryType {
    let file = File::open(path).expect("pose.txt not found.");
    let reader = BufReader::new(file);
    let mut poses = Vec::new();

    for line in reader.lines() {
        let line = line.expect("Failed to read line");
        let data: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
        if data.len() == 7 {
            // 创建单位四元数
            let quaternion = UnitQuaternion::from_quaternion(Quaternion::new(data[6], data[3], data[4], data[5]));
            // 创建平移
            let translation = Translation3::new(data[0], data[1], data[2]);
            // 创建李代数
            let isometry = Isometry3::<f64>::from_parts(translation, quaternion);
            poses.push(isometry);
        }
    }
    // 返回位姿
    poses
}

// 加载图像
fn load_images(num: usize) -> (Vec<DynamicImage>, Vec<DynamicImage>) {
    let mut color_imgs = Vec::new();
    let mut depth_imgs = Vec::new();

    for i in 0..num {
        let color_path = format!("./assets/ch5-color/{}.png", i + 1);
        let depth_path = format!("./assets/ch5-depth/{}.pgm", i + 1);
        color_imgs.push(image::open(&color_path).expect("Failed to load color image"));
        depth_imgs.push(image::open(&depth_path).expect("Failed to load depth image"));
    }
    (color_imgs, depth_imgs)
}

// 计算点云
fn compute_pointcloud(color_imgs: &Vec<DynamicImage>, depth_imgs: &Vec<DynamicImage>, poses: &TrajectoryType) -> Vec<Vector6d> {
    let cx = 325.5;
    let cy = 253.5;
    let fx = 518.0;
    let fy = 519.0;
    let depth_scale = 1000.0;
    let mut pointcloud = Vec::new();

    for (i, color) in color_imgs.iter().enumerate() {
        let depth = &depth_imgs[i];
        let T = poses[i];

        for v in 0..color.height() {
            for u in 0..color.width() {
                // 获取深度值
                let d = depth.get_pixel(u, v)[0]; 
                // 深度值为0表示没有测量到
                if d == 0 { continue; } 

                let mut point = Vector3::zeros();
                point[2] = d as f64 / depth_scale;
                point[0] = (u as f64 - cx) * point[2] / fx;
                point[1] = (v as f64 - cy) * point[2] / fy;

                let point_world = T * point;

                let mut p = Vector6d::zeros();

                // 从point_world中复制数据到6x1矩阵
                p.fixed_rows_mut::<3>(0).copy_from(&point_world.into());

                let color_data = color.get_pixel(u, v);
                // red
                p[3] = color_data[2] as f64; 
                // green
                p[4] = color_data[1] as f64; 
                // blue
                p[5] = color_data[0] as f64; 
                pointcloud.push(p);
            }
        }
    }
    pointcloud
}

// 显示点云
fn show_point_cloud(pointcloud: Vec<Vector6d>) {
    if pointcloud.is_empty() {
        eprintln!("Point cloud is empty!");
        return;
    }

    // 打印点云数据
    // println!("{:?}",pointcloud);
    
    // 1. 创建窗口
    let window = three_d::Window::new(WindowSettings {
        title: "Point Cloud!".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();
    let context = window.gl();

    // 2. 创建观察相机
    let mut camera = three_d::Camera::new_perspective(
        window.viewport(),
        vec3(0.125, -0.25, -0.5),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.01,
        100.0,
    );
    let mut control = three_d::OrbitControl::new(camera.target(), 0.1, 3.0);

    // 3. 解析点云数据
    let mut positions = Vec::new();
    let mut colors = Vec::new();

    for point in pointcloud {
        positions.push(vec3(point[0] as f32, point[1] as f32, point[2] as f32));
        colors.push(three_d::Srgba::new_opaque((point[3] / 255.0) as u8, (point[4] / 255.0) as u8, (point[5] / 255.0) as u8));
    }

    let cpu_mesh = three_d::CpuMesh {
        positions: three_d::Positions::F32(positions),
        colors: Some(colors),
        ..Default::default()
    };

    let mut point_cloud = Gm {
        geometry: Mesh::new(&context, &cpu_mesh),
        material: ColorMaterial::default(),
    };

    let c = -point_cloud.aabb().center();
    point_cloud.set_transformation(three_d::Mat4::from_translation(c));

    // 4. 创建渲染循环
    window.render_loop(move |mut frame_input| {
        let mut redraw = frame_input.first_frame;
        redraw |= camera.set_viewport(frame_input.viewport);
        redraw |= control.handle_events(&mut camera, &mut frame_input.events);

        if redraw {
            frame_input
                .screen()
                .clear(ClearState::color_and_depth(1.0, 1.0, 1.0, 1.0, 1.0))
                .render(&camera, &point_cloud, &[]);
        }

        FrameOutput {
            swap_buffers: redraw,
            ..Default::default()
        }
    }); // end render_loop

}