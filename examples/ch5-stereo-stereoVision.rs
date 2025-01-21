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

/*
** 功能: 双目视觉视差图点云
*/

use std::fs::File;
use std::io::{self, BufRead, BufReader};
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;
use std::cmp;

use nalgebra::{
    Vector4, Vector3, Point3, Matrix3, Quaternion, 
    UnitQuaternion, Isometry3, Translation3, Const
};

use image::{
    GenericImageView, ImageBuffer, Rgba, RgbaImage, 
    DynamicImage, ImageFormat, imageops::FilterType,
    GrayImage, Luma
};

use three_d::{
    Window, Camera, OrbitControl, CpuMesh, Positions,
    Gm, Mesh, ColorMaterial, ClearState, FrameOutput,
    Geometry, Mat4, Srgba, vec3, degrees, WindowSettings
};

// 文件路径
const LEFT_FILE: &str = "./assets/ch5-left.png";
const RIGHT_FILE: &str = "./assets/ch5-right.png";

fn main() {
    // 内参
    let fx = 718.856;
    let fy = 718.856;
    let cx = 607.1928;
    let cy = 185.2157;
    // 基线
    let b = 0.573;

    // 读取图像
    let left = image::open(LEFT_FILE).expect("Failed to load left image").to_luma8();
    let right = image::open(RIGHT_FILE).expect("Failed to load right image").to_luma8();

    // 使用StereoSGBM算法计算视差图
    let disparity = compute_disparity(&left, &right);

    // 生成点云
    let pointcloud = compute_pointcloud(&left, &disparity, fx, fy, cx, cy, b);

    // 显示点云
    show_point_cloud(pointcloud);
}

// 计算视差图
fn compute_disparity(left: &GrayImage, right: &GrayImage) -> GrayImage {
    let (width, height) = left.dimensions();
    // 最大视差范围
    let max_disparity = 64; 
    let mut disparity_map = GrayImage::new(width, height);

    for y in 0..height {
        for x in 0..width {
            let mut min_sad = u32::MAX;
            let mut best_disparity = 0;

            for d in 0..max_disparity {
                if x >= d {
                    let mut sad = 0;

                    // 计算SAD(绝对差值之和)
                    for i in -1..=1 {
                        for j in -1..=1 {
                            let lx = x as i32 + i;
                            let ly = y as i32 + j;
                            let rx = (x as i32 - d as i32) + i;
                            let ry = y as i32 + j;

                            if lx >= 0 && lx < width as i32 && ly >= 0 && ly < height as i32 &&
                               rx >= 0 && rx < width as i32 && ry >= 0 && ry < height as i32 {
                                let left_pixel = left.get_pixel(lx as u32, ly as u32)[0];
                                let right_pixel = right.get_pixel(rx as u32, ry as u32)[0];
                                sad += (left_pixel as i32 - right_pixel as i32).abs() as u32;
                            }
                        }
                    }

                    if sad < min_sad {
                        min_sad = sad;
                        best_disparity = d;
                    }
                }
            }

            disparity_map.put_pixel(x, y, Luma([best_disparity as u8]));
        }
    }

    // 视差优化：中值滤波
    let filtered_disparity_map = median_filter(&disparity_map, 3);
    filtered_disparity_map
}

// 中值滤波
fn median_filter(image: &GrayImage, window_size: u32) -> GrayImage {
    let (width, height) = image.dimensions();
    let mut filtered_image = GrayImage::new(width, height);
    let half_window = window_size as i32 / 2;

    for y in 0..height {
        for x in 0..width {
            let mut values = Vec::new();

            for i in -half_window..=half_window {
                for j in -half_window..=half_window {
                    let nx = x as i32 + i;
                    let ny = y as i32 + j;

                    if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                        values.push(image.get_pixel(nx as u32, ny as u32)[0]);
                    }
                }
            }

            values.sort();
            let median = values[values.len() / 2];
            filtered_image.put_pixel(x, y, Luma([median]));
        }
    }

    filtered_image
}


// 计算点云
fn compute_pointcloud(
    left: &image::GrayImage,
    disparity: &image::GrayImage,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    b: f64,
) -> Vec<Vector4<f64>> {
    let mut pointcloud = Vec::new();

    for v in 0..left.height() {
        for u in 0..left.width() {
            let d = disparity.get_pixel(u, v)[0] as f64;
            if d <= 0.0 || d >= 96.0 {
                continue;
            }

            let mut point = Vector4::zeros();
            let depth = fx * b / d;

            point[0] = (u as f64 - cx) / fx * depth;
            point[1] = (v as f64 - cy) / fy * depth;
            point[2] = depth;
            point[3] = left.get_pixel(u, v)[0] as f64 / 255.0;

            pointcloud.push(point);
        }
    }

    pointcloud
}

// 显示点云
fn show_point_cloud(pointcloud: Vec<Vector4<f64>>) {
    if pointcloud.is_empty() {
        eprintln!("Point cloud is empty!");
        return;
    }

    // 创建窗口
    let window = Window::new(WindowSettings {
        title: "Point Cloud Viewer".to_string(),
        max_size: Some((1024, 768)),
        ..Default::default()
    })
    .unwrap();
    let context = window.gl();

    // 创建观察相机
    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(0.0, -0.1, -1.8),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, -1.0, 0.0),
        degrees(45.0),
        0.1,
        1000.0,
    );
    let mut control = OrbitControl::new(camera.target(), 0.1, 3.0);

    // 解析点云数据
    let mut positions = Vec::new();
    let mut colors = Vec::new();

    for point in pointcloud {
        positions.push(vec3(point[0] as f32, point[1] as f32, point[2] as f32));
        colors.push(Srgba::new_opaque(
            (point[3] * 255.0) as u8,
            (point[3] * 255.0) as u8,
            (point[3] * 255.0) as u8,
        ));
    }

    let cpu_mesh = CpuMesh {
        positions: Positions::F32(positions),
        colors: Some(colors),
        ..Default::default()
    };

    let mut point_cloud = Gm {
        geometry: Mesh::new(&context, &cpu_mesh),
        material: ColorMaterial::default(),
    };

    let c = -point_cloud.aabb().center();
    point_cloud.set_transformation(Mat4::from_translation(c));

    // 创建渲染循环
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
    });
}