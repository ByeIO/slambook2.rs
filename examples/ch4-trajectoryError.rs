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

use std::fs::File;
use std::io::{self, BufRead, BufReader};
use std::path::Path;
use std::thread::sleep;
use std::time::Duration;

use sophus::lie::groups::isometry3::{
    Isometry3F64
};
use sophus::lie::groups::rotation3::Rotation3 as Rotation3F64;

use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, Isometry3, Translation3
};
use three_d::*;

type TrajectoryType = Vec<Isometry3<f64>>;

fn main() {
    // 从文件中读取轨迹
    let groundtruth_file = "./assets/ch4-groundtruth.txt";
    let estimated_file = "./assets/ch4-estimated.txt";
    let groundtruth = read_trajectory(groundtruth_file);
    let estimated = read_trajectory(estimated_file);
    assert!(!groundtruth.is_empty() && !estimated.is_empty());
    assert_eq!(groundtruth.len(), estimated.len());

    // 计算RMSE
    let mut rmse = 0.0;
    for i in 0..estimated.len() {
        let p1 = &estimated[i];
        let p2 = &groundtruth[i];
        // 计算误差
        let error = p2.inverse() * p1;

        // 数据转换:error从`nalgebra::Isometry3<f64>`转为`sophus::lie::groups::isometry3::Isometry3` aka `Isometry3F64`类型(类型名称与nalgebra库冲突)后再进行log运算
        // 提取平移
        let translation = Vector3::from(error.translation.vector);
        // FIXME: 转换可能有错误
        // 提取旋转矩阵
        let rotation_matrix : Matrix3<f64> = error.to_matrix().fixed_view::<3, 3>(0,0).into();
        // 将nalgebra的Matrix3<f64>转换为sophus的Rotation3F64
        let rotation = Rotation3F64::try_from_mat(rotation_matrix).expect("Invalid rotation matrix");
        let sophus_error = Isometry3F64::from_translation_and_rotation(translation, rotation);

        // 计算对数
        let log = sophus_error.log();
        rmse += log.norm_squared();
    }
    rmse /= estimated.len() as f64;
    rmse = rmse.sqrt();
    println!("RMSE = {}", rmse);

    // 绘制轨迹
    draw_trajectory(groundtruth, estimated);
}

// 读取轨迹
fn read_trajectory(path: &str) -> TrajectoryType {
    let file = File::open(path).expect("trajectory file not found.");
    let reader = BufReader::new(file);
    let mut trajectory = Vec::new();
    for line in reader.lines() {
        let line = line.expect("Failed to read line");
        let parts: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
        if parts.len() == 8 {
            // parts[0]为时间戳
            let tx = parts[1];
            let ty = parts[2];
            let tz = parts[3];
            let qx = parts[4];
            let qy = parts[5];
            let qz = parts[6];
            let qw = parts[7];
            // 创建单位四元数
            let quaternion = UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz));
            // 创建平移
            let translation = Translation3::new(tx, ty, tz);
            // 创建李代数
            let isometry = Isometry3::<f64>::from_parts(translation, quaternion);
            // 追加到容器中
            trajectory.push(isometry);
        }
    }
    trajectory
}

// 绘制轨迹
fn draw_trajectory(groundtruth: TrajectoryType, estimated: TrajectoryType) {
    let window = three_d::Window::new(WindowSettings {
        title: "Trajectory Viewer".to_string(),
        max_size: Some((1024, 768)),
        ..Default::default()
    }).unwrap();
    let context = window.gl();
    let mut camera = three_d::Camera::new_perspective(
        window.viewport(),
        vec3(0.0, -0.1, -1.8),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, -1.0, 0.0),
        degrees(45.0),
        0.1,
        1000.0,
    );
    let mut control = three_d::OrbitControl::new(camera.target(), 1.0, 100.0);
    window.render_loop(move |mut frame_input| {
        control.handle_events(&mut camera, &mut frame_input.events);
        frame_input.screen().clear(ClearState::color_and_depth(1.0, 1.0, 1.0, 1.0, 1.0));
        // Draw ground truth trajectory
        for i in 0..groundtruth.len() - 1 {
            let p1 = groundtruth[i].translation.vector;
            let p2 = groundtruth[i + 1].translation.vector;
            draw_line(&context, &camera, &p1, &p2, &Srgba::new(0, 0, 1, 1));
        }
        // Draw estimated trajectory
        for i in 0..estimated.len() - 1 {
            let p1 = estimated[i].translation.vector;
            let p2 = estimated[i + 1].translation.vector;
            draw_line(&context, &camera, &p1, &p2, &Srgba::new(1, 0, 0, 1));
        }
        // Finish the frame
        frame_input.context.programs.write().unwrap();
        three_d::FrameOutput::default()
    });
}

// 辅助函数，用于绘制线段
fn draw_line(context: &Context, camera: &Camera, start: &Vector3<f64>, end: &Vector3<f64>, color: &Srgba) {
    let start = vec2(start.x as f32, start.y as f32);
    let end = vec2(end.x as f32, end.y as f32);
    let mut line = Gm::new(
        Line::new(
            context,
            start,
            end,
            2.0, // 线宽，可以根据需要调整
        ),
        ColorMaterial {
            color: color.clone(),
            ..Default::default()
        },
    );
    line.render(camera, &[]);
}