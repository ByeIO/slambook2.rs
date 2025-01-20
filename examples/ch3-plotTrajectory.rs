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

use nalgebra::{
    Matrix3, Vector3, UnitQuaternion, 
    Quaternion, Isometry3, Rotation3, UnitComplex, Rotation2, Unit,
    Translation3, Perspective3, Orthographic3, Vector4, Point3, Const,
    ArrayStorage, Matrix4, ViewStorage
};

use std::fs::File;
use std::io::{
    BufReader, BufRead
};
use std::path::Path;

use three_d::*;
use three_d::egui::*;

// 轨迹文件路径
const TRAJECTORY_FILE: &str = "./assets/ch3-trajectory.txt";
// 绘制轨迹
fn draw_trajectory(poses: Vec<Isometry3<f64>>) {
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
    let mut gui = GUI::new(&context);
    window.render_loop(move |mut frame_input| {
        let mut panel_width = 0.0;
        gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |gui_context| {
                // ... (可以添加EGUI面板代码 here)
            },
        );
        camera.set_viewport(Viewport {
            x: (panel_width * frame_input.device_pixel_ratio) as i32,
            y: 0,
            width: frame_input.viewport.width - (panel_width * frame_input.device_pixel_ratio) as u32,
            height: frame_input.viewport.height,
        });
        control.handle_events(&mut camera, &mut frame_input.events);
        frame_input.screen().clear(ClearState::color_and_depth(1.0, 1.0, 1.0, 1.0, 1.0));

        // 修改函数参数类型为 3x1 矩阵
        fn convert_to_vector2(vec3: &nalgebra::Matrix<f64, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f64, 3, 1>>) -> three_d::Vector2<f32> {
            Vector2::new(vec3[(0, 0)] as f32, vec3[(1, 0)] as f32)
        }

        // 遍历pose数据
        for pose in &poses {
            let origin = pose.translation.vector;
            let x_axis = pose * Vector3::new(0.1, 0.0, 0.0);
            let y_axis = pose * Vector3::new(0.0, 0.1, 0.0);
            let z_axis = pose * Vector3::new(0.0, 0.0, 0.1);
            // 绘制坐标轴
            draw_line(&context, &camera, &convert_to_vector2(&origin), &convert_to_vector2(&x_axis), &Srgba::new(1, 0, 0, 1));
            draw_line(&context, &camera, &convert_to_vector2(&origin), &convert_to_vector2(&y_axis), &Srgba::new(0, 1, 0, 1));
            draw_line(&context, &camera, &convert_to_vector2(&origin), &convert_to_vector2(&z_axis), &Srgba::new(0, 0, 1, 1));
        }
        // 绘制轨迹连线
        for i in 0..poses.len() - 1 {
            let p1 = poses[i].translation.vector;
            let p2 = poses[i + 1].translation.vector;
            draw_line(&context, &camera, &convert_to_vector2(&p1), &convert_to_vector2(&p2), &Srgba::new(0, 0, 0, 1));
        }
        frame_input.context.programs.write().unwrap();
        FrameOutput::default()
    });
}

fn main() {
    let mut poses = Vec::new();
    let file_path = Path::new(TRAJECTORY_FILE);
    let file = File::open(file_path).expect("Cannot find trajectory file");
    let reader = BufReader::new(file);
    for line in reader.lines() {
        let line = line.unwrap();
        let parts: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
        if parts.len() == 8 {
            let time = parts[0];
            let tx = parts[1];
            let ty = parts[2];
            let tz = parts[3];
            let qx = parts[4];
            let qy = parts[5];
            let qz = parts[6];
            let qw = parts[7];
            // 获取四元数
            let quaternion = Quaternion::new(qw, qx, qy, qz);
            // 获取平移向量
            let translation = Vector3::new(tx, ty, tz);
            // 确保四元数是单位的
            let unit_quaternion = UnitQuaternion::from_quaternion(quaternion.normalize());
            // 直接使用 UnitQuaternion 来构造 Isometry3
            let isometry = Isometry3::from_parts(nalgebra::Translation { vector: translation }, unit_quaternion);
            poses.push(isometry);
        }
    }
    println!("Read total {} pose entries", poses.len());
    // 绘制轨迹
    draw_trajectory(poses);
}

// 辅助函数，用于绘制线段
fn draw_line(context: &three_d::Context, camera: &Camera, start: &Vector2<f32>, end: &Vector2<f32>, color: &Srgba) {
    let mut line = Gm::new(
        Line::new(
            context,
            *start,
            *end,
            5.0, // 线宽，可以根据需要调整
        ),
        ColorMaterial {
            color: color.clone(),
            ..Default::default()
        },
    );
    line.render(camera, &[]);
}